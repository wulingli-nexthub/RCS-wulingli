using GridDemo.RobotModels.Pathfinding;
using GridDemo.Robots;
using System;
using System.Collections.Generic;

namespace GridDemo.MultiRobots
{
    /// <summary>
    /// PathClaimManager：
    /// 职责：
    /// - 管理格子锁抢占板 GridCellClaimBoard；
    /// - 负责路径前缀抢占（ClaimPathToGoalOrPrefix）并回灌给 AutoNavigator；
    /// - 做基本的“互卡/死锁”检测与让步冷却（yield cooldown）；
    /// - 支持“边走边释放”策略：机器人移动出一个格子后释放对应锁。
    ///
    /// 线程模型：
    /// - 仅在持有 RobotWorld.RobotLock 时调用本类的公开方法（约定为 *NoLock* 的意思是：本类内部不加锁）。
    /// </summary>
    internal sealed class PathClaimManager
    {
        private readonly RobotWorld _world;
        private readonly GridCellClaimBoard _claimBoard;

        // 让步冷却：强制某机器人在一段时间内不再扩张 claim，避免抖动
        private const int YieldCooldownFrames = 12;

        // 死锁确认阈值与清理时间
        private const int DeadlockConfirmFrames = 100;
        private const int DeadlockClearFrames = 12;

        // robotId -> 冷却剩余帧数
        private readonly Dictionary<int, int> _yieldCooldownTicks = new Dictionary<int, int>();

        // blockedId -> 死锁信息
        private readonly Dictionary<int, DeadlockInfo> _deadlockByBlockedId = new Dictionary<int, DeadlockInfo>();

        // 边走边释放：记录上一帧所在格
        private readonly Dictionary<int, GridPos> _lastGridCellByRobotId = new Dictionary<int, GridPos>();

        private struct DeadlockInfo
        {
            public int BlockerId;
            public int ConfirmFrames;
            public int LostFrames;
        }

        public PathClaimManager(RobotWorld world)
        {
            _world = world;
            _claimBoard = new GridCellClaimBoard(world.GridCount, world.ObstacleMap);
        }

        /// <summary> 对外暴露格子锁抢占板（供 UI 显示快照等）。 </summary>
        public GridCellClaimBoard ClaimBoard => _claimBoard;

        /// <summary>
        /// 每帧调用：推进让步冷却与死锁关系的“淡出”计数。
        /// 要求：已持有 RobotLock。
        /// </summary>
        public void TickCooling_NoLock()
        {
            // 让步冷却衰减
            if (_yieldCooldownTicks.Count > 0)
            {
                var keys = new List<int>(_yieldCooldownTicks.Keys);
                for (int i = 0; i < keys.Count; i++)
                {
                    int id = keys[i];
                    int t = _yieldCooldownTicks[id] - 1;
                    if (t <= 0)
                        _yieldCooldownTicks.Remove(id);
                    else
                        _yieldCooldownTicks[id] = t;
                }
            }

            // 死锁关系“丢失帧数”自增，到达阈值后清理
            if (_deadlockByBlockedId.Count > 0)
            {
                var deadlockKeys = new List<int>(_deadlockByBlockedId.Keys);
                for (int i = 0; i < deadlockKeys.Count; i++)
                {
                    int blockedId = deadlockKeys[i];
                    var info = _deadlockByBlockedId[blockedId];
                    info.LostFrames++;

                    if (info.LostFrames >= DeadlockClearFrames)
                        _deadlockByBlockedId.Remove(blockedId);
                    else
                        _deadlockByBlockedId[blockedId] = info;
                }
            }
        }

        /// <summary>
        /// 为所有机器人执行“路径格子锁抢占”，并把抢占到的路径前缀写回对应 AutoNavigator。
        /// 抢占优先级：按照“当前格到终点的曼哈顿距离”从近到远，近者优先。
        ///
        /// 另外，在抢占过程中同时进行“互卡检测与打破”：
        /// - 若某机器人长期被另一个机器人堵在下一格，则触发：
        ///   1) 被认为“堵路”的机器人进入让步冷却（YieldCooldownFrames）；
        ///   2) 释放其所有已抢占格子锁；
        ///   3) 双方重建路径。
        ///
        /// 要求：调用方已持有 RobotLock。
        /// </summary>
        public void ApplyPathClaiming_NoLock()
        {
            var robots = _world.Robots;
            int gridCount = _world.GridCount;

            var items = new List<(RobotInstance R, int Dist)>(robots.Count);

            // 1. 收集机器人当前到目标的距离，用于排序优先级
            for (int i = 0; i < robots.Count; i++)
            {
                RobotInstance r = robots[i];
                GridPos? goal = r.AutoNavigator.GetGoalGridSnapshot();
                if (!goal.HasValue)
                {
                    _claimBoard.ReleaseAllByRobot(r.Id);
                    r.AutoNavigator.SetClaimedPathPrefix(null);
                    continue;
                }

                GridPos cur = r.GetGridPos_NoLock();
                int dist = Math.Abs(cur.X - goal.Value.X) + Math.Abs(cur.Y - goal.Value.Y);
                items.Add((r, dist));
            }

            // 近者优先；若距离相同，用 Id 保证确定性
            items.Sort((a, b) =>
            {
                int c = a.Dist.CompareTo(b.Dist);
                if (c != 0) return c;
                return a.R.Id.CompareTo(b.R.Id);
            });

            // 2. 预生成“当前占用格 -> robotId”映射，用于死锁检测
            var occupiedByKey = new Dictionary<int, int>(robots.Count);
            for (int i = 0; i < robots.Count; i++)
            {
                GridPos c = robots[i].GetGridPos_NoLock();
                int key = c.Y * gridCount + c.X;
                if (!occupiedByKey.ContainsKey(key))
                    occupiedByKey.Add(key, robots[i].Id);
            }

            // 关键：把所有机器人“当前占用格”设置为保留格，禁止他人 claim 起点格
            _claimBoard.SetReservedCells(occupiedByKey);

            // 3. 逐机器人执行 claim
            for (int i = 0; i < items.Count; i++)
            {
                RobotInstance r = items[i].R;

                // 3.1 让步冷却期间：该机器人不再扩张 claim（只保留当前位置）
                if (_yieldCooldownTicks.TryGetValue(r.Id, out int cooldown) && cooldown > 0)
                {
                    _claimBoard.ReleaseAllByRobot(r.Id);

                    GridPos curCell = r.GetGridPos_NoLock();
                    var keep = new List<GridPos>(1) { curCell };
                    List<GridPos> claimedKeep = _claimBoard.ClaimPathToGoalOrPrefix(r.Id, keep, 0);
                    r.AutoNavigator.SetClaimedPathPrefix(claimedKeep);
                    continue;
                }

                // 3.2 从 AutoNavigator 拿整条路径，用于基于 claimBoard 生成“可行前缀”
                List<GridPos> path = r.AutoNavigator.GetPathGridSnapshot();
                if ((path == null || path.Count == 0) && r.AutoNavigator.IsEnabled)
                {
                    r.AutoNavigator.RebuildPath();
                    path = r.AutoNavigator.GetPathGridSnapshot();
                }

                if (path == null || path.Count == 0)
                {
                    _claimBoard.ReleaseAllByRobot(r.Id);
                    r.AutoNavigator.SetClaimedPathPrefix(null);
                    continue;
                }

                // 找到路径中与“当前所在格”对应的索引（作为抢占起点）
                GridPos curCell2 = r.GetGridPos_NoLock();
                int startIndex = 0;
                for (int j = 0; j < path.Count; j++)
                {
                    if (path[j].Equals(curCell2))
                    {
                        startIndex = j;
                        break;
                    }
                }

                List<GridPos> claimedPrefix = _claimBoard.ClaimPathToGoalOrPrefix(r.Id, path, startIndex);
                r.AutoNavigator.SetClaimedPathPrefix(claimedPrefix);

                // 4. 死锁/互卡检测
                if (!r.AutoNavigator.IsEnabled)
                    continue;

                int claimedCount = claimedPrefix?.Count ?? 0;
                bool stuckAtStart = claimedCount <= 1;  // 只拿到当前格或更少，说明下一步就被堵

                if (!stuckAtStart)
                    continue;

                int nextIndex = startIndex + 1;
                if (nextIndex < 0 || nextIndex >= path.Count)
                    continue;

                GridPos nextCell = path[nextIndex];
                int nextKey = nextCell.Y * gridCount + nextCell.X;

                if (!occupiedByKey.TryGetValue(nextKey, out int blockerId) || blockerId == r.Id)
                    continue;

                // blocked=r.Id, blocker=blockerId
                DeadlockInfo info;
                if (!_deadlockByBlockedId.TryGetValue(r.Id, out info) || info.BlockerId != blockerId)
                {
                    info = new DeadlockInfo
                    {
                        BlockerId = blockerId,
                        ConfirmFrames = 1,
                        LostFrames = 0
                    };
                }
                else
                {
                    info.ConfirmFrames++;
                    info.LostFrames = 0;
                }

                _deadlockByBlockedId[r.Id] = info;

                if (info.ConfirmFrames < DeadlockConfirmFrames)
                    continue;

                // 达到确认阈值 -> 破局：
                // 1) blocker 进入冷却期；
                // 2) 释放 blocker 的全部 claim；
                // 3) blocker & blocked 双方重建路径。
                _yieldCooldownTicks[blockerId] = YieldCooldownFrames;
                _claimBoard.ReleaseAllByRobot(blockerId);

                RobotInstance blocker = TryGetRobotById_NoLock(blockerId);
                if (blocker != null)
                {
                    blocker.AutoNavigator.SetClaimedPathPrefix(null);
                    blocker.AutoNavigator.RebuildPath();
                }

                r.AutoNavigator.RebuildPath();

                // 触发后重置计数，避免每帧重复触发
                info.ConfirmFrames = 0;
                info.LostFrames = 0;
                _deadlockByBlockedId[r.Id] = info;
            }
        }

        /// <summary>
        /// 边走边释放：当机器人检测到“网格位置发生变化”，就释放其上一个格子的锁。
        /// 要求：调用方已持有 RobotLock，且在 Move.Update() 之后调用。
        /// </summary>
        public void ReleaseClaimByMovement_NoLock(RobotInstance r)
        {
            GridPos current = r.GetGridPos_NoLock();

            if (!_lastGridCellByRobotId.TryGetValue(r.Id, out GridPos last))
            {
                _lastGridCellByRobotId[r.Id] = current;
                return;
            }

            if (!current.Equals(last))
            {
                _claimBoard.ReleaseCell(r.Id, last);
                _lastGridCellByRobotId[r.Id] = current;
            }
        }

        /// <summary>
        /// 清理某个机器人相关的抢占状态：
        /// - 释放其占用的所有格子锁；
        /// - 删除“边走边释放”的历史记录；
        /// - 删除其让步冷却计数与死锁关系记录。
        /// 用途：删除机器人、重置单机时的清理工作。
        /// </summary>
        public void ClearRobotState(int robotId)
        {
            _claimBoard.ReleaseAllByRobot(robotId);
            _lastGridCellByRobotId.Remove(robotId);
            _yieldCooldownTicks.Remove(robotId);

            // 清理所有与该机器人相关的死锁记录
            var toRemove = new List<int>();
            foreach (var kv in _deadlockByBlockedId)
            {
                if (kv.Key == robotId || kv.Value.BlockerId == robotId)
                    toRemove.Add(kv.Key);
            }
            foreach (var k in toRemove)
                _deadlockByBlockedId.Remove(k);
        }

        /// <summary> 内部工具：按 Id 查找机器人（已持有锁）。 </summary>
        private RobotInstance TryGetRobotById_NoLock(int robotId)
        {
            if (robotId < 0) return null;

            var robots = _world.Robots;
            if (robotId < robots.Count && robots[robotId].Id == robotId)
                return robots[robotId];

            for (int i = 0; i < robots.Count; i++)
            {
                if (robots[i].Id == robotId)
                    return robots[i];
            }
            return null;
        }
    }
}