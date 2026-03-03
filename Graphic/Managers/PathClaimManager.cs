using GridDemo.Managers;
using GridDemo.Maps;
using GridDemo.Models;
using GridDemo.Models.Pathfinding;
using System;
using System.Collections.Generic;

namespace GridDemo.Models
{
    /// <summary>
    /// PathClaimManager：
    /// 职责：
    /// - 管理格子锁抢占板 GridCellClaimBoard；
    /// - 负责路径前缀抢占（ClaimPathToGoalOrPrefix）并回灌给 AutoNavigator；
    /// - 做实时"互卡/死锁"诊断：立即分析前方堵路机器人停止原因，
    ///   根据死锁类型（交换位置/十字路口/环形等待）快速决策绕路或等待；
    /// - 支持"边走边释放"策略：机器人移动出一个格子后释放对应锁。
    ///
    /// 线程模型：
    /// - 仅在持有 RobotWorld.RobotLock 时调用本类的公开方法。
    /// </summary>
    internal sealed class PathClaimManager
    {
        private readonly RobotWorld _world;
        private readonly GridCellClaimBoard _claimBoard;

        // 让步冷却：强制某机器人在一段时间内不再扩张 claim，避免抖动
        private const int YieldCooldownFrames = 12;

        // 非死锁场景下的最大等待帧数：超过后执行重规划
        private const int NonDeadlockMaxWaitFrames = 80;

        // 高优先级原地等待后的重规划阈值（长时间不动则强制重规划）
        private const int HighPriorityReplanFrames = 200;

        // robotId -> 冷却剩余帧数
        private readonly Dictionary<int, int> _yieldCooldownTicks = new Dictionary<int, int>();

        // blockedId -> 阻塞跟踪信息
        private readonly Dictionary<int, BlockTrackInfo> _blockTracking = new Dictionary<int, BlockTrackInfo>();

        // wait-for 映射：blockedId -> blockerId（每帧重建）
        private readonly Dictionary<int, int> _waitForMap = new Dictionary<int, int>();

        // 边走边释放：记录上一帧所在格
        private readonly Dictionary<int, GridPos> _lastGridCellByRobotId = new Dictionary<int, GridPos>();

        private struct BlockTrackInfo
        {
            public int BlockerId;
            public int WaitedFrames;
            public EnumDeadlockType LastDeadlockType;
            public bool RerouteTriggered;
        }

        public PathClaimManager(RobotWorld world)
        {
            _world = world;
            _claimBoard = new GridCellClaimBoard(world.GridCount, world.ObstacleMap);
        }

        /// <summary> 对外暴露格子锁抢占板（供 UI 显示快照等）。 </summary>
        public GridCellClaimBoard ClaimBoard => _claimBoard;

        /// <summary>
        /// 每帧调用：推进让步冷却。
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
                    {
                        _yieldCooldownTicks.Remove(id);
                    }
                    else
                    {
                        _yieldCooldownTicks[id] = t;
                    }
                }
            }

            // 阻塞跟踪：自增等待帧数
            if (_blockTracking.Count > 0)
            {
                var trackKeys = new List<int>(_blockTracking.Keys);
                for (int i = 0; i < trackKeys.Count; i++)
                {
                    int id = trackKeys[i];
                    var info = _blockTracking[id];
                    info.WaitedFrames++;
                    _blockTracking[id] = info;
                }
            }
        }

        /// <summary>
        /// 为所有机器人执行"路径格子锁抢占"，并把抢占到的路径前缀写回对应 AutoNavigator。
        ///
        /// 改进逻辑：
        /// 当机器人被堵时，立即分析前方堵路机器人的停止原因：
        ///   - GoalPauseWait / InPlaceTurn / QueueWait
        /// 然后判断是否构成死锁（交换位置/十字路口/wait-for环）：
        ///   - 死锁：低优先级方立即绕路，高优先级方原地等待（超时则重规划）
        ///   - 非死锁：根据停止原因给出短等待窗口，超时后重规划
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
                if (c != 0)
                {
                    return c;
                }
                return a.R.Id.CompareTo(b.R.Id);
            });

            // 2. 预生成"当前占用格 -> robotId"映射
            var occupiedByKey = new Dictionary<int, int>(robots.Count);
            for (int i = 0; i < robots.Count; i++)
            {
                GridPos c = robots[i].GetGridPos_NoLock();
                int key = c.Y * gridCount + c.X;
                if (!occupiedByKey.ContainsKey(key))
                {
                    occupiedByKey.Add(key, robots[i].Id);
                }
            }

            // 把所有机器人"当前占用格"设置为保留格
            _claimBoard.SetReservedCells(occupiedByKey);

            // 每帧重建 waitFor 映射
            _waitForMap.Clear();

            // 记录本帧哪些机器人仍在被堵
            var stillBlockedIds = new HashSet<int>();

            // 3. 逐机器人执行 claim
            for (int i = 0; i < items.Count; i++)
            {
                RobotInstance r = items[i].R;

                // 3.1 让步冷却期间：该机器人不再扩张 claim（只保留当前位置）
                int cooldown;
                if (_yieldCooldownTicks.TryGetValue(r.Id, out cooldown) && cooldown > 0)
                {
                    _claimBoard.ReleaseAllByRobot(r.Id);

                    GridPos curCell = r.GetGridPos_NoLock();
                    var keep = new List<GridPos>(1) { curCell };
                    List<GridPos> claimedKeep = _claimBoard.ClaimPathToGoalOrPrefix(r.Id, keep, 0);
                    r.AutoNavigator.SetClaimedPathPrefix(claimedKeep);
                    continue;
                }

                // 3.2 从 AutoNavigator 拿整条路径
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

                // 找到路径中与"当前所在格"对应的索引
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

                // 4. 实时阻塞诊断
                if (!r.AutoNavigator.IsEnabled)
                {
                    continue;
                }

                int claimedCount = claimedPrefix != null ? claimedPrefix.Count : 0;
                bool stuckAtStart = claimedCount <= 1;

                if (!stuckAtStart)
                {
                    // 不再阻塞，清除跟踪记录
                    _blockTracking.Remove(r.Id);
                    continue;
                }

                int nextIndex = startIndex + 1;
                if (nextIndex < 0 || nextIndex >= path.Count)
                {
                    continue;
                }

                GridPos nextCell = path[nextIndex];
                int nextKey = nextCell.Y * gridCount + nextCell.X;

                int blockerId;
                if (!occupiedByKey.TryGetValue(nextKey, out blockerId) || blockerId == r.Id)
                {
                    continue;
                }

                // 记录 waitFor 关系
                _waitForMap[r.Id] = blockerId;
                stillBlockedIds.Add(r.Id);

                RobotInstance blocker = TryGetRobotById_NoLock(blockerId);
                if (blocker == null)
                {
                    continue;
                }

                // ★ 核心改动：立即诊断前方堵路原因与死锁类型
                BlockDiagnosis diagnosis = DeadlockDiagnoser.Diagnose(
                    r, blocker, gridCount, occupiedByKey, _waitForMap);

                // 更新跟踪信息
                BlockTrackInfo track;
                if (!_blockTracking.TryGetValue(r.Id, out track) || track.BlockerId != blockerId)
                {
                    track = new BlockTrackInfo
                    {
                        BlockerId = blockerId,
                        WaitedFrames = 1,
                        LastDeadlockType = diagnosis.DeadlockType,
                        RerouteTriggered = false
                    };
                }
                else
                {
                    track.LastDeadlockType = diagnosis.DeadlockType;
                }
                _blockTracking[r.Id] = track;

                // ★ 根据诊断结果执行决策
                if (diagnosis.IsDeadlock)
                {
                    HandleDeadlock(r, blocker, diagnosis, track);
                }
                else
                {
                    HandleNonDeadlockBlock(r, blocker, diagnosis, track);
                }
            }

            // 清理不再阻塞的跟踪记录
            var trackKeys2 = new List<int>(_blockTracking.Keys);
            for (int i = 0; i < trackKeys2.Count; i++)
            {
                if (!stillBlockedIds.Contains(trackKeys2[i]))
                {
                    _blockTracking.Remove(trackKeys2[i]);
                }
            }
        }

        /// <summary>
        /// 处理确认为死锁的情况：低优先级方立即绕路，高优先级方等待（超时重规划）
        /// </summary>
        private void HandleDeadlock(RobotInstance self, RobotInstance blocker,
            BlockDiagnosis diagnosis, BlockTrackInfo track)
        {
            if (diagnosis.ShouldReroute && !track.RerouteTriggered)
            {
                // self 是低优先级 → 立即绕路
                _yieldCooldownTicks[self.Id] = YieldCooldownFrames;
                _claimBoard.ReleaseAllByRobot(self.Id);
                self.AutoNavigator.SetClaimedPathPrefix(null);
                self.AutoNavigator.RebuildPath();

                track.RerouteTriggered = true;
                track.WaitedFrames = 0;
                _blockTracking[self.Id] = track;
            }
            else if (diagnosis.ShouldHoldAndWait)
            {
                // self 是高优先级 → 原地等待
                // 长时间不动则强制重规划
                if (track.WaitedFrames > HighPriorityReplanFrames)
                {
                    // 对 blocker（低优先级）施加让步
                    _yieldCooldownTicks[blocker.Id] = YieldCooldownFrames;
                    _claimBoard.ReleaseAllByRobot(blocker.Id);
                    blocker.AutoNavigator.SetClaimedPathPrefix(null);
                    blocker.AutoNavigator.RebuildPath();

                    // 自身也重规划
                    self.AutoNavigator.RebuildPath();

                    track.WaitedFrames = 0;
                    track.RerouteTriggered = false;
                    _blockTracking[self.Id] = track;
                }
            }
        }

        /// <summary>
        /// 处理非死锁阻塞：根据前方停止原因短暂等待，超时后重规划
        /// </summary>
        private void HandleNonDeadlockBlock(RobotInstance self, RobotInstance blocker,
            BlockDiagnosis diagnosis, BlockTrackInfo track)
        {
            // 等待帧数未超过建议值 → 继续等待
            if (track.WaitedFrames <= diagnosis.SuggestedWaitFrames)
            {
                return;
            }

            // 超过建议但未超过最大值 → 继续等待
            if (track.WaitedFrames <= NonDeadlockMaxWaitFrames)
            {
                return;
            }

            // 超过最大等待帧数 → 重规划
            _yieldCooldownTicks[self.Id] = YieldCooldownFrames;
            _claimBoard.ReleaseAllByRobot(self.Id);
            self.AutoNavigator.SetClaimedPathPrefix(null);
            self.AutoNavigator.RebuildPath();

            track.WaitedFrames = 0;
            track.RerouteTriggered = false;
            _blockTracking[self.Id] = track;
        }

        /// <summary>
        /// 边走边释放：当机器人检测到"网格位置发生变化"，就释放其上一个格子的锁。
        /// 要求：调用方已持有 RobotLock，且在 Move.Update() 之后调用。
        /// </summary>
        public void ReleaseClaimByMovement_NoLock(RobotInstance r)
        {
            GridPos current = r.GetGridPos_NoLock();

            GridPos last;
            if (!_lastGridCellByRobotId.TryGetValue(r.Id, out last))
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
        /// 清理某个机器人相关的抢占状态。
        /// </summary>
        public void ClearRobotState(int robotId)
        {
            _claimBoard.ReleaseAllByRobot(robotId);
            _lastGridCellByRobotId.Remove(robotId);
            _yieldCooldownTicks.Remove(robotId);
            _blockTracking.Remove(robotId);

            // 清理 waitFor 中相关记录
            var toRemoveWait = new List<int>();
            foreach (var kv in _waitForMap)
            {
                if (kv.Key == robotId || kv.Value == robotId)
                {
                    toRemoveWait.Add(kv.Key);
                }
            }
            for (int i = 0; i < toRemoveWait.Count; i++)
            {
                _waitForMap.Remove(toRemoveWait[i]);
            }

            // 清理其他机器人对该机器人的阻塞跟踪
            var toRemoveTrack = new List<int>();
            foreach (var kv in _blockTracking)
            {
                if (kv.Value.BlockerId == robotId)
                {
                    toRemoveTrack.Add(kv.Key);
                }
            }
            for (int i = 0; i < toRemoveTrack.Count; i++)
            {
                _blockTracking.Remove(toRemoveTrack[i]);
            }
        }

        /// <summary> 内部工具：按 Id 查找机器人（已持有锁）。 </summary>
        private RobotInstance TryGetRobotById_NoLock(int robotId)
        {
            if (robotId < 0)
            {
                return null;
            }

            var robots = _world.Robots;
            if (robotId < robots.Count && robots[robotId].Id == robotId)
            {
                return robots[robotId];
            }

            for (int i = 0; i < robots.Count; i++)
            {
                if (robots[i].Id == robotId)
                {
                    return robots[i];
                }
            }
            return null;
        }
    }
}