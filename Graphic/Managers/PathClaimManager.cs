using GridDemo.Models.Pathfinding;
using GridDemo.Models;
using System;
using System.Collections.Generic;
using GridDemo.Maps;

namespace GridDemo.Models
{
    /// <summary>
    /// PathClaimManager：
    /// 职责：
    /// - 管理格子锁抢占板 GridCellClaimBoard；
    /// - 负责路径前缀抢占（ClaimPathToGoalOrPrefix）并回灌给 AutoNavigator；
    /// - 智能堵塞分析：识别堵路方路径，按优先级即时决策（高优先级等待/低优先级让步绕路）；
    /// - 支持"边走边释放"策略：机器人移动出一个格子后释放对应锁。
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

        // robotId -> 冷却剩余帧数
        private readonly Dictionary<int, int> _yieldCooldownTicks = new Dictionary<int, int>();
        // 判定"处于格心"的世界坐标容差（米），与 AutoNavigator.ArriveEpsilonM 一致
        private const double CellCenterEpsilonM = 0.05;
        // 边走边释放：记录上一帧所在格
        private readonly Dictionary<int, GridPos> _lastGridCellByRobotId = new Dictionary<int, GridPos>();

        public PathClaimManager(RobotWorld world)
        {
            _world = world;
            _claimBoard = new GridCellClaimBoard(world.GridCount, world.ObstacleMap);
        }

        /// <summary> 对外暴露格子锁抢占板（供 UI 显示快照等）。 </summary>
        public GridCellClaimBoard ClaimBoard => _claimBoard;

        /// <summary>
        /// 每帧调用：推进让步冷却计数。
        /// 要求：已持有 RobotLock。
        /// </summary>
        public void TickCooling_NoLock()
        {
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
        }

        /// <summary>
        /// 为所有机器人执行"路径格子锁抢占"，并把抢占到的路径前缀写回对应 AutoNavigator。
        /// 抢占优先级：按照"当前格到终点的曼哈顿距离"从近到远，近者优先。
        ///
        /// 堵塞处理（即时分析，替代旧版150帧死锁确认）：
        /// - 若被堵方发现下一格被另一个机器人占用，立即分析堵路方路径：
        ///   1) 堵路方正在通过该格（路径朝其他方向且前方畅通）→ 被堵方等待；
        ///   2) 迎面冲突或堵路方也被卡住 → 按优先级决策：
        ///      - 被堵方优先级更高 → 堵路方让步（冷却 + 释放锁 + 绕路重规划）；
        ///      - 被堵方优先级更低 → 被堵方自己让步（冷却 + 释放锁 + 绕路重规划）。
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

            // 2. 预生成"当前占用格 -> robotId"映射，用于堵塞检测
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

            // 关键：把所有机器人"当前占用格"设置为保留格，禁止他人 claim 起点格
            _claimBoard.SetReservedCells(occupiedByKey);

            // 构建优先级排序表：robotId -> 在 items 中的排序索引（越小优先级越高）
            var sortOrderById = new Dictionary<int, int>(items.Count);
            for (int idx = 0; idx < items.Count; idx++)
            {
                sortOrderById[items[idx].R.Id] = idx;
            }

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

                // 3.2 从 AutoNavigator 拿整条路径，用于基于 claimBoard 生成"可行前缀"
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

                // 找到路径中与"当前所在格"对应的索引（作为抢占起点）
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

                // 4. 智能堵塞分析与处理
                if (!r.AutoNavigator.IsEnabled)
                {
                    continue;
                }

                int claimedCount = claimedPrefix?.Count ?? 0;
                bool stuckAtStart = claimedCount <= 1;  // 只拿到当前格或更少，说明下一步就被堵

                if (!stuckAtStart)
                {
                    continue;
                }

                // 机器人尚未到达格心 → 先让其完成移动，不在此帧触发堵塞分析
                if (!IsAtCellCenter(r))
                {
                    continue;
                }

                int nextIndex = startIndex + 1;
                if (nextIndex < 0 || nextIndex >= path.Count)
                {
                    continue;
                }

                GridPos nextCell = path[nextIndex];
                int nextKey = nextCell.Y * gridCount + nextCell.X;

                if (!occupiedByKey.TryGetValue(nextKey, out int blockerId) || blockerId == r.Id)
                {
                    continue;
                }

                // 堵路方已经在让步冷却中 → 无需重复处理，等待其冷却结束自然解决
                if (_yieldCooldownTicks.TryGetValue(blockerId, out int blockerCd) && blockerCd > 0)
                {
                    continue;
                }

                RobotInstance blocker = TryGetRobotById_NoLock(blockerId);
                if (blocker == null)
                {
                    continue;
                }

                // ─── 分析堵路原因 ───

                // 判断堵路方是否正在通过被堵格（路径朝其他方向且下一格可走）
                if (IsBlockerPassingThrough(blocker, curCell2, gridCount, occupiedByKey))
                {
                    // 堵路方即将离开该格 → 被堵方原地等待即可，无需干预
                    continue;
                }

                // ─── 优先级决策 ───

                // 获取堵路方在排序表中的位置（越小优先级越高）
                int blockerOrder;
                if (!sortOrderById.TryGetValue(blockerId, out blockerOrder))
                {
                    blockerOrder = int.MaxValue; // 无目标的机器人 → 最低优先级
                }

                int blockedOrder = i; // 被堵方在 items 中的索引

                // 检查是否互堵（堵路方的下一步也是被堵方当前格）
                bool isMutualBlock = IsMutuallyBlocked(blocker, curCell2);

                bool blockedHasHigherPriority = blockedOrder < blockerOrder;

                if (blockedHasHigherPriority || isMutualBlock)
                {
                    // 堵路方让步：进入冷却 + 释放全部锁 + 重建路径绕路
                    _yieldCooldownTicks[blockerId] = YieldCooldownFrames;
                    _claimBoard.ReleaseAllByRobot(blockerId);

                    if (blocker.AutoNavigator.IsEnabled)
                    {
                        blocker.AutoNavigator.RebuildPath();
                    }

                    // 立刻为堵路方重新抢占当前格，保证其能移向格心停稳
                    ReclaimCurrentCell(blocker);

                    // 被堵方也重建路径以适应新布局，并重新抢占
                    _claimBoard.ReleaseAllByRobot(r.Id);
                    r.AutoNavigator.RebuildPath();

                    List<GridPos> newPath = r.AutoNavigator.GetPathGridSnapshot();
                    if (newPath != null && newPath.Count > 0)
                    {
                        GridPos rCell = r.GetGridPos_NoLock();
                        int newStart = 0;
                        for (int j = 0; j < newPath.Count; j++)
                        {
                            if (newPath[j].Equals(rCell)) { newStart = j; break; }
                        }
                        List<GridPos> rClaimed = _claimBoard.ClaimPathToGoalOrPrefix(r.Id, newPath, newStart);
                        r.AutoNavigator.SetClaimedPathPrefix(rClaimed);
                    }
                    else
                    {
                        ReclaimCurrentCell(r);
                    }
                }
                else
                {
                    // 被堵方优先级更低 → 被堵方自己让步
                    _yieldCooldownTicks[r.Id] = YieldCooldownFrames;
                    _claimBoard.ReleaseAllByRobot(r.Id);
                    r.AutoNavigator.RebuildPath();

                    // 立刻为被堵方重新抢占当前格，保证其能移向格心停稳
                    ReclaimCurrentCell(r);
                }
            }
        }

        /// <summary>
        /// 判断堵路方是否正在通过被堵格（即将离开），不需要触发让步。
        /// 条件：
        ///   1) 堵路方有路径且未到终点；
        ///   2) 堵路方路径的下一步 不是 朝向被堵方当前格（非迎面冲突）；
        ///   3) 堵路方路径的下一格 没有被其他机器人占用（前方畅通，可以实际走出去）；
        ///   4) 堵路方没有处于让步冷却中（冷却中无法移动，不算"通过"）。
        /// </summary>
        private bool IsBlockerPassingThrough(
            RobotInstance blocker,
            GridPos blockedCurrentCell,
            int gridCount,
            Dictionary<int, int> occupiedByKey)
        {
            // 堵路方在冷却中（不能扩张 claim），无法实际移动 → 不算"通过"
            if (_yieldCooldownTicks.TryGetValue(blocker.Id, out int cd) && cd > 0)
            {
                return false;
            }

            List<GridPos> blockerPath = blocker.AutoNavigator.GetPathGridSnapshot();
            if (blockerPath == null || blockerPath.Count == 0)
            {
                return false;
            }

            // 找到堵路方在其路径中的当前位置索引
            GridPos blockerPos = blocker.GetGridPos_NoLock();
            int blockerIdx = -1;
            for (int j = 0; j < blockerPath.Count; j++)
            {
                if (blockerPath[j].Equals(blockerPos))
                {
                    blockerIdx = j;
                    break;
                }
            }

            if (blockerIdx < 0)
            {
                return false;
            }

            // 已在路径末尾（到达目标或路径耗尽）→ 不会再走 → 不算"通过"
            if (blockerIdx + 1 >= blockerPath.Count)
            {
                return false;
            }

            GridPos blockerNextCell = blockerPath[blockerIdx + 1];

            // 如果堵路方下一步 = 被堵方当前格 → 迎面冲突，不是"通过"
            if (blockerNextCell.Equals(blockedCurrentCell))
            {
                return false;
            }

            // 检查堵路方的下一格是否也被其他机器人占用（前方也被堵）
            int blockerNextKey = blockerNextCell.Y * gridCount + blockerNextCell.X;
            if (occupiedByKey.TryGetValue(blockerNextKey, out int occupantId) && occupantId != blocker.Id)
            {
                // 堵路方前方也被堵 → 它自己也走不了 → 不算"通过"
                return false;
            }

            // 堵路方路径朝其他方向走且前方畅通 → 正在通过，即将离开
            return true;
        }

        /// <summary>
        /// 检查是否形成互堵：堵路方的路径下一步指向被堵方当前格。
        /// 即 A→B 且 B→A，双方互相挡住对方的下一步。
        /// </summary>
        private bool IsMutuallyBlocked(RobotInstance blocker, GridPos blockedCurrentCell)
        {
            List<GridPos> blockerPath = blocker.AutoNavigator.GetPathGridSnapshot();
            if (blockerPath == null || blockerPath.Count == 0)
            {
                return false;
            }

            GridPos blockerPos = blocker.GetGridPos_NoLock();
            int blockerIdx = -1;
            for (int j = 0; j < blockerPath.Count; j++)
            {
                if (blockerPath[j].Equals(blockerPos))
                {
                    blockerIdx = j;
                    break;
                }
            }

            if (blockerIdx < 0 || blockerIdx + 1 >= blockerPath.Count)
            {
                return false;
            }

            // 堵路方的下一步是被堵方的当前格 → 互堵
            GridPos blockerNextCell = blockerPath[blockerIdx + 1];
            return blockerNextCell.Equals(blockedCurrentCell);
        }

        /// <summary>
        /// 边走边释放：当机器人检测到"网格位置发生变化"，就释放其上一个格子的锁。
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
        /// 判断机器人是否处于其所在格子的中心位置（容差内）。
        /// </summary>
        private bool IsAtCellCenter(RobotInstance r)
        {
            double cellSize = _world.CellSizeM;
            GridPos g = r.GetGridPos_NoLock();
            double cx = g.X * cellSize + cellSize / 2.0;
            double cy = g.Y * cellSize + cellSize / 2.0;
            return Math.Abs(r.X - cx) <= CellCenterEpsilonM
                && Math.Abs(r.Y - cy) <= CellCenterEpsilonM;
        }

        /// <summary>
        /// 为指定机器人重新抢占当前格，确保其能移向格心后停稳。
        /// </summary>
        private void ReclaimCurrentCell(RobotInstance r)
        {
            GridPos cell = r.GetGridPos_NoLock();
            var keep = new List<GridPos>(1) { cell };
            List<GridPos> claimed = _claimBoard.ClaimPathToGoalOrPrefix(r.Id, keep, 0);
            r.AutoNavigator.SetClaimedPathPrefix(claimed);
        }

        /// <summary>
        /// 清理某个机器人相关的抢占状态：
        /// - 释放其占用的所有格子锁；
        /// - 删除"边走边释放"的历史记录；
        /// - 删除其让步冷却计数。
        /// 用途：删除机器人、重置单机时的清理工作。
        /// </summary>
        public void ClearRobotState(int robotId)
        {
            _claimBoard.ReleaseAllByRobot(robotId);
            _lastGridCellByRobotId.Remove(robotId);
            _yieldCooldownTicks.Remove(robotId);
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