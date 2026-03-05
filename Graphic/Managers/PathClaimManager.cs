using GridDemo.Models.Pathfinding;
using GridDemo.Models;
using System;
using System.Collections.Generic;
using GridDemo.Maps;

namespace GridDemo.Models
{
    /// <summary>
    /// PathClaimManager：路径格子锁抢占管理器。
    /// 
    /// 职责：
    /// - 管理格子锁抢占板 <see cref="GridCellClaimBoard"/>，协调多机器人对网格格子的互斥占用；
    /// - 负责路径前缀抢占（<see cref="GridCellClaimBoard.ClaimPathToGoalOrPrefix"/>）并回灌给 <see cref="RobotAutoNavigator"/>；
    /// - 智能堵塞分析：识别堵路方路径，按优先级即时决策（高优先级等待/低优先级让步绕路）；
    /// - 支持"边走边释放"策略：机器人移动出一个格子后释放对应锁，提高格子利用率。
    ///
    /// 核心流程：
    /// 1. 每帧由引擎调用 <see cref="TickCooling_NoLock"/> 递减让步冷却计时；
    /// 2. 每帧由引擎调用 <see cref="ApplyPathClaiming_NoLock"/> 为所有机器人执行路径抢占与堵塞分析；
    /// 3. 每帧由引擎在 Move.Update() 后调用 <see cref="ReleaseClaimByMovement_NoLock"/> 释放已离开的格子锁。
    ///
    /// 线程模型：
    /// - 仅在持有 RobotWorld.RobotLock 时调用本类的公开方法（约定为 *NoLock* 的意思是：本类内部不加锁）。
    /// - 所有内部字典/集合的读写均依赖调用方已持有的外部锁来保证线程安全。
    /// </summary>
    internal sealed class PathClaimManager
    {
        private readonly RobotWorld _world;

        /// <summary>
        /// 格子锁抢占板：记录 cellKey -> robotId 的占用关系，
        /// 以及 robotId -> claimed cells 的路径前缀集合。
        /// </summary>
        private readonly GridCellClaimBoard _claimBoard;

        /// <summary>
        /// 让步冷却帧数常量。
        /// 当某机器人被迫让步后，在接下来的 12 帧内不再扩张 claim（只保留当前格），
        /// 防止让步后立即重新抢占原路径导致两机器人反复互让（抖动现象）。
        /// </summary>
        private const int YieldCooldownFrames = 12;

        /// <summary>
        /// 让步冷却表：robotId -> 剩余冷却帧数。
        /// 值 > 0 时，该机器人处于"冷却期"——只保留当前格的 claim，不会向前扩张路径锁。
        /// 每帧由 <see cref="TickCooling_NoLock"/> 递减，归零时自动移除。
        /// </summary>
        private readonly Dictionary<int, int> _yieldCooldownTicks = new Dictionary<int, int>();

        /// <summary>
        /// 判定"处于格心"的世界坐标容差（单位：米）。
        /// 与 <see cref="RobotAutoNavigator"/> 中的 ArriveEpsilonM 保持一致（0.05m），
        /// 确保 PathClaimManager 和 AutoNavigator 对"机器人已到达格心"的判定标准统一。
        /// </summary>
        private const double CellCenterEpsilonM = 0.01;

        /// <summary>
        /// 边走边释放：记录每个机器人上一帧所在的网格坐标。
        /// 当机器人从一个格子移动到另一个格子时，释放旧格子的锁，
        /// 使后方机器人能更快地抢占到这些格子。
        /// </summary>
        private readonly Dictionary<int, GridPos> _lastGridCellByRobotId = new Dictionary<int, GridPos>();

        // ── 后退让路状态 ──

        /// <summary>
        /// 后退让路状态表：robotId -> <see cref="RetreatState"/>。
        /// 当机器人找不到绕路方案时，会被触发"后退让路"——沿远离 keeper 的方向后退，
        /// 直到 keeper 通过后再恢复正常寻路。
        /// </summary>
        private readonly Dictionary<int, RetreatState> _retreatByRobotId = new Dictionary<int, RetreatState>();

        /// <summary>
        /// 后退让路状态：记录让路方需要给谁让路、以及后退方向。
        /// </summary>
        private struct RetreatState
        {
            /// <summary> 需要让路给谁（keeper 的 robotId）。 </summary>
            public int KeeperId;

            /// <summary> 后退方向 X 分量（-1、0 或 1），表示沿 X 轴的退让方向。 </summary>
            public int RetreatDx;

            /// <summary> 后退方向 Y 分量（-1、0 或 1），表示沿 Y 轴的退让方向。 </summary>
            public int RetreatDy;
        }

        /// <summary>
        /// 构造函数：根据世界实例初始化路径抢占管理器。
        /// </summary>
        /// <param name="world">所属世界实例，用于获取网格尺寸和障碍物地图。</param>
        public PathClaimManager(RobotWorld world)
        {
            _world = world;
            _claimBoard = new GridCellClaimBoard(world.GridCount, world.ObstacleMap);
        }

        /// <summary> 对外暴露格子锁抢占板（供 UI 显示快照等）。 </summary>
        public GridCellClaimBoard ClaimBoard => _claimBoard;

        /// <summary>
        /// 查询某机器人是否正在后退让路中。
        /// 用于外部判断该机器人当前是否处于"让路后退"的特殊状态。
        /// </summary>
        /// <param name="robotId">待查询的机器人 ID。</param>
        /// <returns>true 表示该机器人正在后退让路。</returns>
        public bool IsRetreating(int robotId)
        {
            return _retreatByRobotId.ContainsKey(robotId);
        }

        /// <summary>
        /// 每帧调用：推进让步冷却计数。
        /// 遍历所有处于冷却中的机器人，将剩余帧数 -1；归零则移除冷却记录。
        /// 
        /// 要求：调用方已持有 RobotLock。
        /// </summary>
        public void TickCooling_NoLock()
        {
            if (_yieldCooldownTicks.Count > 0)
            {
                // 复制 key 列表以避免遍历时修改字典引发异常
                var keys = new List<int>(_yieldCooldownTicks.Keys);
                for (int i = 0; i < keys.Count; i++)
                {
                    int id = keys[i];
                    int t = _yieldCooldownTicks[id] - 1;
                    if (t <= 0)
                    {
                        // 冷却结束，移除该机器人的冷却记录，下一帧可恢复正常 claim 扩张
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
        /// 核心方法：为所有机器人执行"路径格子锁抢占"，并把抢占到的路径前缀写回对应 AutoNavigator。
        /// 
        /// 整体流程：
        /// 1. 收集所有有目标的机器人，按"当前格到终点的曼哈顿距离"排序（近者优先抢占）；
        /// 2. 构建"当前占用格 -> robotId"映射，用于快速判断某格被谁占用；
        /// 3. 逐机器人执行路径 claim：
        ///    - 后退让路中的机器人：交由 <see cref="HandleRetreatingRobot"/> 处理；
        ///    - 冷却期中的机器人：只保留当前格，不扩张 claim；
        ///    - 正常机器人：调用 ClaimPathToGoalOrPrefix 抢占尽可能长的路径前缀；
        /// 4. 对抢占失败（被堵在起点）的机器人进行智能堵塞分析：
        ///    - 堵路方即将通过 → 等待；
        ///    - 堵路方停滞/迎面冲突 → 按优先级决策让步方。
        ///
        /// 抢占优先级：按照"当前格到终点的曼哈顿距离"从近到远，近者优先。
        /// 距离相同时，按 robotId 升序保证确定性（避免排序不稳定导致的帧间抖动）。
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

            // (RobotInstance, 曼哈顿距离) 元组列表，用于按优先级排序后逐一 claim
            var items = new List<(RobotInstance R, int Dist)>(robots.Count);

            // ════════════════════════════════════════
            //  阶段 1：收集机器人及其到目标的曼哈顿距离
            // ════════════════════════════════════════
            for (int i = 0; i < robots.Count; i++)
            {
                RobotInstance r = robots[i];
                GridPos? goal = r.AutoNavigator.GetGoalGridSnapshot();
                if (!goal.HasValue)
                {
                    // 没有目标的机器人：释放所有格子锁，清空抢占前缀
                    _claimBoard.ReleaseAllByRobot(r.Id);
                    r.AutoNavigator.SetClaimedPathPrefix(null);
                    continue;
                }

                GridPos cur = r.GetGridPos_NoLock();
                // 曼哈顿距离 = |Δx| + |Δy|，值越小说明越接近目标，优先级越高
                int dist = Math.Abs(cur.X - goal.Value.X) + Math.Abs(cur.Y - goal.Value.Y);
                items.Add((r, dist));
            }

            // 排序规则：距离小的排前面（优先级高）；距离相同则 Id 小的排前面（保证确定性）
            items.Sort((a, b) =>
            {
                int c = a.Dist.CompareTo(b.Dist);
                if (c != 0)
                {
                    return c;
                }
                return a.R.Id.CompareTo(b.R.Id);
            });

            // ════════════════════════════════════════
            //  阶段 2：构建"格子占用映射"与"保留格"
            // ════════════════════════════════════════

            // occupiedByKey：格子键(y*gridCount+x) -> robotId，表示某格当前被哪个机器人物理占用
            // 用于后续堵塞检测时快速查找"某格上是谁"
            var occupiedByKey = new Dictionary<int, int>(robots.Count);
            for (int i = 0; i < robots.Count; i++)
            {
                GridPos c = robots[i].GetGridPos_NoLock();
                int key = c.Y * gridCount + c.X;
                // 同一格子可能有多个机器人（穿模过渡），只记录第一个
                if (!occupiedByKey.ContainsKey(key))
                {
                    occupiedByKey.Add(key, robots[i].Id);
                }
            }

            // 关键：将所有机器人的"当前占用格"设为保留格。
            // 保留格的规则：其他机器人禁止 claim 该格，owner 自己可以 claim。
            // 这样可以防止后优先级的机器人把高优先级机器人的当前格抢走，导致高优先级机器人无法移动。
            _claimBoard.SetReservedCells(occupiedByKey);

            // 构建优先级排序表：robotId -> 排序索引（索引越小优先级越高）
            // 用于堵塞决策时快速比较两个机器人的相对优先级
            var sortOrderById = new Dictionary<int, int>(items.Count);
            for (int idx = 0; idx < items.Count; idx++)
            {
                sortOrderById[items[idx].R.Id] = idx;
            }

            // ════════════════════════════════════════
            //  阶段 3：逐机器人执行路径 claim + 堵塞分析
            // ════════════════════════════════════════
            for (int i = 0; i < items.Count; i++)
            {
                RobotInstance r = items[i].R;

                // ── 3.0 正在后退让路的机器人：交由专门方法处理 ──
                if (_retreatByRobotId.TryGetValue(r.Id, out RetreatState retreatState))
                {
                    HandleRetreatingRobot(r, retreatState, gridCount, occupiedByKey);
                    continue;
                }

                // ── 3.1 让步冷却期间：只保留当前格的 claim，不向前扩张 ──
                // 原因：冷却期内扩张 claim 可能导致与刚让步的对方再次冲突（抖动）
                if (_yieldCooldownTicks.TryGetValue(r.Id, out int cooldown) && cooldown > 0)
                {
                    _claimBoard.ReleaseAllByRobot(r.Id);

                    GridPos curCell = r.GetGridPos_NoLock();
                    var keep = new List<GridPos>(1) { curCell };
                    List<GridPos> claimedKeep = _claimBoard.ClaimPathToGoalOrPrefix(r.Id, keep, 0);
                    r.AutoNavigator.SetClaimedPathPrefix(claimedKeep);
                    continue;
                }

                // ── 3.2 正常情况：获取完整路径并尝试抢占尽可能长的前缀 ──
                List<GridPos> path = r.AutoNavigator.GetPathGridSnapshot();
                // 如果路径为空但导航已启用，触发重建路径（可能是新设了目标还没来得及寻路）
                if ((path == null || path.Count == 0) && r.AutoNavigator.IsEnabled)
                {
                    r.AutoNavigator.RebuildPath();
                    path = r.AutoNavigator.GetPathGridSnapshot();
                }

                if (path == null || path.Count == 0)
                {
                    // 仍然没有路径：释放锁，清空前缀
                    _claimBoard.ReleaseAllByRobot(r.Id);
                    r.AutoNavigator.SetClaimedPathPrefix(null);
                    continue;
                }

                // 在路径中找到"当前所在格"的索引，作为本次 claim 的起始位置。
                // 这样跳过已经走过的路径段，只 claim 当前格及后续格子。
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

                // 调用 ClaimBoard 执行实际抢占：从 startIndex 开始逐格锁定，
                // 遇到被他人占用的格子则停止，返回成功抢到的"路径前缀"
                List<GridPos> claimedPrefix = _claimBoard.ClaimPathToGoalOrPrefix(r.Id, path, startIndex);
                // 将抢到的前缀写回 AutoNavigator，AutoNavigator 只允许沿此前缀发出移动指令
                r.AutoNavigator.SetClaimedPathPrefix(claimedPrefix);

                // ════════════════════════════════════════
                //  阶段 4：智能堵塞分析与处理
                // ════════════════════════════════════════

                // 未启用自动导航的机器人不参与堵塞分析
                if (!r.AutoNavigator.IsEnabled)
                {
                    continue;
                }

                int claimedCount = claimedPrefix?.Count ?? 0;
                // stuckAtStart = true 表示只抢到当前格或更少，下一步就被堵住了
                bool stuckAtStart = claimedCount <= 1;

                if (!stuckAtStart)
                {
                    // 成功抢到了至少2格（当前格+下一格），没有被堵，跳过堵塞分析
                    continue;
                }

                // 机器人尚未到达格心 → 先让其完成当前格内的位移对齐，不在此帧触发堵塞分析。
                // 原因：未到格心时判断"下一格被堵"可能不准确（机器人还在移动中）。
                if (!IsAtCellCenter(r))
                {
                    continue;
                }

                // 取路径中的下一格，检查是谁挡住了
                int nextIndex = startIndex + 1;
                if (nextIndex < 0 || nextIndex >= path.Count)
                {
                    // 已在路径末尾（到达目标），无需堵塞分析
                    continue;
                }

                GridPos nextCell = path[nextIndex];
                int nextKey = nextCell.Y * gridCount + nextCell.X;

                // 查找下一格上的占用者
                if (!occupiedByKey.TryGetValue(nextKey, out int blockerId) || blockerId == r.Id)
                {
                    // 下一格没有被占用，或被自己占用 → 不是被他人堵住，跳过
                    continue;
                }

                // ── 快速跳过已在处理中的堵路方 ──

                // 堵路方已经在让步冷却中 → 它正在让步过程中，无需重复处理
                if (_yieldCooldownTicks.TryGetValue(blockerId, out int blockerCd) && blockerCd > 0)
                {
                    continue;
                }

                // 堵路方已在后退让路中 → 它已经在退让了，无需重复处理
                if (_retreatByRobotId.ContainsKey(blockerId))
                {
                    continue;
                }

                RobotInstance blocker = TryGetRobotById_NoLock(blockerId);
                if (blocker == null)
                {
                    // 堵路方机器人已被删除（异常情况），跳过
                    continue;
                }

                // ─── 分析堵路原因 ───

                // 判断堵路方是否"正在通过"被堵格：
                // 即堵路方的路径朝其他方向走、且前方畅通，意味着它即将离开该格。
                // 此时被堵方只需原地等待，无需触发让步机制。
                if (IsBlockerPassingThrough(blocker, curCell2, gridCount, occupiedByKey))
                {
                    continue;
                }

                // ─── 优先级决策：决定谁让步 ───

                // 获取堵路方在优先级排序表中的位置
                int blockerOrder;
                if (!sortOrderById.TryGetValue(blockerId, out blockerOrder))
                {
                    // 堵路方没有目标（不在 items 列表中）→ 视为最低优先级
                    blockerOrder = int.MaxValue;
                }

                int blockedOrder = i; // 被堵方就是当前遍历的机器人，索引即为其优先级

                // 索引越小优先级越高：被堵方索引 < 堵路方索引 → 被堵方优先级更高
                bool blockedHasHigherPriority = blockedOrder < blockerOrder;

                if (blockedHasHigherPriority)
                {
                    // ── 被堵方优先级更高 → 堵路方让步 ──
                    // 堵路方释放锁 + 绕路或后退
                    HandleYieldOrRetreat(blocker, r, gridCount);

                    // 被堵方在堵路方让步后重新抢占（因为堵路方腾出了格子）
                    _claimBoard.ReleaseAllByRobot(r.Id);
                    r.AutoNavigator.RebuildPath();

                    List<GridPos> newPath = r.AutoNavigator.GetPathGridSnapshot();
                    if (newPath != null && newPath.Count > 0)
                    {
                        // 在新路径中定位当前格作为 claim 起点
                        GridPos rCell = r.GetGridPos_NoLock();
                        int newStart = 0;
                        for (int j = 0; j < newPath.Count; j++)
                        {
                            if (newPath[j].Equals(rCell))
                            {
                                newStart = j; break;
                            }
                        }
                        List<GridPos> rClaimed = _claimBoard.ClaimPathToGoalOrPrefix(r.Id, newPath, newStart);
                        r.AutoNavigator.SetClaimedPathPrefix(rClaimed);
                    }
                    else
                    {
                        // 重建路径失败：至少保住当前格的 claim，防止机器人"无锁漂移"
                        ReclaimCurrentCell(r);
                    }
                }
                else
                {
                    // ── 被堵方优先级更低 → 被堵方自己让步 ──
                    HandleYieldOrRetreat(r, blocker, gridCount);
                }
            }
        }

        /// <summary>
        /// 判断堵路方是否正在通过被堵格（即将离开），不需要触发让步。
        /// 
        /// 判定条件（全部满足才算"通过"）：
        ///   1) 堵路方没有处于让步冷却中（冷却中无法扩张 claim，无法实际移动，不算"通过"）；
        ///   2) 堵路方有路径且当前位置不在路径末尾（未到终点）；
        ///   3) 堵路方路径的下一步不是朝向被堵方当前格（非迎面冲突）；
        ///   4) 堵路方路径的下一格没有被其他机器人占用（前方畅通，可以实际走出去）。
        /// 
        /// 使用场景：在堵塞分析中，如果堵路方正在通过，则被堵方只需等待即可，
        /// 避免触发不必要的让步/后退操作。
        /// </summary>
        /// <param name="blocker">堵路方机器人实例。</param>
        /// <param name="blockedCurrentCell">被堵方当前所在格子。</param>
        /// <param name="gridCount">网格边长（正方形网格）。</param>
        /// <param name="occupiedByKey">当前帧的格子占用映射（cellKey -> robotId）。</param>
        /// <returns>true 表示堵路方正在通过、即将离开，被堵方应等待。</returns>
        private bool IsBlockerPassingThrough(
            RobotInstance blocker,
            GridPos blockedCurrentCell,
            int gridCount,
            Dictionary<int, int> occupiedByKey)
        {
            // 条件 1：堵路方在冷却中（不能扩张 claim），无法实际移动 → 不算"通过"
            if (_yieldCooldownTicks.TryGetValue(blocker.Id, out int cd) && cd > 0)
            {
                return false;
            }

            // 获取堵路方的完整路径快照
            List<GridPos> blockerPath = blocker.AutoNavigator.GetPathGridSnapshot();
            if (blockerPath == null || blockerPath.Count == 0)
            {
                // 无路径的机器人不可能"正在通过"
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
                // 堵路方当前位置不在其路径上（异常/路径已过期）
                return false;
            }

            // 条件 2：已在路径末尾 → 不会再走 → 不算"通过"
            if (blockerIdx + 1 >= blockerPath.Count)
            {
                return false;
            }

            GridPos blockerNextCell = blockerPath[blockerIdx + 1];

            // 条件 3：堵路方下一步 = 被堵方当前格 → 迎面冲突（A→B 且 B→A），不是"通过"
            if (blockerNextCell.Equals(blockedCurrentCell))
            {
                return false;
            }

            // 条件 4：检查堵路方的下一格是否也被其他机器人占用
            int blockerNextKey = blockerNextCell.Y * gridCount + blockerNextCell.X;
            if (occupiedByKey.TryGetValue(blockerNextKey, out int occupantId) && occupantId != blocker.Id)
            {
                // 堵路方前方也被堵 → 它自己也走不了 → 不算"通过"
                return false;
            }

            // 所有条件均满足：堵路方路径朝其他方向走且前方畅通 → 正在通过，即将离开
            return true;
        }

        /// <summary>
        /// 检查是否形成互堵（对向死锁）：堵路方的路径下一步指向被堵方当前格。
        /// 即 A→B 且 B→A，双方互相挡住对方的下一步，形成经典的"狭路相逢"死锁。
        /// 
        /// 注意：本方法仅判断堵路方→被堵方方向，调用方需自行确保被堵方→堵路方的方向已知。
        /// </summary>
        /// <param name="blocker">堵路方机器人实例。</param>
        /// <param name="blockedCurrentCell">被堵方当前所在格子。</param>
        /// <returns>true 表示堵路方的下一步指向被堵方当前格（形成互堵）。</returns>
        private bool IsMutuallyBlocked(RobotInstance blocker, GridPos blockedCurrentCell)
        {
            List<GridPos> blockerPath = blocker.AutoNavigator.GetPathGridSnapshot();
            if (blockerPath == null || blockerPath.Count == 0)
            {
                return false;
            }

            // 在堵路方路径中定位其当前格
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
                // 找不到当前格或已在路径末尾 → 不构成互堵
                return false;
            }

            // 堵路方的下一步是被堵方的当前格 → 互堵
            GridPos blockerNextCell = blockerPath[blockerIdx + 1];
            return blockerNextCell.Equals(blockedCurrentCell);
        }

        // ══════════════════════════════════════════════════════════
        //  让步 / 后退让路
        // ══════════════════════════════════════════════════════════

        /// <summary>
        /// 让步决策入口：先尝试绕路，绕不了则触发后退让路。
        /// 
        /// 策略：
        /// 1. 以 keeper 当前格作为额外障碍（不可通行），调用 A* 寻找绕行路径；
        /// 2. 如果找到替代路径 → 覆写 yielder 的路径 + 设置让步冷却 + 仅 claim 当前格；
        /// 3. 如果找不到替代路径 → 触发后退让路（<see cref="TriggerRetreat"/>）。
        /// </summary>
        /// <param name="yielder">需要让步的机器人（释放锁并绕路/后退）。</param>
        /// <param name="keeper">被保护的高优先级机器人（yielder 需为其让路）。</param>
        /// <param name="gridCount">网格边长。</param>
        private void HandleYieldOrRetreat(RobotInstance yielder, RobotInstance keeper, int gridCount)
        {
            // 先释放让步方的所有格子锁，为重新规划做准备
            _claimBoard.ReleaseAllByRobot(yielder.Id);

            // 尝试绕路：以 keeper 当前格为"虚拟障碍"重新 A*
            GridPos keeperCell = keeper.GetGridPos_NoLock();
            List<GridPos> altPath = FindAlternatePath(yielder, keeperCell, gridCount);

            if (altPath != null && altPath.Count > 0)
            {
                // 找到绕行路径：覆写路径 + 设置冷却（避免反复抖动）+ 保留当前格锁
                yielder.AutoNavigator.OverridePath(altPath);
                _yieldCooldownTicks[yielder.Id] = YieldCooldownFrames;
                ReclaimCurrentCell(yielder);
            }
            else
            {
                // 绕不过去（例如死胡同/单通道场景）→ 触发后退让路
                TriggerRetreat(yielder, keeper, gridCount);
            }
        }

        /// <summary>
        /// 以堵路方当前格为额外障碍，为让步方寻找替代路径。
        /// 使用 A* 算法，在原有障碍地图基础上将 blockerCell 标记为不可通行。
        /// </summary>
        /// <param name="yielder">让步方机器人实例。</param>
        /// <param name="blockerCell">堵路方当前所在格子（视为临时障碍）。</param>
        /// <param name="gridCount">网格边长。</param>
        /// <returns>替代路径（含起点和终点）；找不到则返回 null。</returns>
        private List<GridPos> FindAlternatePath(RobotInstance yielder, GridPos blockerCell, int gridCount)
        {
            GridPos start = yielder.GetGridPos_NoLock();
            GridPos? goal = yielder.AutoNavigator.GetGoalGridSnapshot();
            if (!goal.HasValue)
            {
                return null;
            }

            var obstacleMap = _world.ObstacleMap;
            // 在原有可通行性判断基础上，额外将 blockerCell 标记为不可通行
            List<GridPos> path = GridPathfinder.FindPath(
                width: gridCount,
                height: gridCount,
                start: start,
                goal: goal.Value,
                isWalkable: p => !obstacleMap.IsObstacle(p) && !p.Equals(blockerCell),
                algorithm: EnumPathfindingAlgorithm.AStar);

            return (path != null && path.Count > 0) ? path : null;
        }

        /// <summary>
        /// 触发后退让路：当绕路失败时，让 yielder 沿远离 keeper 的方向后退。
        /// 
        /// 后退方向计算：
        /// - 取 yielder 到 keeper 的位移向量，取主轴（X 或 Y 中绝对值更大的方向）；
        /// - 后退方向 = 该主轴上远离 keeper 的方向（即位移向量的正方向）。
        /// 
        /// 后退路径构建（<see cref="BuildRetreatPath"/>）：
        /// - 从当前格沿后退方向逐格延伸，直到遇到边界/障碍物；
        /// - 每一步检查是否有侧向出口（<see cref="FindSideExit"/>），有则拐入侧向格结束后退。
        /// 
        /// 如果后退路径只有1格（无法后退），则退化为冷却等待。
        /// </summary>
        /// <param name="yielder">需要后退的机器人。</param>
        /// <param name="keeper">被让路的高优先级机器人。</param>
        /// <param name="gridCount">网格边长。</param>
        private void TriggerRetreat(RobotInstance yielder, RobotInstance keeper, int gridCount)
        {
            GridPos yPos = yielder.GetGridPos_NoLock();
            GridPos kPos = keeper.GetGridPos_NoLock();

            // 计算后退方向：沿远离 keeper 的主轴方向
            int rdx = yPos.X - kPos.X;
            int rdy = yPos.Y - kPos.Y;
            // 选择绝对值更大的轴作为后退主轴，归一化为 -1/0/1
            if (Math.Abs(rdx) >= Math.Abs(rdy))
            {
                rdx = rdx >= 0 ? 1 : -1; rdy = 0;
            }
            else
            {
                rdx = 0; rdy = rdy >= 0 ? 1 : -1;
            }

            // 构建后退路径：从当前格沿后退方向延伸，尽可能找到侧向出口
            List<GridPos> retreatPath = BuildRetreatPath(yPos, rdx, rdy, gridCount);

            if (retreatPath.Count <= 1)
            {
                // 无法后退（前后左右都是障碍/边界）：退化为冷却等待，只保留当前格
                _yieldCooldownTicks[yielder.Id] = YieldCooldownFrames;
                ReclaimCurrentCell(yielder);
                return;
            }

            // 记录后退状态，后续帧由 HandleRetreatingRobot 持续管理
            _retreatByRobotId[yielder.Id] = new RetreatState
            {
                KeeperId = keeper.Id,
                RetreatDx = rdx,
                RetreatDy = rdy
            };

            // 用后退路径覆写导航器路径，并尝试抢占
            yielder.AutoNavigator.OverridePath(retreatPath);
            List<GridPos> claimed = _claimBoard.ClaimPathToGoalOrPrefix(yielder.Id, retreatPath, 0);
            yielder.AutoNavigator.SetClaimedPathPrefix(claimed);
        }

        /// <summary>
        /// 构建后退路径：从 origin 出发，沿 (retreatDx, retreatDy) 方向逐格延伸。
        /// 路径包含 origin 本身（起点），然后逐步向后退方向添加格子。
        /// 
        /// 终止条件（任意一个满足即停止）：
        /// - 下一格越界（超出网格边界）；
        /// - 下一格是障碍物；
        /// - 找到侧向出口（<see cref="FindSideExit"/>）：拐入侧向格后结束，
        ///   这样机器人后退后可以"闪到一边"，让 keeper 通过。
        /// </summary>
        /// <param name="origin">后退起点（当前格）。</param>
        /// <param name="retreatDx">后退方向 X 分量（-1/0/1）。</param>
        /// <param name="retreatDy">后退方向 Y 分量（-1/0/1）。</param>
        /// <param name="gridCount">网格边长。</param>
        /// <returns>后退路径（至少包含 origin，长度为1时表示无法后退）。</returns>
        private List<GridPos> BuildRetreatPath(GridPos origin, int retreatDx, int retreatDy, int gridCount)
        {
            var obstacleMap = _world.ObstacleMap;
            var path = new List<GridPos> { origin };

            for (int step = 1; step <= gridCount; step++)
            {
                GridPos next = new GridPos(origin.X + retreatDx * step, origin.Y + retreatDy * step);

                // 越界检查
                if (next.X < 0 || next.Y < 0 || next.X >= gridCount || next.Y >= gridCount)
                {
                    break;
                }

                // 障碍物检查
                if (obstacleMap.IsObstacle(next))
                {
                    break;
                }

                path.Add(next);

                // 检查当前格是否有侧向出口：
                // 侧向出口 = 与后退方向垂直的相邻可通行格子。
                // 如果有侧向出口，机器人可以"闪到一边"让 keeper 通过，
                // 而不必一直后退到底。
                GridPos? side = FindSideExit(next, retreatDx, retreatDy, gridCount, obstacleMap);
                if (side.HasValue)
                {
                    path.Add(side.Value);
                    break;
                }
            }
            return path;
        }

        /// <summary>
        /// 查找侧向出口：在后退方向的垂直方向上寻找可通行的相邻格子。
        /// 
        /// 例如：后退方向为水平（retreatDx != 0）时，检查上下两个相邻格子；
        ///       后退方向为垂直（retreatDy != 0）时，检查左右两个相邻格子。
        /// 
        /// 优先返回第一个找到的可通行侧向格子。
        /// </summary>
        /// <param name="cell">当前后退到达的格子。</param>
        /// <param name="retreatDx">后退方向 X 分量。</param>
        /// <param name="retreatDy">后退方向 Y 分量。</param>
        /// <param name="gridCount">网格边长。</param>
        /// <param name="obstacleMap">障碍物地图。</param>
        /// <returns>可通行的侧向格子；若两侧都不可通行则返回 null。</returns>
        private static GridPos? FindSideExit(GridPos cell, int retreatDx, int retreatDy,
            int gridCount, ObstacleMap obstacleMap)
        {
            // 根据后退方向确定两个垂直方向的相邻格子
            int s1x, s1y, s2x, s2y;
            if (retreatDx != 0)
            {
                // 水平后退 → 检查上(Y-1)和下(Y+1)
                s1x = cell.X; s1y = cell.Y - 1; s2x = cell.X; s2y = cell.Y + 1;
            }
            else
            {
                // 垂直后退 → 检查左(X-1)和右(X+1)
                s1x = cell.X - 1; s1y = cell.Y; s2x = cell.X + 1; s2y = cell.Y;
            }

            // 检查侧向格子1（上/左）
            if (s1x >= 0 && s1y >= 0 && s1x < gridCount && s1y < gridCount
                && !obstacleMap.IsObstacle(new GridPos(s1x, s1y)))
            {
                return new GridPos(s1x, s1y);
            }

            // 检查侧向格子2（下/右）
            if (s2x >= 0 && s2y >= 0 && s2x < gridCount && s2y < gridCount
                && !obstacleMap.IsObstacle(new GridPos(s2x, s2y)))
            {
                return new GridPos(s2x, s2y);
            }

            return null;
        }

        /// <summary>
        /// 处理正在后退让路中的机器人：每帧检查后退是否完成，并持续维护其 claim。
        /// 
        /// 后退结束条件：
        /// - keeper 已被删除（目标消失） → 立即结束后退；
        /// - keeper 已经通过（不再在 yielder 前方）且 yielder 已到达格心 → 结束后退，恢复正常寻路。
        /// 
        /// "keeper 已通过"的判定：
        /// 计算 keeper 位置相对于 yielder 在"前进方向"（后退方向的反方向）上的投影 dot：
        /// - dot <= 0 表示 keeper 已经在 yielder 的后方或同一位置 → keeper 已通过。
        /// </summary>
        /// <param name="r">正在后退的机器人实例。</param>
        /// <param name="state">该机器人的后退状态。</param>
        /// <param name="gridCount">网格边长。</param>
        /// <param name="occupiedByKey">当前帧的格子占用映射。</param>
        private void HandleRetreatingRobot(RobotInstance r, RetreatState state,
            int gridCount, Dictionary<int, int> occupiedByKey)
        {
            // 查找 keeper 是否还存在
            RobotInstance keeper = TryGetRobotById_NoLock(state.KeeperId);
            if (keeper == null)
            {
                // keeper 已被删除 → 后退不再有意义，结束后退并恢复正常寻路
                EndRetreat(r); return;
            }

            GridPos rPos = r.GetGridPos_NoLock();
            GridPos kPos = keeper.GetGridPos_NoLock();

            // 前进方向 = 后退方向的反向（即原来面对 keeper 的方向）
            int forwardDx = -state.RetreatDx;
            int forwardDy = -state.RetreatDy;

            // dot = keeper 位置在"前进方向"上的投影
            // dot > 0：keeper 在 yielder 前方（还需要继续后退让路）
            // dot <= 0：keeper 在 yielder 后方或同一位置（keeper 已通过，可以结束后退）
            int dot = (kPos.X - rPos.X) * forwardDx + (kPos.Y - rPos.Y) * forwardDy;

            if (dot <= 0 && IsAtCellCenter(r))
            {
                // keeper 已通过，且 yielder 已稳定在格心 → 结束后退，重新寻路
                EndRetreat(r);
                return;
            }

            // 后退仍在进行中：释放旧 claim，重新以当前位置为起点 claim 后退路径
            _claimBoard.ReleaseAllByRobot(r.Id);
            List<GridPos> retreatPath = r.AutoNavigator.GetPathGridSnapshot();
            if (retreatPath == null || retreatPath.Count == 0)
            {
                // 后退路径丢失：保留当前格锁
                ReclaimCurrentCell(r);
                return;
            }

            // 在后退路径中定位当前格
            int startIdx = 0;
            for (int j = 0; j < retreatPath.Count; j++)
            {
                if (retreatPath[j].Equals(rPos))
                {
                    startIdx = j;
                    break;
                }
            }

            // 从当前位置开始重新 claim 后退路径
            List<GridPos> claimed = _claimBoard.ClaimPathToGoalOrPrefix(r.Id, retreatPath, startIdx);
            r.AutoNavigator.SetClaimedPathPrefix(claimed);
        }

        /// <summary>
        /// 结束后退让路状态：
        /// 1. 移除后退状态记录；
        /// 2. 释放所有格子锁；
        /// 3. 触发重新寻路（恢复正常目标导航）；
        /// 4. 重新 claim 当前格（确保机器人不会"无锁漂移"）。
        /// </summary>
        /// <param name="r">结束后退的机器人实例。</param>
        private void EndRetreat(RobotInstance r)
        {
            _retreatByRobotId.Remove(r.Id);
            _claimBoard.ReleaseAllByRobot(r.Id);
            r.AutoNavigator.RebuildPath();
            ReclaimCurrentCell(r);
        }

        /// <summary>
        /// 边走边释放：当机器人检测到"网格位置发生变化"（从一格移到另一格），释放旧格子的锁。
        /// 
        /// 原理：
        /// - 记录每个机器人上一帧所在的网格坐标；
        /// - 每帧比较当前格与上一帧格，若不同则说明机器人已离开旧格子；
        /// - 释放旧格子的锁，使后方机器人能更早地抢占到这些格子。
        /// 
        /// 注意：首次调用（字典中无记录）时仅初始化记录，不执行释放。
        /// 
        /// 要求：调用方已持有 RobotLock，且在 Move.Update() 之后调用。
        /// </summary>
        /// <param name="r">待处理的机器人实例。</param>
        public void ReleaseClaimByMovement_NoLock(RobotInstance r)
        {
            GridPos current = r.GetGridPos_NoLock();

            if (!_lastGridCellByRobotId.TryGetValue(r.Id, out GridPos last))
            {
                // 首次记录：仅初始化，不释放
                _lastGridCellByRobotId[r.Id] = current;
                return;
            }

            if (!current.Equals(last))
            {
                // 格子发生变化：释放旧格子的锁
                _claimBoard.ReleaseCell(r.Id, last);
                // 更新记录为当前格
                _lastGridCellByRobotId[r.Id] = current;
            }
        }

        /// <summary>
        /// 判断机器人是否处于其所在格子的中心位置（在容差范围内）。
        /// 
        /// 计算方式：
        /// - 根据机器人所在的网格坐标(gx, gy)计算该格子中心的世界坐标：
        ///   cx = gx * cellSize + cellSize/2，cy = gy * cellSize + cellSize/2；
        /// - 若机器人世界坐标(X,Y)与(cx,cy)的偏差在 <see cref="CellCenterEpsilonM"/> 以内，则视为"在格心"。
        /// 
        /// 用途：堵塞分析时，只有机器人已稳定在格心才触发让步决策，
        /// 避免机器人还在移动过程中就做出不准确的判断。
        /// </summary>
        /// <param name="r">待判断的机器人实例。</param>
        /// <returns>true 表示机器人处于格心（容差内）。</returns>
        private bool IsAtCellCenter(RobotInstance r)
        {
            double cellSize = _world.CellSizeM;
            GridPos g = r.GetGridPos_NoLock();
            // 计算所在格子中心的世界坐标
            double cx = g.X * cellSize + cellSize / 2.0;
            double cy = g.Y * cellSize + cellSize / 2.0;
            // 双轴偏差均在容差内 → 视为在格心
            return Math.Abs(r.X - cx) <= CellCenterEpsilonM
                && Math.Abs(r.Y - cy) <= CellCenterEpsilonM;
        }

        /// <summary>
        /// 为指定机器人重新抢占当前格，确保其能移向格心后停稳。
        /// 
        /// 使用场景：
        /// - 让步/后退后需要保证至少持有当前格的锁；
        /// - 路径重建失败时的兜底操作，防止机器人处于"无锁"状态。
        /// </summary>
        /// <param name="r">待 claim 当前格的机器人实例。</param>
        private void ReclaimCurrentCell(RobotInstance r)
        {
            GridPos cell = r.GetGridPos_NoLock();
            // 构建只含当前格的"路径"，交由 ClaimBoard 抢占
            var keep = new List<GridPos>(1) { cell };
            List<GridPos> claimed = _claimBoard.ClaimPathToGoalOrPrefix(r.Id, keep, 0);
            r.AutoNavigator.SetClaimedPathPrefix(claimed);
        }

        /// <summary>
        /// 清理某个机器人相关的所有抢占状态。
        /// 
        /// 清理内容：
        /// - 释放其在 ClaimBoard 中占用的所有格子锁；
        /// - 删除"边走边释放"中记录的上一帧所在格；
        /// - 删除其让步冷却计数（若有）；
        /// - 删除其后退让路状态（若有）。
        /// 
        /// 用途：删除机器人、重置单机状态时的清理工作，确保不残留无效数据。
        /// </summary>
        /// <param name="robotId">待清理的机器人 ID。</param>
        public void ClearRobotState(int robotId)
        {
            _claimBoard.ReleaseAllByRobot(robotId);
            _lastGridCellByRobotId.Remove(robotId);
            _yieldCooldownTicks.Remove(robotId);
            _retreatByRobotId.Remove(robotId);
        }

        /// <summary>
        /// 内部工具：按 Id 查找机器人实例（调用方已持有锁）。
        /// 
        /// 查找策略（兼顾性能与正确性）：
        /// 1. 先尝试"快速路径"：如果 robotId 小于 Robots.Count 且 Robots[robotId].Id == robotId，
        ///    说明 Id 与列表索引一致（常见情况），直接返回，O(1)；
        /// 2. 快速路径不命中则线性查找，O(n)。
        /// </summary>
        /// <param name="robotId">待查找的机器人 ID（负值直接返回 null）。</param>
        /// <returns>对应的 RobotInstance；找不到或 ID 非法则返回 null。</returns>
        private RobotInstance TryGetRobotById_NoLock(int robotId)
        {
            if (robotId < 0)
            {
                return null;
            }

            var robots = _world.Robots;

            // 快速路径：Id 与索引一致时直接返回
            if (robotId < robots.Count && robots[robotId].Id == robotId)
            {
                return robots[robotId];
            }

            // 兜底：线性遍历查找
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