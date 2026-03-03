<<<<<<< Updated upstream
﻿using GridDemo.Models.Pathfinding;
=======
﻿using GridDemo.Managers;
using GridDemo.Maps;
using GridDemo.Models.Pathfinding;
>>>>>>> Stashed changes
using System;
using System.Collections.Generic;

namespace GridDemo.Models
{
    /// <summary>
    /// 死锁类型枚举：描述两台（或多台）机器人原地不动的根本原因。
    /// </summary>
    internal enum DeadlockType
    {
        /// <summary> 无死锁。 </summary>
        None,

        /// <summary>
        /// 位置互换死锁：A 想去 B 所在格，B 想去 A 所在格（或路径下一步互指对方当前格）。
        /// </summary>
        PositionSwap,

        /// <summary>
        /// 对向阻塞：两台机器人面对面行进，各自的下一步被对方占用，但并非严格的位置互换。
        /// 例如 A→右 被 B 挡住，B→左 被 A 挡住，但目标格不完全是对方当前格。
        /// </summary>
        HeadOn,

        /// <summary>
        /// 链式/环形死锁：A 被 B 挡，B 被 C 挡，C 被 A 挡（或更长环路）。
        /// </summary>
        CyclicChain,

        /// <summary>
        /// 单向阻塞：A 被 B 挡住，但 B 并未被 A 挡住（B 可能被其它机器人或障碍挡住）。
        /// </summary>
        UnidirectionalBlock
    }

    /// <summary>
    /// 死锁分析结果：描述一组互相阻塞的机器人及其死锁类型。
    /// </summary>
    internal struct DeadlockAnalysis
    {
        /// <summary> 死锁类型。 </summary>
        public DeadlockType Type;

        /// <summary> 参与死锁的机器人 Id 列表（按环形顺序或阻塞链顺序排列）。 </summary>
        public List<int> InvolvedRobotIds;

        /// <summary>
        /// 应该让步（绕路）的机器人 Id。
        /// 规则：优先级低的机器人绕路，优先级高的原地等待。
        /// 优先级定义：到目标的曼哈顿距离越近，优先级越高（Id 越小作为次要排序）。
        /// </summary>
        public int YieldRobotId;

        /// <summary>
        /// 应该原地等待的机器人 Id（优先级高的一方）。
        /// </summary>
        public int WaitRobotId;
    }

    /// <summary>
    /// PathClaimManager：
    /// 职责：
    /// - 管理格子锁抢占板 GridCellClaimBoard；
    /// - 负责路径前缀抢占（ClaimPathToGoalOrPrefix）并回灌给 AutoNavigator；
<<<<<<< Updated upstream
    /// - 做即时死锁类型分析（位置互换 / 对向阻塞 / 环形链 / 单向阻塞）；
    /// - 根据优先级执行"低优先级绕路 + 高优先级原地等待"策略；
    /// - 支持"边走边释放"策略：机器人移动出一个格子后释放对应锁。
    ///
    /// 死锁检测策略（重构版）：
    /// - 不再等待 150 帧确认死锁；
    /// - 每帧构建"被阻塞图"（blocked -> blocker），通过算法分析阻塞原因；
    /// - 检测到互卡（互换 / 对向 / 环形）后立即按优先级决定谁绕路、谁等待；
    /// - 绕路方：进入让步冷却 + 释放锁 + 以对方占用格为临时障碍重建路径；
    /// - 等待方：保持当前位置不动，不释放锁，等绕路方走开后自然恢复。
=======
    /// - 做实时"互卡/死锁"诊断：立即分析前方堵路机器人停止原因，
    ///   根据死锁类型（交换位置/十字路口/环形等待）快速决策绕路或等待；
    /// - 支持"边走边释放"策略：机器人移动出一个格子后释放对应锁。
>>>>>>> Stashed changes
    ///
    /// 线程模型：
    /// - 仅在持有 RobotWorld.RobotLock 时调用本类的公开方法。
    /// </summary>
    internal sealed class PathClaimManager
    {
        private readonly RobotWorld _world;
        private readonly GridCellClaimBoard _claimBoard;

        // 让步冷却：强制某机器人在一段时间内不再扩张 claim，避免抖动
        private const int YieldCooldownFrames = 20;

<<<<<<< Updated upstream
        // 阻塞确认帧数：连续被同一机器人阻塞达到此帧数才触发死锁分析，
        // 避免短暂交错误判（值较小，响应快）
        private const int BlockConfirmFrames = 6;
=======
        // 非死锁场景下的最大等待帧数：超过后执行重规划
        private const int NonDeadlockMaxWaitFrames = 80;

        // 高优先级原地等待后的重规划阈值（长时间不动则强制重规划）
        private const int HighPriorityReplanFrames = 200;
>>>>>>> Stashed changes

        // robotId -> 冷却剩余帧数
        private readonly Dictionary<int, int> _yieldCooldownTicks = new Dictionary<int, int>();

<<<<<<< Updated upstream
        // 阻塞持续帧计数：blockedId -> (blockerId, 持续帧数)
        private readonly Dictionary<int, BlockTrack> _blockTracks = new Dictionary<int, BlockTrack>();
=======
        // blockedId -> 阻塞诊断跟踪信息
        private readonly Dictionary<int, BlockTrackInfo> _blockTracking = new Dictionary<int, BlockTrackInfo>();

        // wait-for 映射：blockedId -> blockerId（每帧重建）
        private readonly Dictionary<int, int> _waitForMap = new Dictionary<int, int>();
>>>>>>> Stashed changes

        // 边走边释放：记录上一帧所在格
        private readonly Dictionary<int, GridPos> _lastGridCellByRobotId = new Dictionary<int, GridPos>();

<<<<<<< Updated upstream
        // 已处理的死锁对记录，避免同一对在冷却期内反复触发
        // key = min(idA,idB)*10000 + max(idA,idB)，value = 剩余冷却帧
        private readonly Dictionary<long, int> _resolvedPairCooldown = new Dictionary<long, int>();

        private struct BlockTrack
        {
            public int BlockerId;
            public int Frames;
=======
        private struct BlockTrackInfo
        {
            public int BlockerId;
            public int WaitedFrames;       // 已等待帧数
            public EnumDeadlockType LastDeadlockType;
            public bool RerouteTriggered;  // 是否已触发绕路
>>>>>>> Stashed changes
        }

        public PathClaimManager(RobotWorld world)
        {
            _world = world;
            _claimBoard = new GridCellClaimBoard(world.GridCount, world.ObstacleMap);
        }

        /// <summary> 对外暴露格子锁抢占板（供 UI 显示快照等）。 </summary>
        public GridCellClaimBoard ClaimBoard => _claimBoard;

        /// <summary>
<<<<<<< Updated upstream
        /// 每帧调用：推进让步冷却与已处理死锁对冷却的衰减。
=======
        /// 每帧调用：推进让步冷却。
>>>>>>> Stashed changes
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

<<<<<<< Updated upstream
            // 已处理死锁对冷却衰减
            if (_resolvedPairCooldown.Count > 0)
            {
                var pairKeys = new List<long>(_resolvedPairCooldown.Keys);
                for (int i = 0; i < pairKeys.Count; i++)
                {
                    long pk = pairKeys[i];
                    int remain = _resolvedPairCooldown[pk] - 1;
                    if (remain <= 0)
                    {
                        _resolvedPairCooldown.Remove(pk);
                    }
                    else
                    {
                        _resolvedPairCooldown[pk] = remain;
                    }
=======
            // 阻塞跟踪：自增等待帧数，清理不再阻塞的记录
            if (_blockTracking.Count > 0)
            {
                var trackKeys = new List<int>(_blockTracking.Keys);
                for (int i = 0; i < trackKeys.Count; i++)
                {
                    int id = trackKeys[i];
                    var info = _blockTracking[id];
                    info.WaitedFrames++;
                    _blockTracking[id] = info;
>>>>>>> Stashed changes
                }
            }
        }

        /// <summary>
<<<<<<< Updated upstream
        /// 为所有机器人执行"路径格子锁抢占"，并在抢占过程中进行即时死锁分析与处理。
        ///
        /// 抢占优先级：按照"当前格到终点的曼哈顿距离"从近到远，近者优先。
        ///
        /// 死锁检测与处理流程：
        /// 1) 构建阻塞关系图：blockedId -> blockerId（路径下一步被谁的当前格/锁占用）；
        /// 2) 累计阻塞帧数，达到 BlockConfirmFrames 后进入死锁分析；
        /// 3) 分析死锁类型：
        ///    - 位置互换（A 下一步 = B 当前格，B 下一步 = A 当前格）
        ///    - 对向阻塞（互相阻塞但目标格不完全互换）
        ///    - 环形链（A→B→C→A 或更长环路）
        ///    - 单向阻塞（A 被 B 挡，但 B 没被 A 挡）
        /// 4) 按优先级处理：
        ///    - 优先级 = (曼哈顿距离越近越高, Id 越小越高)
        ///    - 低优先级方：进入让步冷却 + 释放锁 + 重建路径（绕路）
        ///    - 高优先级方：保持不动（原地等待），不释放锁
=======
        /// 为所有机器人执行"路径格子锁抢占"，并把抢占到的路径前缀写回对应 AutoNavigator。
        /// 
        /// 改进逻辑：
        /// 当机器人被堵时，**立即**分析前方堵路机器人的停止原因：
        ///   - GoalPauseWait（终点停顿）/ InPlaceTurn（原地转向）/ QueueWait（排队等待）
        /// 然后判断是否构成死锁（交换位置/十字路口/wait-for环）：
        ///   - 死锁：低优先级方立即绕路，高优先级方原地等待（超时则重规划）
        ///   - 非死锁：根据停止原因给出短等待窗口，超时后重规划
>>>>>>> Stashed changes
        ///
        /// 要求：调用方已持有 RobotLock。
        /// </summary>
        public void ApplyPathClaiming_NoLock()
        {
            var robots = _world.Robots;
            int gridCount = _world.GridCount;

            // ---- 第一步：收集并排序 ----
            var items = new List<(RobotInstance R, int Dist)>(robots.Count);

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

<<<<<<< Updated upstream
            // ---- 第二步：构建占用格映射与保留格 ----
=======
            // 2. 预生成"当前占用格 -> robotId"映射
>>>>>>> Stashed changes
            var occupiedByKey = new Dictionary<int, int>(robots.Count);
            // robotId -> 当前格
            var robotCurrentCell = new Dictionary<int, GridPos>(robots.Count);

            for (int i = 0; i < robots.Count; i++)
            {
                GridPos c = robots[i].GetGridPos_NoLock();
                int key = c.Y * gridCount + c.X;
                if (!occupiedByKey.ContainsKey(key))
                {
                    occupiedByKey.Add(key, robots[i].Id);
                }
                robotCurrentCell[robots[i].Id] = c;
            }

<<<<<<< Updated upstream
            _claimBoard.SetReservedCells(occupiedByKey);

            // ---- 第三步：逐机器人执行 claim，并记录阻塞关系 ----
            // blockedId -> (blockerId, blockedNextCell)：本帧的阻塞关系
            var frameBlocked = new Dictionary<int, (int BlockerId, GridPos NextCell)>();
            // robotId -> 路径下一步目标格
            var robotNextCell = new Dictionary<int, GridPos>();

=======
            // 把所有机器人"当前占用格"设置为保留格
            _claimBoard.SetReservedCells(occupiedByKey);

            // 每帧重建 waitFor 映射
            _waitForMap.Clear();

            // 记录本帧哪些机器人仍在被堵（用于清理不再阻塞的跟踪记录）
            var stillBlockedIds = new HashSet<int>();

            // 3. 逐机器人执行 claim
>>>>>>> Stashed changes
            for (int i = 0; i < items.Count; i++)
            {
                RobotInstance r = items[i].R;

                // 让步冷却期间：该机器人不再扩张 claim（只保留当前位置）
                if (_yieldCooldownTicks.TryGetValue(r.Id, out int cooldown) && cooldown > 0)
                {
                    _claimBoard.ReleaseAllByRobot(r.Id);

                    GridPos curCell = r.GetGridPos_NoLock();
                    var keep = new List<GridPos>(1) { curCell };
                    List<GridPos> claimedKeep = _claimBoard.ClaimPathToGoalOrPrefix(r.Id, keep, 0);
                    r.AutoNavigator.SetClaimedPathPrefix(claimedKeep);
                    continue;
                }

<<<<<<< Updated upstream
                // 从 AutoNavigator 拿整条路径
=======
                // 3.2 从 AutoNavigator 拿整条路径
>>>>>>> Stashed changes
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

<<<<<<< Updated upstream
                // 找到路径中与"当前所在格"对应的索引（作为抢占起点）
=======
                // 找到路径中与"当前所在格"对应的索引
>>>>>>> Stashed changes
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

                // 记录路径下一步目标格，用于死锁分析
                int nextIdx = startIndex + 1;
                if (nextIdx < path.Count)
                {
                    robotNextCell[r.Id] = path[nextIdx];
                }

                List<GridPos> claimedPrefix = _claimBoard.ClaimPathToGoalOrPrefix(r.Id, path, startIndex);
                r.AutoNavigator.SetClaimedPathPrefix(claimedPrefix);

<<<<<<< Updated upstream
                // ---- 检测阻塞关系 ----
=======
                // 4. 实时阻塞诊断（替代原150帧盲等）
>>>>>>> Stashed changes
                if (!r.AutoNavigator.IsEnabled)
                {
                    continue;
                }

                int claimedCount = claimedPrefix?.Count ?? 0;
                bool stuckAtStart = claimedCount <= 1;

                if (!stuckAtStart)
                {
<<<<<<< Updated upstream
                    // 未被阻塞：清除该机器人的阻塞计数
                    _blockTracks.Remove(r.Id);
                    continue;
                }

                if (nextIdx < 0 || nextIdx >= path.Count)
=======
                    // 不再阻塞，清除跟踪记录
                    _blockTracking.Remove(r.Id);
                    continue;
                }

                int nextIndex = startIndex + 1;
                if (nextIndex < 0 || nextIndex >= path.Count)
>>>>>>> Stashed changes
                {
                    continue;
                }

<<<<<<< Updated upstream
                GridPos nextCell = path[nextIdx];
=======
                GridPos nextCell = path[nextIndex];
>>>>>>> Stashed changes
                int nextKey = nextCell.Y * gridCount + nextCell.X;

                if (!occupiedByKey.TryGetValue(nextKey, out int blockerId) || blockerId == r.Id)
                {
                    continue;
                }

<<<<<<< Updated upstream
                // 记录本帧阻塞关系
                frameBlocked[r.Id] = (blockerId, nextCell);

                // 累计阻塞帧数
                BlockTrack track;
                if (!_blockTracks.TryGetValue(r.Id, out track) || track.BlockerId != blockerId)
                {
                    track = new BlockTrack { BlockerId = blockerId, Frames = 1 };
                }
                else
                {
                    track.Frames++;
                }
                _blockTracks[r.Id] = track;
            }

            // ---- 第四步：死锁分析与处理 ----
            // 收集达到确认阈值的阻塞关系
            var confirmedBlocked = new Dictionary<int, int>(); // blockedId -> blockerId
            foreach (var kv in _blockTracks)
            {
                if (kv.Value.Frames >= BlockConfirmFrames)
                {
                    confirmedBlocked[kv.Key] = kv.Value.BlockerId;
                }
            }

            if (confirmedBlocked.Count == 0)
            {
                return;
            }

            // 分析死锁并处理
            var processed = new HashSet<int>(); // 已处理的机器人 Id，避免重复处理

            foreach (var kv in confirmedBlocked)
            {
                int blockedId = kv.Key;
                int blockerId = kv.Value;

                if (processed.Contains(blockedId) || processed.Contains(blockerId))
=======
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
                    // 新的阻塞关系，重置计数
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
>>>>>>> Stashed changes
                {
                    HandleDeadlock(r, blocker, diagnosis, track);
                }
<<<<<<< Updated upstream

                // 检查该对是否在冷却期内（已处理过）
                long pairKey = MakePairKey(blockedId, blockerId);
                if (_resolvedPairCooldown.ContainsKey(pairKey))
                {
                    continue;
=======
                else
                {
                    HandleNonDeadlockBlock(r, blocker, diagnosis, track);
>>>>>>> Stashed changes
                }
            }

<<<<<<< Updated upstream
                // 分析死锁类型
                DeadlockAnalysis analysis = AnalyzeDeadlock(
                    blockedId, blockerId, confirmedBlocked,
                    robotCurrentCell, robotNextCell, gridCount);

                if (analysis.Type == DeadlockType.None)
                {
                    continue;
                }

                // 执行处理：低优先级绕路，高优先级等待
                ResolveDeadlock(analysis, robotCurrentCell, gridCount);

                // 标记已处理
                if (analysis.InvolvedRobotIds != null)
                {
                    for (int i = 0; i < analysis.InvolvedRobotIds.Count; i++)
                    {
                        processed.Add(analysis.InvolvedRobotIds[i]);
                    }
                }

                // 记录该对的冷却，避免反复触发
                _resolvedPairCooldown[pairKey] = YieldCooldownFrames + 5;
=======
            // 清理不再阻塞的跟踪记录
            var trackKeys = new List<int>(_blockTracking.Keys);
            for (int i = 0; i < trackKeys.Count; i++)
            {
                if (!stillBlockedIds.Contains(trackKeys[i]))
                {
                    _blockTracking.Remove(trackKeys[i]);
                }
>>>>>>> Stashed changes
            }
        }

        /// <summary>
<<<<<<< Updated upstream
        /// 分析死锁类型：判断两台（或多台）机器人为什么原地不动。
        /// </summary>
        /// <param name="blockedId">被阻塞的机器人 Id。</param>
        /// <param name="blockerId">阻塞者的机器人 Id。</param>
        /// <param name="allBlocked">所有达到确认阈值的阻塞关系。</param>
        /// <param name="currentCells">所有机器人当前所在格。</param>
        /// <param name="nextCells">所有机器人路径下一步目标格。</param>
        /// <param name="gridCount">网格尺寸。</param>
        /// <returns>死锁分析结果。</returns>
        private DeadlockAnalysis AnalyzeDeadlock(
            int blockedId, int blockerId,
            Dictionary<int, int> allBlocked,
            Dictionary<int, GridPos> currentCells,
            Dictionary<int, GridPos> nextCells,
            int gridCount)
        {
            var result = new DeadlockAnalysis
            {
                Type = DeadlockType.None,
                InvolvedRobotIds = new List<int>()
            };

            // 获取当前格和目标格
            GridPos blockedCur, blockerCur;
            if (!currentCells.TryGetValue(blockedId, out blockedCur) ||
                !currentCells.TryGetValue(blockerId, out blockerCur))
            {
                return result;
            }

            GridPos blockedNext = default;
            bool hasBlockedNext = nextCells.TryGetValue(blockedId, out blockedNext);

            GridPos blockerNext = default;
            bool hasBlockerNext = nextCells.TryGetValue(blockerId, out blockerNext);

            // ---- 检测1：位置互换死锁 ----
            // A 的下一步 = B 的当前格，且 B 的下一步 = A 的当前格
            if (hasBlockedNext && hasBlockerNext)
            {
                bool aWantsB = blockedNext.Equals(blockerCur);
                bool bWantsA = blockerNext.Equals(blockedCur);

                if (aWantsB && bWantsA)
                {
                    result.Type = DeadlockType.PositionSwap;
                    result.InvolvedRobotIds.Add(blockedId);
                    result.InvolvedRobotIds.Add(blockerId);
                    DetermineYieldByPriority(ref result, currentCells, gridCount);
                    return result;
                }
            }

            // ---- 检测2：环形链死锁 ----
            // 从 blockedId 沿阻塞链追溯，看是否能回到 blockedId
            var chain = new List<int>();
            var visited = new HashSet<int>();
            int current = blockedId;

            while (true)
            {
                if (visited.Contains(current))
                {
                    // 找到环：从 current 开始截取环上的所有节点
                    int cycleStart = chain.IndexOf(current);
                    if (cycleStart >= 0)
                    {
                        var cycle = new List<int>();
                        for (int i = cycleStart; i < chain.Count; i++)
                        {
                            cycle.Add(chain[i]);
                        }

                        if (cycle.Count >= 2)
                        {
                            result.Type = cycle.Count == 2 ? DeadlockType.HeadOn : DeadlockType.CyclicChain;
                            result.InvolvedRobotIds = cycle;
                            DetermineYieldByPriority(ref result, currentCells, gridCount);
                            return result;
                        }
                    }
                    break;
                }

                visited.Add(current);
                chain.Add(current);

                int nextInChain;
                if (!allBlocked.TryGetValue(current, out nextInChain))
                {
                    break;
                }

                current = nextInChain;
            }

            // ---- 检测3：对向阻塞（互相阻塞但不满足严格互换条件） ----
            // B 也被 A 阻塞
            int blockerBlocker;
            if (allBlocked.TryGetValue(blockerId, out blockerBlocker) && blockerBlocker == blockedId)
            {
                result.Type = DeadlockType.HeadOn;
                result.InvolvedRobotIds.Add(blockedId);
                result.InvolvedRobotIds.Add(blockerId);
                DetermineYieldByPriority(ref result, currentCells, gridCount);
                return result;
            }

            // ---- 检测4：单向阻塞 ----
            // A 被 B 挡住，但 B 没有被 A 挡住
            result.Type = DeadlockType.UnidirectionalBlock;
            result.InvolvedRobotIds.Add(blockedId);
            result.InvolvedRobotIds.Add(blockerId);
            // 单向阻塞时：被阻塞方绕路，阻塞方保持行进
            result.YieldRobotId = blockedId;
            result.WaitRobotId = blockerId;
            return result;
        }

        /// <summary>
        /// 根据优先级决定谁绕路、谁等待。
        /// 优先级规则：
        /// - 到目标的曼哈顿距离越近，优先级越高（即更接近终点的机器人优先通行）；
        /// - 距离相同时，Id 越小优先级越高。
        /// 低优先级方绕路，高优先级方原地等待。
        /// </summary>
        private void DetermineYieldByPriority(
            ref DeadlockAnalysis analysis,
            Dictionary<int, GridPos> currentCells,
            int gridCount)
        {
            if (analysis.InvolvedRobotIds == null || analysis.InvolvedRobotIds.Count < 2)
=======
        /// 处理确认为死锁的情况：低优先级方立即绕路，高优先级方等待（超时重规划）
        /// </summary>
        private void HandleDeadlock(RobotInstance self, RobotInstance blocker,
            BlockDiagnosis diagnosis, BlockTrackInfo track)
        {
            if (diagnosis.ShouldReroute && !track.RerouteTriggered)
            {
                // self 是低优先级 → 立即绕路
                // 1) self 进入冷却期并释放 claim
                _yieldCooldownTicks[self.Id] = YieldCooldownFrames;
                _claimBoard.ReleaseAllByRobot(self.Id);
                self.AutoNavigator.SetClaimedPathPrefix(null);
                self.AutoNavigator.RebuildPath();

                // 标记已触发，避免每帧重复
                track.RerouteTriggered = true;
                track.WaitedFrames = 0;
                _blockTracking[self.Id] = track;
            }
            else if (diagnosis.ShouldHoldAndWait)
            {
                // self 是高优先级 → 原地等待
                // 但如果长时间不动，强制重规划
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
                // 否则继续等待，不做额外操作
            }
        }

        /// <summary>
        /// 处理非死锁阻塞：根据前方停止原因短暂等待，超时后重规划
        /// </summary>
        private void HandleNonDeadlockBlock(RobotInstance self, RobotInstance blocker,
            BlockDiagnosis diagnosis, BlockTrackInfo track)
        {
            // 等待帧数未超过建议值 → 继续等待（前方在转向/排队，很快会消除）
            if (track.WaitedFrames <= diagnosis.SuggestedWaitFrames)
>>>>>>> Stashed changes
            {
                return;
            }

<<<<<<< Updated upstream
            // 计算每个参与者的优先级（曼哈顿距离, Id）
            int bestId = -1;
            int bestDist = int.MaxValue;
            int worstId = -1;
            int worstDist = int.MinValue;

            for (int i = 0; i < analysis.InvolvedRobotIds.Count; i++)
            {
                int rid = analysis.InvolvedRobotIds[i];
                RobotInstance robot = TryGetRobotById_NoLock(rid);
                if (robot == null)
                {
                    continue;
                }

                GridPos? goal = robot.AutoNavigator.GetGoalGridSnapshot();
                int dist;
                if (goal.HasValue && currentCells.TryGetValue(rid, out GridPos cur))
                {
                    dist = Math.Abs(cur.X - goal.Value.X) + Math.Abs(cur.Y - goal.Value.Y);
                }
                else
                {
                    dist = int.MaxValue; // 无目标的视为最低优先级
                }

                // 优先级高 = 距离近 + Id小
                bool isBetter = (dist < bestDist) || (dist == bestDist && rid < bestId);
                if (bestId < 0 || isBetter)
                {
                    bestId = rid;
                    bestDist = dist;
                }

                // 优先级低 = 距离远 + Id大
                bool isWorse = (dist > worstDist) || (dist == worstDist && rid > worstId);
                if (worstId < 0 || isWorse)
                {
                    worstId = rid;
                    worstDist = dist;
                }
            }

            // 高优先级等待，低优先级绕路
            analysis.WaitRobotId = bestId;
            analysis.YieldRobotId = worstId;
        }

        /// <summary>
        /// 执行死锁处理：
        /// - 低优先级方（YieldRobotId）：进入让步冷却 + 释放所有锁 + 重建路径（绕路）；
        /// - 高优先级方（WaitRobotId）：保持当前位置和锁，原地等待。
        /// </summary>
        private void ResolveDeadlock(
            DeadlockAnalysis analysis,
            Dictionary<int, GridPos> currentCells,
            int gridCount)
        {
            int yieldId = analysis.YieldRobotId;
            int waitId = analysis.WaitRobotId;

            // ---- 处理让步方（低优先级）：释放锁 + 冷却 + 重建路径 ----
            RobotInstance yielder = TryGetRobotById_NoLock(yieldId);
            if (yielder != null)
            {
                // 进入让步冷却：冷却期间不扩张 claim，给高优先级方让出空间
                _yieldCooldownTicks[yieldId] = YieldCooldownFrames;

                // 释放让步方的所有格子锁
                _claimBoard.ReleaseAllByRobot(yieldId);
                yielder.AutoNavigator.SetClaimedPathPrefix(null);

                // 重建路径：寻路算法会自动绕开动态障碍（高优先级方占用的格子）
                yielder.AutoNavigator.RebuildPath();

                // 清除让步方的阻塞计数
                _blockTracks.Remove(yieldId);
            }

            // ---- 处理等待方（高优先级）：保持不动 ----
            // 高优先级方不需要任何操作，保持当前锁和位置。
            // 引擎 Tick 中因为 claimedPrefix 只有当前格（或更多），
            // 等让步方绕路走开后，下一帧等待方自然能抢到新的前缀继续前进。

            // 清除等待方的阻塞计数（如果有）
            _blockTracks.Remove(waitId);

            // 对于环形链死锁（3+ 机器人），除了最高优先级外全部让步
            if (analysis.Type == DeadlockType.CyclicChain && analysis.InvolvedRobotIds.Count > 2)
            {
                for (int i = 0; i < analysis.InvolvedRobotIds.Count; i++)
                {
                    int rid = analysis.InvolvedRobotIds[i];
                    if (rid == waitId)
                    {
                        continue; // 最高优先级方等待
                    }

                    if (rid == yieldId)
                    {
                        continue; // 已经处理过
                    }

                    RobotInstance otherYielder = TryGetRobotById_NoLock(rid);
                    if (otherYielder != null)
                    {
                        _yieldCooldownTicks[rid] = YieldCooldownFrames;
                        _claimBoard.ReleaseAllByRobot(rid);
                        otherYielder.AutoNavigator.SetClaimedPathPrefix(null);
                        otherYielder.AutoNavigator.RebuildPath();
                        _blockTracks.Remove(rid);
                    }
                }
            }
=======
            // 超过建议等待时间但未超过最大值 → 仍然等待（可能只是慢了点）
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
>>>>>>> Stashed changes
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

                // 机器人移动了说明阻塞解除，清除其阻塞计数
                _blockTracks.Remove(r.Id);
            }
        }

        /// <summary>
<<<<<<< Updated upstream
        /// 清理某个机器人相关的抢占状态：
        /// - 释放其占用的所有格子锁；
        /// - 删除"边走边释放"的历史记录；
        /// - 删除其让步冷却计数与阻塞计数记录。
        /// 用途：删除机器人、重置单机时的清理工作。
=======
        /// 清理某个机器人相关的抢占状态。
>>>>>>> Stashed changes
        /// </summary>
        public void ClearRobotState(int robotId)
        {
            _claimBoard.ReleaseAllByRobot(robotId);
            _lastGridCellByRobotId.Remove(robotId);
            _yieldCooldownTicks.Remove(robotId);
<<<<<<< Updated upstream
            _blockTracks.Remove(robotId);

            // 清理所有与该机器人相关的阻塞记录
            var toRemove = new List<int>();
            foreach (var kv in _blockTracks)
=======
            _blockTracking.Remove(robotId);

            // 清理 waitFor 中相关记录
            var toRemoveWait = new List<int>();
            foreach (var kv in _waitForMap)
>>>>>>> Stashed changes
            {
                if (kv.Key == robotId || kv.Value == robotId)
                    toRemoveWait.Add(kv.Key);
            }
<<<<<<< Updated upstream
            foreach (var k in toRemove)
                _blockTracks.Remove(k);

            // 清理相关的已处理死锁对冷却
            var pairsToRemove = new List<long>();
            foreach (var kv in _resolvedPairCooldown)
            {
                long pk = kv.Key;
                int a = (int)(pk / 10000);
                int b = (int)(pk % 10000);
                if (a == robotId || b == robotId)
                {
                    pairsToRemove.Add(pk);
                }
            }
            foreach (var pk in pairsToRemove)
                _resolvedPairCooldown.Remove(pk);
        }

        /// <summary>
        /// 生成死锁对的唯一 key（与顺序无关）。
        /// </summary>
        private static long MakePairKey(int idA, int idB)
        {
            int min = Math.Min(idA, idB);
            int max = Math.Max(idA, idB);
            return (long)min * 10000L + max;
=======
            foreach (var k in toRemoveWait)
                _waitForMap.Remove(k);

            // 清理其他机器人对该机器人的阻塞跟踪
            var toRemoveTrack = new List<int>();
            foreach (var kv in _blockTracking)
            {
                if (kv.Value.BlockerId == robotId)
                    toRemoveTrack.Add(kv.Key);
            }
            foreach (var k in toRemoveTrack)
                _blockTracking.Remove(k);
>>>>>>> Stashed changes
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