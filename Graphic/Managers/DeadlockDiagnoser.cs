using System.Collections.Generic;
using GridDemo.Models;
using GridDemo.Models.Pathfinding;

namespace GridDemo.Managers
{
    /// <summary>
    /// 前方堵路机器人的停止原因
    /// </summary>
    public enum EnumBlockerStopReason
    {
        Unknown = 0,
        /// <summary> 已到达终点，在目标点停顿等待（如装卸货） </summary>
        GoalPauseWait = 1,
        /// <summary> 原地转向中（速度为0但角度在变化） </summary>
        InPlaceTurn = 2,
        /// <summary> 排队等待（前方也被堵） </summary>
        QueueWait = 3
    }

    /// <summary>
    /// 死锁类型
    /// </summary>
    public enum EnumDeadlockType
    {
        None = 0,
        /// <summary> 双方交换位置互锁 </summary>
        SwapPosition = 1,
        /// <summary> 十字路口互锁（同路口不同出口互等） </summary>
        CrossJunction = 2,
        /// <summary> wait-for 链成环 </summary>
        WaitForCycle = 3
    }

    /// <summary>
    /// 阻塞诊断结果
    /// </summary>
    public sealed class BlockDiagnosis
    {
        public EnumBlockerStopReason StopReason { get; set; }
        public EnumDeadlockType DeadlockType { get; set; }
        public bool IsDeadlock => DeadlockType != EnumDeadlockType.None;

        /// <summary> 低优先级方应绕路 </summary>
        public bool ShouldReroute { get; set; }
        /// <summary> 高优先级方应原地等待 </summary>
        public bool ShouldHoldAndWait { get; set; }
        /// <summary> 非死锁时的建议等待帧数（替代固定150帧） </summary>
        public int SuggestedWaitFrames { get; set; }
    }

    /// <summary>
    /// 实时诊断前方堵路机器人的停止原因，判断是否构成死锁，
    /// 并根据优先级给出绕路/等待决策。
    /// </summary>
    internal static class DeadlockDiagnoser
    {
        /// <summary>
        /// 分析前方堵路机器人的停止原因
        /// </summary>
        public static EnumBlockerStopReason AnalyzeStopReason(
            RobotInstance blocker,
            int gridCount,
            Dictionary<int, int> occupiedByKey)
        {
            // 1) 已到达终点停留
            GridPos blockerPos = blocker.GetGridPos_NoLock();
            GridPos? blockerGoal = blocker.AutoNavigator.GetGoalGridSnapshot();
            if (blockerGoal.HasValue && blockerPos.Equals(blockerGoal.Value))
            {
                return EnumBlockerStopReason.GoalPauseWait;
            }

            // 2) 原地转向：有路径但当前速度为0，且路径下一步可通行
            List<GridPos> blockerPath = blocker.AutoNavigator.GetPathGridSnapshot();
            if (blockerPath != null && blockerPath.Count > 1)
            {
                // 找到blocker在路径中的位置
                int blockerStartIdx = -1;
                for (int j = 0; j < blockerPath.Count; j++)
                {
                    if (blockerPath[j].Equals(blockerPos))
                    {
                        blockerStartIdx = j;
                        break;
                    }
                }

                if (blockerStartIdx >= 0 && blockerStartIdx + 1 < blockerPath.Count)
                {
                    GridPos blockerNext = blockerPath[blockerStartIdx + 1];
                    int blockerNextKey = blockerNext.Y * gridCount + blockerNext.X;

                    // 前方没有其他机器人占据 → 说明blocker可能在转向
                    if (!occupiedByKey.ContainsKey(blockerNextKey))
                    {
                        return EnumBlockerStopReason.InPlaceTurn;
                    }
                }
            }

            // 3) 排队等待：blocker的下一格也被别人占据
            return EnumBlockerStopReason.QueueWait;
        }

        /// <summary>
        /// 检测是否构成死锁，并给出处置决策
        /// </summary>
        public static BlockDiagnosis Diagnose(
            RobotInstance self,
            RobotInstance blocker,
            int gridCount,
            Dictionary<int, int> occupiedByKey,
            Dictionary<int, int> waitForMap)
        {
            var diagnosis = new BlockDiagnosis();
            diagnosis.StopReason = AnalyzeStopReason(blocker, gridCount, occupiedByKey);

            GridPos selfPos = self.GetGridPos_NoLock();
            GridPos blockerPos = blocker.GetGridPos_NoLock();

            // — 获取双方路径下一格 —
            GridPos? selfNext = GetNextCell(self, selfPos);
            GridPos? blockerNext = GetNextCell(blocker, blockerPos);

            // 1) 双方交换位置死锁：self要去blocker的位，blocker要去self的位
            if (selfNext.HasValue && blockerNext.HasValue
                && selfNext.Value.Equals(blockerPos) && blockerNext.Value.Equals(selfPos))
            {
                diagnosis.DeadlockType = EnumDeadlockType.SwapPosition;
                AssignByPriority(diagnosis, self, blocker);
                return diagnosis;
            }

            // 2) 十字路口互锁：双方都堵在同一个格子的相邻格、各自要去对方的方向
            //    简化判断：双方都在等待、且互相出现在对方的 waitFor 链中
            if (selfNext.HasValue && blockerNext.HasValue
                && AreNeighbors(selfPos, blockerPos)
                && selfNext.Value.Equals(blockerPos))
            {
                // blocker 的下一步也被其他人堵，且那个人最终等的是 self → 环
                if (HasCycle(waitForMap, self.Id))
                {
                    diagnosis.DeadlockType = EnumDeadlockType.CrossJunction;
                    AssignByPriority(diagnosis, self, blocker);
                    return diagnosis;
                }
            }

            // 3) wait-for 链成环
            if (HasCycle(waitForMap, self.Id))
            {
                diagnosis.DeadlockType = EnumDeadlockType.WaitForCycle;
                AssignByPriority(diagnosis, self, blocker);
                return diagnosis;
            }

            // 非死锁：根据停止原因给出建议等待帧数
            diagnosis.DeadlockType = EnumDeadlockType.None;
            diagnosis.ShouldHoldAndWait = true;
            diagnosis.ShouldReroute = false;

            switch (diagnosis.StopReason)
            {
                case EnumBlockerStopReason.InPlaceTurn:
                    diagnosis.SuggestedWaitFrames = 30;   // 转向约0.6秒
                    break;
                case EnumBlockerStopReason.GoalPauseWait:
                    diagnosis.SuggestedWaitFrames = 40;   // 终点停顿约0.8秒
                    break;
                case EnumBlockerStopReason.QueueWait:
                    diagnosis.SuggestedWaitFrames = 60;   // 排队约1.2秒
                    break;
                default:
                    diagnosis.SuggestedWaitFrames = 25;   // 未知约0.5秒
                    break;
            }


            return diagnosis;
        }

        /// <summary>
        /// 根据优先级分配决策：距离近(优先级高)的等待，距离远(优先级低)的绕路
        /// 注意：在 ApplyPathClaiming_NoLock 中 items 已按距离排序，距离近的先 claim，
        /// 所以这里用距离作为优先级指标。
        /// </summary>
        private static void AssignByPriority(BlockDiagnosis diagnosis, RobotInstance self, RobotInstance blocker)
        {
            GridPos? selfGoal = self.AutoNavigator.GetGoalGridSnapshot();
            GridPos? blockerGoal = blocker.AutoNavigator.GetGoalGridSnapshot();
            GridPos selfPos = self.GetGridPos_NoLock();
            GridPos blockerPos = blocker.GetGridPos_NoLock();

            int selfDist = selfGoal.HasValue
                ? System.Math.Abs(selfPos.X - selfGoal.Value.X) + System.Math.Abs(selfPos.Y - selfGoal.Value.Y)
                : int.MaxValue;
            int blockerDist = blockerGoal.HasValue
                ? System.Math.Abs(blockerPos.X - blockerGoal.Value.X) + System.Math.Abs(blockerPos.Y - blockerGoal.Value.Y)
                : int.MaxValue;

            // 距离更远（优先级低）的一方绕路
            if (selfDist > blockerDist || (selfDist == blockerDist && self.Id > blocker.Id))
            {
                // self 是低优先级，self 绕路
                diagnosis.ShouldReroute = true;
                diagnosis.ShouldHoldAndWait = false;
            }
            else
            {
                // self 是高优先级，self 等待；blocker 应被通知绕路
                diagnosis.ShouldReroute = false;
                diagnosis.ShouldHoldAndWait = true;
            }
        }

        private static GridPos? GetNextCell(RobotInstance r, GridPos current)
        {
            List<GridPos> path = r.AutoNavigator.GetPathGridSnapshot();
            if (path == null || path.Count == 0) return null;

            for (int j = 0; j < path.Count; j++)
            {
                if (path[j].Equals(current) && j + 1 < path.Count)
                {
                    return path[j + 1];
                }
            }
            return null;
        }

        private static bool AreNeighbors(GridPos a, GridPos b)
        {
            int dx = System.Math.Abs(a.X - b.X);
            int dy = System.Math.Abs(a.Y - b.Y);
            return (dx + dy) == 1;
        }

        /// <summary>
        /// wait-for 链环检测：从 startId 沿 waitForMap 遍历，检测是否回到 startId
        /// </summary>
        private static bool HasCycle(Dictionary<int, int> waitForMap, int startId)
        {
            var visited = new HashSet<int>();
            int cur = startId;
            while (waitForMap.TryGetValue(cur, out int next))
            {
                if (next == startId) return true;
                if (!visited.Add(cur)) return false; // 环不经过 start
                cur = next;
            }
            return false;
        }
    }
}