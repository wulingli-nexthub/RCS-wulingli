using Graphic.Maps;
using GridDemo.RobotModels.Pathfinding;
using System;
using System.Collections.Generic;

namespace GridDemo.Robots
{
    internal sealed class GridCellClaimBoard
    {
        private readonly int _gridCount;
        private readonly ObstacleMap _obstacleMap;

        // cellKey -> robotId
        private readonly Dictionary<int, int> _claimedBy = new Dictionary<int, int>();

        // robotId -> claimed cells（按路径顺序记录）
        private readonly Dictionary<int, List<GridPos>> _claimedCellsByRobotId = new Dictionary<int, List<GridPos>>();

        public GridCellClaimBoard(int gridCount, ObstacleMap obstacleMap)
        {
            _gridCount = gridCount;
            _obstacleMap = obstacleMap ?? throw new ArgumentNullException(nameof(obstacleMap));
        }

        /// <summary>
        /// 清空全部抢占（用于重置仿真/清理）。
        /// 注意：整段锁定模式下，不能在每帧调用。
        /// </summary>
        public void ResetAll()
        {
            _claimedBy.Clear();
            _claimedCellsByRobotId.Clear();
        }

        /// <summary>
        /// 释放某个机器人当前占用的全部格子锁（用于目标切换/导航禁用/机器人删除等）。
        /// </summary>
        public void ReleaseAllByRobot(int robotId)
        {
            List<GridPos> list;
            if (!_claimedCellsByRobotId.TryGetValue(robotId, out list) || list.Count == 0)
            {
                _claimedCellsByRobotId.Remove(robotId);
                return;
            }

            for (int i = 0; i < list.Count; i++)
            {
                GridPos cell = list[i];
                int key = cell.Y * _gridCount + cell.X;

                int owner;
                if (_claimedBy.TryGetValue(key, out owner) && owner == robotId)
                {
                    _claimedBy.Remove(key);
                }
            }

            _claimedCellsByRobotId.Remove(robotId);
        }

        /// <summary>
        /// 整段锁定到终点：从 path 的 startIndex 开始，尽可能抢占到终点；
        /// 若遇到别人的锁则停止，返回“已抢到的前缀”（并写入 board）。
        /// </summary>
        public List<GridPos> ClaimPathToGoalOrPrefix(int robotId, List<GridPos> path, int startIndex)
        {
            // 先清掉该机器人旧锁，避免“旧路径残留锁”影响新路径
            ReleaseAllByRobot(robotId);

            if (path == null || path.Count == 0)
            {
                return null;
            }

            if (startIndex < 0)
            {
                startIndex = 0;
            }
            if (startIndex >= path.Count)
            {
                startIndex = Math.Max(0, path.Count - 1);
            }

            var claimed = new List<GridPos>(Math.Max(4, path.Count - startIndex));

            for (int i = startIndex; i < path.Count; i++)
            {
                GridPos p = path[i];

                if (p.X < 0 || p.Y < 0 || p.X >= _gridCount || p.Y >= _gridCount)
                {
                    continue;
                }

                if (_obstacleMap.IsObstacle(p))
                {
                    continue;
                }

                int key = p.Y * _gridCount + p.X;

                int owner;
                if (_claimedBy.TryGetValue(key, out owner))
                {
                    if (owner != robotId)
                    {   // 被其它机器人占用：只能抢占前缀，立即停止
                        break;
                    }

                    // 已经是自己占用（理论上不应该，因为上面 ReleaseAllByRobot 了），但做容错
                    claimed.Add(p);
                    continue;
                }

                _claimedBy[key] = robotId;
                claimed.Add(p);
            }

            if (claimed.Count > 0)
            {
                _claimedCellsByRobotId[robotId] = claimed;
                return new List<GridPos>(claimed);
            }

            _claimedCellsByRobotId.Remove(robotId);
            return null;
        }

        public bool IsClaimedByOther(int robotId, GridPos p)
        {
            int key = p.Y * _gridCount + p.X;

            int owner;
            if (_claimedBy.TryGetValue(key, out owner))
            {
                return owner != robotId;
            }

            return false;
        }

        public void ReleaseCell(int robotId, GridPos cell)
        {
            int key = cell.Y * _gridCount + cell.X;

            int owner;
            if (_claimedBy.TryGetValue(key, out owner) && owner == robotId)
            {
                _claimedBy.Remove(key);
            }

            List<GridPos> list;
            if (_claimedCellsByRobotId.TryGetValue(robotId, out list))
            {
                for (int i = list.Count - 1; i >= 0; i--)
                {
                    if (list[i].Equals(cell))
                    {
                        list.RemoveAt(i);
                        break;
                    }
                }

                if (list.Count == 0)
                {
                    _claimedCellsByRobotId.Remove(robotId);
                }
            }
        }

        public Dictionary<int, List<GridPos>> GetClaimedCellsSnapshot()
        {
            var snapshot = new Dictionary<int, List<GridPos>>(_claimedCellsByRobotId.Count);

            foreach (var kv in _claimedCellsByRobotId)
            {
                snapshot[kv.Key] = new List<GridPos>(kv.Value);
            }

            return snapshot;
        }
    }
}