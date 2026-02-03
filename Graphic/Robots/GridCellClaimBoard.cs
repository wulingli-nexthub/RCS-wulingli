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

        // robotId -> claimed cells
        private readonly Dictionary<int, List<GridPos>> _claimedCellsByRobotId = new Dictionary<int, List<GridPos>>();

        public GridCellClaimBoard(int gridCount, ObstacleMap obstacleMap)
        {
            _gridCount = gridCount;
            _obstacleMap = obstacleMap ?? throw new ArgumentNullException(nameof(obstacleMap));
        }

        public void Reset()
        {
            _claimedBy.Clear();
            _claimedCellsByRobotId.Clear();
        }

        public void ClaimPathPrefix(int robotId, List<GridPos> path)
        {
            if (path == null || path.Count == 0)
            {
                return;
            }

            List<GridPos> list;
            if (!_claimedCellsByRobotId.TryGetValue(robotId, out list))
            {
                list = new List<GridPos>(path.Count);
                _claimedCellsByRobotId[robotId] = list;
            }

            for (int i = 0; i < path.Count; i++)
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
                    {   // 被更高优先级/先到者占用：只能抢占前缀，立即停止
                        break;
                    }

                    continue;
                }

                _claimedBy[key] = robotId;
                list.Add(p);
            }
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

        /// <summary>
        /// “边走边释放”：当机器人确认自己已经完全进入到 nextCell 后，
        /// 释放之前占用的 currentCell（要求调用方保证 currentCell 是或者曾经是 robotId 占用的格子）。
        /// </summary>
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
            }
        }

        /// <summary>
        /// 抢占格子快照（robotId -> claimedCells）。
        /// 调用方如需线程安全，请在外层持有同一把 _robotLock。
        /// </summary>
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