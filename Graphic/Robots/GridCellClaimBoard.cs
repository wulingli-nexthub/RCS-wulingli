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

        public void ClaimPath(int robotId, List<GridPos> path)
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