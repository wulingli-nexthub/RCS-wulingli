using GridDemo.RobotModels.Pathfinding;
using System;

namespace Graphic.Maps
{
    /// <summary>
    /// 障碍物网格：
    /// - 以网格坐标（GridPos）进行记录；
    /// - 支持 Toggle（点一次设置，再点一次取消）；
    /// - 并提供 IsObstacle 供寻路/可视化查询。
    /// </summary>
    internal sealed class ObstacleMap
    {
        private readonly object _syncRoot = new object();
        private readonly bool[,] _cells;

        public ObstacleMap(int width, int height)
        {
            if (width <= 0) throw new ArgumentOutOfRangeException(nameof(width));
            if (height <= 0) throw new ArgumentOutOfRangeException(nameof(height));

            Width = width;
            Height = height;
            _cells = new bool[width, height];
        }

        public int Width { get; }
        public int Height { get; }

        public bool IsObstacle(GridPos p)
        {
            lock (_syncRoot)
            {
                if (!InBounds_NoLock(p))
                {
                    return false;
                }

                return _cells[p.X, p.Y];
            }
        }

        public bool Toggle(GridPos p)
        {
            lock (_syncRoot)
            {
                if (!InBounds_NoLock(p))
                {
                    return false;
                }

                _cells[p.X, p.Y] = !_cells[p.X, p.Y];
                return _cells[p.X, p.Y];
            }
        }

        public bool[,] GetSnapshot()
        {
            lock (_syncRoot)
            {
                var snapshot = new bool[Width, Height];
                Array.Copy(_cells, snapshot, _cells.Length);
                return snapshot;
            }
        }

        private bool InBounds_NoLock(GridPos p)
        {
            return p.X >= 0 && p.Y >= 0 && p.X < Width && p.Y < Height;
        }
    }
}
