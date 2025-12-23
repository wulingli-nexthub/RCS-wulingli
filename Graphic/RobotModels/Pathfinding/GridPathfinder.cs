using System;
using System.Collections.Generic;

namespace Graphic.RobotModels.Pathfinding
{
    internal enum EnumPathfindingAlgorithm
    {
        Dijkstra = 0,
        AStar = 1
    }

    internal readonly struct GridPos : IEquatable<GridPos>
    {
        public GridPos(int x, int y)
        {
            X = x;
            Y = y;
        }

        public int X { get; }
        public int Y { get; }

        public bool Equals(GridPos other) => X == other.X && Y == other.Y;
        public override bool Equals(object obj) => obj is GridPos other && Equals(other);
        public override int GetHashCode() => (X * 397) ^ Y;
        public override string ToString() => "(" + X + "," + Y + ")";
    }

    internal static class GridPathfinder
    {
        // 4 邻接：保证路径沿网格中心线（不走斜线）
        private static readonly GridPos[] Neighbors4 =
        {
            new GridPos(1, 0),
            new GridPos(-1, 0),
            new GridPos(0, 1),
            new GridPos(0, -1),
        };

        public static List<GridPos> FindPath(
            int width,
            int height,
            GridPos start,
            GridPos goal,
            Func<GridPos, bool> isWalkable,
            EnumPathfindingAlgorithm algorithm)
        {
            if (width <= 0) throw new ArgumentOutOfRangeException(nameof(width));
            if (height <= 0) throw new ArgumentOutOfRangeException(nameof(height));
            if (isWalkable == null) throw new ArgumentNullException(nameof(isWalkable));

            if (!InBounds(start, width, height) || !InBounds(goal, width, height))
            {
                return new List<GridPos>();
            }

            if (!isWalkable(start) || !isWalkable(goal))
            {
                return new List<GridPos>();
            }

            // Dijkstra: f = g
            // A*: f = g + h
            var open = new MinQueue<GridPos>();
            var cameFrom = new Dictionary<GridPos, GridPos>();
            var gScore = new Dictionary<GridPos, int> { [start] = 0 };

            open.Push(start, 0);

            while (open.Count > 0)
            {
                GridPos current = open.Pop(out int currentF);

                if (current.Equals(goal))
                {
                    return Reconstruct(cameFrom, current);
                }

                int currentG = gScore[current];

                for (int i = 0; i < Neighbors4.Length; i++)
                {
                    GridPos next = new GridPos(current.X + Neighbors4[i].X, current.Y + Neighbors4[i].Y);

                    if (!InBounds(next, width, height))
                    {
                        continue;
                    }

                    if (!isWalkable(next))
                    {
                        continue;
                    }

                    int tentativeG = currentG + 1; // 4 邻接每步成本=1

                    if (gScore.TryGetValue(next, out int oldG) && tentativeG >= oldG)
                    {
                        continue;
                    }

                    cameFrom[next] = current;
                    gScore[next] = tentativeG;

                    int h = (algorithm == EnumPathfindingAlgorithm.AStar) ? Manhattan(next, goal) : 0;
                    int f = tentativeG + h;

                    open.Push(next, f);
                }
            }

            return new List<GridPos>();
        }

        private static List<GridPos> Reconstruct(Dictionary<GridPos, GridPos> cameFrom, GridPos current)
        {
            var path = new List<GridPos> { current };
            while (cameFrom.TryGetValue(current, out GridPos prev))
            {
                current = prev;
                path.Add(current);
            }

            path.Reverse();
            return path;
        }

        private static bool InBounds(GridPos p, int width, int height)
            => p.X >= 0 && p.Y >= 0 && p.X < width && p.Y < height;

        private static int Manhattan(GridPos a, GridPos b)
            => Math.Abs(a.X - b.X) + Math.Abs(a.Y - b.Y);

        /// <summary>
        /// 简单最小优先队列（允许重复入队；出队时不做 decrease-key）
        /// </summary>
        private sealed class MinQueue<T>
        {
            private readonly List<(T item, int priority)> _heap = new List<(T, int)>();

            public int Count => _heap.Count;

            public void Push(T item, int priority)
            {
                _heap.Add((item, priority));
                SiftUp(_heap.Count - 1);
            }

            public T Pop(out int priority)
            {
                var root = _heap[0];
                priority = root.priority;

                int last = _heap.Count - 1;
                _heap[0] = _heap[last];
                _heap.RemoveAt(last);

                if (_heap.Count > 0)
                {
                    SiftDown(0);
                }

                return root.item;
            }

            private void SiftUp(int i)
            {
                while (i > 0)
                {
                    int p = (i - 1) / 2;
                    if (_heap[p].priority <= _heap[i].priority)
                    {
                        break;
                    }

                    var tmp = _heap[p];
                    _heap[p] = _heap[i];
                    _heap[i] = tmp;
                    i = p;
                }
            }

            private void SiftDown(int i)
            {
                int n = _heap.Count;
                while (true)
                {
                    int l = i * 2 + 1;
                    int r = l + 1;
                    if (l >= n) break;

                    int min = (r < n && _heap[r].priority < _heap[l].priority) ? r : l;

                    if (_heap[i].priority <= _heap[min].priority)
                    {
                        break;
                    }

                    var tmp = _heap[i];
                    _heap[i] = _heap[min];
                    _heap[min] = tmp;
                    i = min;
                }
            }
        }
    }
}