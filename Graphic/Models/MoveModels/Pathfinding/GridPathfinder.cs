using System;
using System.Collections.Generic;

namespace GridDemo.Models.Pathfinding
{
    /// <summary>
    /// 算法枚举，迪杰斯特拉或A*
    /// </summary>
    internal enum EnumPathfindingAlgorithm
    {
        Dijkstra = 0,
        AStar = 1,
        Serpentine = 2
    }

    /// <summary>
    /// 网格坐标
    /// </summary>
    internal readonly struct GridPos : IEquatable<GridPos>
    {
        public GridPos(int x, int y)
        {
            X = x;
            Y = y;
        }

        public int X { get; }
        public int Y { get; }

        public bool Equals(GridPos other) => X == other.X && Y == other.Y;              // 比较两个坐标是否相等
    }

    internal static class GridPathfinder
    {
        /// <summary>
        /// 4 邻接（上下左右）：保证路径沿网格中心线（不走斜线）
        /// </summary>
        private static readonly GridPos[] Neighbors4 =
        {
            new GridPos(1, 0),
            new GridPos(-1, 0),
            new GridPos(0, 1),
            new GridPos(0, -1),
        };

        /// <summary>
        /// 在给定的网格上寻找路径
        /// </summary>
        /// <returns>
        /// 找到则返回路径（包含起点和终点）；否则返回空列表
        /// </returns>
        /// <exception cref="ArgumentOutOfRangeException">高度/宽度小于等于0</exception>
        /// <exception cref="ArgumentNullException">格子不可通行</exception>
        public static List<GridPos> FindPath(
            int width,
            int height,
            GridPos start,
            GridPos goal,
            Func<GridPos, bool> isWalkable,
            EnumPathfindingAlgorithm algorithm)
        {

            // 参数校验，确保宽高和回调合法
            if (width <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(width));
            }
            if (height <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(height));
            }
            if (isWalkable == null)
            {
                throw new ArgumentNullException(nameof(isWalkable));
            }

            // 起点/终点越界，直接返回空路径
            if (!InBounds(start, width, height) || !InBounds(goal, width, height))
            {
                return new List<GridPos>();
            }

            // 起点/终点不可通行，直接返回空路径
            if (!isWalkable(start) || !isWalkable(goal))
            {
                return new List<GridPos>();
            }

            // 兼容：蛇形算法不走这里（由 RobotAutoNavigator 直接调用 SerpentinePathfinder 生成路径）
            if (algorithm == EnumPathfindingAlgorithm.Serpentine)
            {
                return new List<GridPos>();
            }

            // 核心思路：
            // - gScore：起点到某点的最短已知代价 g
            // - cameFrom：记录路径回溯（next -> prev）
            // - open：待扩展集合，按 f 最小优先出队
            //   Dijkstra: f = g
            //   A*:       f = g + h（h 为启发式）
            var open = new MinQueue<GridPos>();   // 最小优先队列（开集），存储待扩展的节点以及对应优先级 f 值
            var cameFrom = new Dictionary<GridPos, GridPos>();  // 路径回溯字典，记录每个节点的前驱节点
            var gScore = new Dictionary<GridPos, int> { [start] = 0 };  // gScore 字典，记录起点到各节点的最短已知代价 g

            open.Push(start, 0);     // 起点入队，f=0

            // 主循环：不断取出当前 f 最小的点进行扩展
            while (open.Count > 0)
            {
                // 取出当前 f 最小的点
                GridPos current = open.Pop(out int currentF);

                // 到达终点，重建路径并返回
                if (current.Equals(goal))
                {
                    return Reconstruct(cameFrom, current);
                }

                // 扩展当前点的邻居
                int currentG = gScore[current];

                // 遍历 4 邻接
                for (int i = 0; i < Neighbors4.Length; i++)
                {
                    GridPos next = new GridPos(current.X + Neighbors4[i].X, current.Y + Neighbors4[i].Y);

                    if (!InBounds(next, width, height))  // 越界，跳过
                    {
                        continue;
                    }

                    if (!isWalkable(next))    // 不可通行，跳过
                    {
                        continue;
                    }

                    int tentativeG = currentG + 1; // 4 邻接每步成本=1

                    // 已知更优路径，跳过
                    if (gScore.TryGetValue(next, out int oldG) && tentativeG >= oldG)
                    {
                        continue;
                    }

                    // 记录更优路径
                    cameFrom[next] = current;
                    gScore[next] = tentativeG;

                    // A*：使用曼哈顿距离作为启发式 h（适用于 4 邻接移动）
                    // Dijkstra：h=0
                    int h = (algorithm == EnumPathfindingAlgorithm.AStar) ? Manhattan(next, goal) : 0;
                    int f = tentativeG + h;

                    open.Push(next, f);   // 入队待扩展
                }
            }

            return new List<GridPos>();   // 无路径，返回空列表
        }

        /// <summary>
        /// 从终点开始，沿 cameFrom 回溯路径
        /// </summary>
        private static List<GridPos> Reconstruct(Dictionary<GridPos, GridPos> cameFrom, GridPos current)
        {
            // current 初始为 goal，将其加入，然后不断找 prev
            var path = new List<GridPos> { current };
            while (cameFrom.TryGetValue(current, out GridPos prev))
            {
                current = prev;
                path.Add(current);
            }

            // 反转路径，使其从 start 到 goal
            path.Reverse();
            return path;
        }

        /// <summary>
        /// 判断坐标是否在网格范围内
        /// </summary>
        private static bool InBounds(GridPos p, int width, int height)
            => p.X >= 0 && p.Y >= 0 && p.X < width && p.Y < height;

        /// <summary>
        /// 曼哈顿距离（|dx| + |dy|），适用于 4 邻接移动的 A* 启发式。
        /// </summary>
        private static int Manhattan(GridPos a, GridPos b)
            => Math.Abs(a.X - b.X) + Math.Abs(a.Y - b.Y);

        /// <summary>
        /// 简单最小优先队列（允许重复入队；出队时不做 decrease-key）
        /// </summary>
        private sealed class MinQueue<T>
        {
            // 二叉堆数组表示：每个元素为 (item, priority)。
            // priority 越小越优先出队。
            private readonly List<(T item, int priority)> _heap = new List<(T, int)>();

            // 当前队列个数
            public int Count => _heap.Count;

            /// <summary>
            /// 入队，插入到堆尾并上滤
            /// </summary>
            public void Push(T item, int priority)
            {
                _heap.Add((item, priority));
                SiftUp(_heap.Count - 1);
            }

            /// <summary>
            /// 出队，弹出优先级最小的元素，取堆顶并下滤
            /// </summary>
            public T Pop(out int priority)
            {
                // 堆顶元素即最小优先级元素
                var root = _heap[0];
                priority = root.priority;

                // 用堆尾元素覆盖堆顶，然后移除堆尾并下滤
                int last = _heap.Count - 1;
                _heap[0] = _heap[last];
                _heap.RemoveAt(last);

                if (_heap.Count > 0)
                {
                    SiftDown(0);
                }

                return root.item;
            }

            /// <summary>
            /// 上滤：当子节点 priority 小于父节点时交换，直到满足堆性质。
            /// </summary>
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

            /// <summary>
            /// 下滤：与左右子中较小者交换，直到满足堆性质。
            /// </summary>
            private void SiftDown(int i)
            {
                int n = _heap.Count;
                while (true)
                {
                    int l = i * 2 + 1;
                    int r = l + 1;
                    if (l >= n)
                    {
                        break;
                    }

                    // 找到左右子节点中 priority 较小的那个
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