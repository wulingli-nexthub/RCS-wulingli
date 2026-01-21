using GridDemo.RobotModels.Pathfinding;
using System;

namespace GridDemo.Maps
{
    /// <summary>
    /// 障碍物网格（线程安全）：
    /// - 使用 <see cref="bool[,]"/> 记录每个格子是否为障碍物（true=障碍，false=可通行）；
    /// - 以网格坐标 <see cref="GridPos"/> 作为访问键（与寻路模块 <see cref="GridDemo.RobotModels.Pathfinding.GridPathfinder"/> 统一坐标系）；
    /// - 提供 Toggle 用于 UI 点击“设置/取消障碍物”；
    /// - 提供快照 GetSnapshot 用于绘制（避免绘制时与写入竞争）。
    ///
    /// 典型调用链（项目内）：
    /// - UI（<see cref="GridDemo.Form1"/>）鼠标点击调用 <see cref="Toggle"/>；
    /// - 自动导航（<see cref="GridDemo.RobotModels.RobotAutoNavigator"/>）通过注入的 isWalkableProvider 间接调用 <see cref="IsObstacle"/>；
    /// - 运动学碰撞（<see cref="GridDemo.RobotRuns.RobotMove"/>）的 isWorldWalkable 委托最终也会查询本地图。
    /// </summary>
    internal sealed class ObstacleMap
    {
        private readonly object _syncRoot = new object();

        // 障碍物数据：_cells[x,y] = true 表示该格子为障碍物。
        // 第一维是 X（列），第二维是 Y（行），与 GridPos.X/GridPos.Y 对齐。
        private readonly bool[,] _cells;

        public ObstacleMap(int width, int height)
        {
            if (width <= 0) throw new ArgumentOutOfRangeException(nameof(width));
            if (height <= 0) throw new ArgumentOutOfRangeException(nameof(height));

            Width = width;
            Height = height;
            _cells = new bool[width, height];
        }

        public int Width { get; }            // 网格宽度（列数）
        public int Height { get; }           // 网格高度（行数）

        /// <summary>
        /// 查询指定格子是否为障碍物。
        /// </summary>
        /// <param name="p">网格坐标</param>
        /// <returns>
        /// - 若 p 越界：返回 false（按“越界非障碍”处理，避免调用方索引异常）；
        /// - 否则返回该格子记录值：true=障碍，false=可通行。
        /// </returns>
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

        /// <summary>
        /// 切换指定格子的障碍状态（点一次设置，再点一次取消）。
        /// </summary>
        /// <param name="p">网格坐标</param>
        /// <returns>
        /// - 若 p 越界：返回 false（表示未发生变更）；
        /// - 否则返回切换后的状态：true=现在是障碍，false=现在可通行。
        /// </returns>
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

        /// <summary>
        /// 清空所有障碍物（线程安全）。
        /// </summary>
        public void Clear()
        {
            lock (_syncRoot)
            {
                Array.Clear(_cells, 0, _cells.Length);
            }
        }

        /// <summary>
        /// 获取当前障碍物网格的快照（深拷贝）。
        /// </summary>
        /// <remarks>
        /// 项目内用于 UI 绘制：绘制线程/回调拿到快照后可无锁读取，
        /// 避免绘制过程中与 Toggle 并发导致读写竞争。
        /// </remarks>
        public bool[,] GetSnapshot()
        {
            lock (_syncRoot)
            {
                var snapshot = new bool[Width, Height];
                Array.Copy(_cells, snapshot, _cells.Length);
                return snapshot;
            }
        }

        /// <summary>
        /// 边界检查（不加锁版本）：要求调用方已持有 <see cref="_syncRoot"/>。
        /// </summary>
        private bool InBounds_NoLock(GridPos p)
        {
            return p.X >= 0 && p.Y >= 0 && p.X < Width && p.Y < Height;
        }
    }
}
