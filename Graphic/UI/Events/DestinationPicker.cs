using GridDemo.RobotModels;
using GridDemo.RobotModels.Pathfinding;
using GridDemo.WorldView;
using System;
using System.Windows.Forms;

namespace GridDemo.Events
{
    /// <summary>
    /// 目的地选择器
    /// - 在画布上通过鼠标右键点击选择目标点；
    /// - 将鼠标屏幕坐标转换为世界坐标（米）；
    /// - 再将世界坐标离散到网格坐标（<see cref="GridPos"/>）；
    /// - 最后将目标设置给 <see cref="RobotAutoNavigator"/>，必要时立即重建路径。
    /// </summary>
    internal sealed class DestinationPicker
    {
        private readonly WorldTransform _transform;
        private readonly RobotAutoNavigator _navigator;

        private readonly Func<double> _getWorldWidthM;
        private readonly Func<double> _getWorldHeightM;
        private readonly double _cellSizeM;

        /// <summary>
        /// 创建目的地选择器实例。
        /// </summary>
        public DestinationPicker(
            WorldTransform transform,
            RobotAutoNavigator navigator,
            Func<double> getWorldWidthM,
            Func<double> getWorldHeightM,
            double cellSizeM)
        {
            _transform = transform ?? throw new ArgumentNullException(nameof(transform));
            _navigator = navigator ?? throw new ArgumentNullException(nameof(navigator));
            _getWorldWidthM = getWorldWidthM ?? throw new ArgumentNullException(nameof(getWorldWidthM));
            _getWorldHeightM = getWorldHeightM ?? throw new ArgumentNullException(nameof(getWorldHeightM));
            _cellSizeM = cellSizeM;
        }

        /// <summary>
        /// 尝试从鼠标事件中“拾取”目标点。
        /// 触发条件：鼠标右键点击。
        /// </summary>
        /// <param name="e">鼠标事件参数。</param>
        /// <returns>
        /// 成功设置目标则返回 true；若不是右键则返回 false（表示本次事件不由该类处理）。
        /// </returns>
        public bool TryPick(MouseEventArgs e)
        {
            if (e == null)
                throw new ArgumentNullException(nameof(e));

            if (e.Button != MouseButtons.Right)          // 仅响应右键点击
            {
                return false;
            }

            var world = _transform.ScreenToWorld(e.X, e.Y);           // 屏幕坐标转换为世界坐标（米）
            var size = GetGridSize();                             // 获取当前网格尺寸
            GridPos goal = WorldToGrid(world.X, world.Y, size.gridW, size.gridH);             // 世界坐标转换为网格坐标

            // 如果当前正在自动巡航：立刻重算路径
            _navigator.SetGoal(goal, rebuildIfEnabled: true);

            return true;
        }

        /// <summary>
        /// 将世界宽高（米）换算成网格宽高（格子数）。
        /// 使用 Round：保证在浮点误差下网格尺寸相对稳定；并且至少为 1x1。
        /// </summary>
        private (int gridW, int gridH) GetGridSize()
        {
            double worldWidth = _getWorldWidthM();
            double worldHeight = _getWorldHeightM();

            int gridW = Math.Max(1, (int)Math.Round(worldWidth / _cellSizeM));
            int gridH = Math.Max(1, (int)Math.Round(worldHeight / _cellSizeM));
            return (gridW, gridH);
        }

        /// <summary>
        /// 将世界坐标（米）转换为网格坐标（格子索引）。
        /// 采用 Floor：例如 wx 落在 [0, cellSize) 映射为 0，落在 [cellSize, 2*cellSize) 映射为 1。
        /// 并对结果做边界裁剪，避免落在网格外部。
        /// </summary>
        private GridPos WorldToGrid(double wx, double wy, int gridW, int gridH)
        {
            int gx = (int)Math.Floor(wx / _cellSizeM);
            int gy = (int)Math.Floor(wy / _cellSizeM);

            // 边界保护：将落在范围外的坐标裁剪回合法区间
            if (gx < 0)
                gx = 0;
            if (gy < 0)
                gy = 0;
            if (gx >= gridW)
                gx = gridW - 1;
            if (gy >= gridH)
                gy = gridH - 1;

            return new GridPos(gx, gy);
        }
    }
}