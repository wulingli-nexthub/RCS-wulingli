namespace Graphic
{
    /// <summary>
    /// 视图/坐标变换控制类：
    /// - 不直接持有窗体，只操作 Scale / OffsetX / OffsetY。
    /// - 提供统一的“居中显示”、“窗口缩放时居中”、“以鼠标为中心缩放”、“拖动平移”、“重置视图”等操作。
    /// - 世界坐标单位为米，屏幕坐标单位为像素。
    /// </summary>
    public class WorldTransform
    {
        private readonly DrawGrid _grid;

        /// <summary>
        /// 用指定的网格对象构造视图变换控制器
        /// </summary>
        /// <param name="grid"></param>
        public WorldTransform(DrawGrid grid)
        {
            _grid = grid;
        }

        /// <summary>
        /// 窗体加载时，让整个世界网格居中显示，并记录初始缩放/偏移
        /// </summary>
        /// <param name="clientWidth"></param>
        /// <param name="clientHeight"></param>
        /// <param name="margin">边距，单位像素</param>
        public void CentreGrid(int clientWidth, int clientHeight, double margin = 40)
        {
            if (clientWidth <= 0 || clientHeight <= 0)
                return;

            double worldWidthM = _grid.WorldWidthM;
            double worldHeightM = _grid.WorldHeightM;

            double scaleX = (clientWidth - margin * 2) / worldWidthM;
            double scaleY = (clientHeight - margin * 2) / worldHeightM;
            double newScale = System.Math.Min(scaleX, scaleY);

            double gridPixelWidth = worldWidthM * newScale;
            double gridPixelHeight = worldHeightM * newScale;

            double newOffsetX = (clientWidth - gridPixelWidth) / 2.0;
            double newOffsetY = (clientHeight - gridPixelHeight) / 2.0;

            _grid.Scale = newScale;
            _grid.OffsetX = newOffsetX;
            _grid.OffsetY = newOffsetY;

            _grid.InitialScale = _grid.Scale;
            _grid.InitialOffsetX = _grid.OffsetX;
            _grid.InitialOffsetY = _grid.OffsetY;
        }

        /// <summary>
        /// 保持当前缩放比例，重新计算OffsetX/OffsetY
        /// 实现在窗口大小变化时让网格居中显示
        /// </summary>
        /// <param name="clientWidth"></param>
        /// <param name="clientHeight"></param>
        public void RecenterOnResize(int clientWidth, int clientHeight)
        {
            if (clientWidth <= 0 || clientHeight <= 0)
                return;

            double worldWidthM = _grid.WorldWidthM;
            double worldHeightM = _grid.WorldHeightM;

            double gridPixelWidth = worldWidthM * _grid.Scale;
            double gridPixelHeight = worldHeightM * _grid.Scale;

            _grid.OffsetX = (clientWidth - gridPixelWidth) / 2.0;
            _grid.OffsetY = (clientHeight - gridPixelHeight) / 2.0;
        }

        /// <summary>
        /// 鼠标滚轮缩放：
        /// 以鼠标所在屏幕坐标为缩放中心，放大/缩小视图，
        /// 并调整 OffsetX/OffsetY，保证缩放前后的“鼠标指向的世界坐标”不变。
        /// </summary>
        /// <param name="delta">鼠标滚轮增量（MouseEventArgs.Delta）。大于0表示向前滚。</param>
        /// <param name="mouseX">鼠标当前屏幕 X 坐标（像素）。</param>
        /// <param name="mouseY">鼠标当前屏幕 Y 坐标（像素）。</param>
        public void ZoomAt(int delta, float mouseX, float mouseY)
        {
            var mouseWorldBefore = _grid.ScreenToWorld(mouseX, mouseY);            // 缩放前，鼠标对应的世界坐标

            double zoomFactor = (delta > 0) ? 1.1 : 0.9;     // 根据滚轮方向选择缩放因子，每次缩放10%
            double newScale = _grid.Scale * zoomFactor;

            if (newScale < 5) newScale = 5;         // 限制缩放范围，避免过大或过小
            if (newScale > 200) newScale = 200;

            _grid.Scale = newScale;

            // 调整偏移，使缩放后鼠标仍然对应同一个世界点
            _grid.OffsetX = mouseX - mouseWorldBefore.X * _grid.Scale;
            _grid.OffsetY = mouseY - mouseWorldBefore.Y * _grid.Scale;
        }

        /// <summary>
        /// 平移（拖动画布）：
        /// 将屏幕上的像素位移 dx/dy 直接叠加到 OffsetX/OffsetY。
        /// </summary>
        /// <param name="dx">X 方向屏幕位移（像素）。</param>
        /// <param name="dy">Y 方向屏幕位移（像素）。</param>
        public void Pan(int dx, int dy)
        {
            _grid.OffsetX += dx;
            _grid.OffsetY += dy;
        }

        /// <summary>
        /// 将缩放重置回初始值，并根据当前窗口大小重新居中显示世界网格。
        /// 通常在“重置按钮”或重新初始化视图时调用。
        /// </summary>
        /// <param name="clientWidth">当前客户区宽度（像素）。</param>
        /// <param name="clientHeight">当前客户区高度（像素）。</param>
        public void ResetAndCenter(int clientWidth, int clientHeight)
        {
            _grid.Scale = _grid.InitialScale;

            if (clientWidth <= 0 || clientHeight <= 0)
                return;

            double worldWidthM = _grid.WorldWidthM;
            double worldHeightM = _grid.WorldHeightM;

            double gridPixelWidth = worldWidthM * _grid.Scale;
            double gridPixelHeight = worldHeightM * _grid.Scale;

            _grid.OffsetX = (clientWidth - gridPixelWidth) / 2.0;
            _grid.OffsetY = (clientHeight - gridPixelHeight) / 2.0;
        }
    }
}
