namespace Graphic
{
    public class WorldTransform
    {
        private readonly DrawGrid _grid;

        public WorldTransform(DrawGrid grid)
        {
            _grid = grid;
        }

        /// <summary>
        /// 窗体加载时，让整个世界网格居中显示，并记录初始缩放/偏移
        /// </summary>
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
        /// 窗口尺寸改变时，保持当前 Scale 不变，让世界重新居中
        /// </summary>
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
        /// 鼠标滚轮缩放，以鼠标所在点为缩放中心
        /// </summary>
        public void ZoomAt(int delta, float mouseX, float mouseY)
        {
            // 缩放前，鼠标对应的世界坐标
            var mouseWorldBefore = _grid.ScreenToWorld(mouseX, mouseY);

            double zoomFactor = (delta > 0) ? 1.1 : 0.9;
            double newScale = _grid.Scale * zoomFactor;

            if (newScale < 5) newScale = 5;
            if (newScale > 200) newScale = 200;

            _grid.Scale = newScale;

            // 调整偏移，使缩放后鼠标仍然对应同一个世界点
            _grid.OffsetX = mouseX - mouseWorldBefore.X * _grid.Scale;
            _grid.OffsetY = mouseY - mouseWorldBefore.Y * _grid.Scale;
        }

        /// <summary>
        /// 平移（拖动画布），dx、dy 为屏幕像素偏移
        /// </summary>
        public void Pan(int dx, int dy)
        {
            _grid.OffsetX += dx;
            _grid.OffsetY += dy;
        }

        /// <summary>
        /// 重置缩放为初始值，并在当前窗口下居中
        /// </summary>
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
