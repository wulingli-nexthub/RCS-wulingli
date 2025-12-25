using System.Drawing;

namespace GridDemo.WorldView
{
    /// <summary>
    /// 世界坐标与屏幕坐标之间的变换器（平移 + 缩放）。
    /// 约定：
    /// - world 坐标（wx/wy）使用“世界单位”（例如米）；
    /// - screen 坐标（sx/sy）使用像素；
    /// - 变换公式：screen = world * scale + offset。
    /// 线程安全：内部用锁保护 scale/offset，允许在绘制线程与输入线程并发读写。
    /// </summary>
    internal class WorldTransform
    {
        private readonly object _syncRoot = new object();      // 同步锁，保护以下字段的并发访问

        private double _scale;
        private float _offsetX;
        private float _offsetY;

        internal WorldTransform(double scale, float offsetX, float offsetY)
        {
            _scale = scale;
            _offsetX = offsetX;
            _offsetY = offsetY;
        }

        /// <summary>
        /// 更新变换参数（通常由鼠标缩放/拖拽或居中策略调用）。
        /// 注意：该方法会整体替换 scale 与 offset，保证一次更新中的参数一致。
        /// </summary>
        internal void Update(double scale, float offsetX, float offsetY)
        {
            lock (_syncRoot)
            {
                _scale = scale;
                _offsetX = offsetX;
                _offsetY = offsetY;
            }
        }

        /// <summary>
        /// 将世界坐标转换为屏幕坐标（像素）。
        /// 公式：sx = wx * scale + offsetX；sy = wy * scale + offsetY。
        /// </summary>
        internal PointF WorldToScreen(double wx, double wy)
        {
            lock (_syncRoot)
            {
                float sx = (float)(wx * _scale + _offsetX);
                float sy = (float)(wy * _scale + _offsetY);
                return new PointF(sx, sy);
            }
        }

        /// <summary>
        /// 将屏幕坐标（像素）转换为世界坐标。
        /// 公式：wx = (sx - offsetX) / scale；wy = (sy - offsetY) / scale。
        /// 常用于鼠标拾取：把点击点从屏幕映射回世界坐标。
        /// </summary>
        internal PointF ScreenToWorld(float sx, float sy)
        {
            lock (_syncRoot)
            {
                float wx = (float)((sx - _offsetX) / _scale);
                float wy = (float)((sy - _offsetY) / _scale);
                return new PointF(wx, wy);
            }
        }
    }
}
