using System;
using System.Windows.Forms;

namespace GridDemo.WorldView.CenterGrid
{
    /// <summary>
    /// 尺寸变化时的网格居中策略：在宿主控件尺寸发生变化（Resize）后，保持当前缩放比例不变，
    /// 仅重新计算 offset，使整个世界网格在新的视口尺寸下继续居中显示。
    /// </summary>
    internal class ResizeCenterStrategy : ICenterStrategy
    {
        private readonly Control _host;

        private readonly Func<double> _getWorldWidthM;
        private readonly Func<double> _getWorldHeightM;

        private readonly Func<double> _getScale;
        private readonly Action<double> _setOffsetX;
        private readonly Action<double> _setOffsetY;

        private readonly Action<double, float, float> _updateWorldTransform;

        public ResizeCenterStrategy(
            Control host,
            Func<double> getWorldWidthM,
            Func<double> getWorldHeightM,
            Func<double> getScale,
            Action<double> setOffsetX,
            Action<double> setOffsetY,
            Action<double, float, float> updateWorldTransform)
        {
            _host = host ?? throw new ArgumentNullException(nameof(host));
            _getWorldWidthM = getWorldWidthM ?? throw new ArgumentNullException(nameof(getWorldWidthM));
            _getWorldHeightM = getWorldHeightM ?? throw new ArgumentNullException(nameof(getWorldHeightM));
            _getScale = getScale ?? throw new ArgumentNullException(nameof(getScale));
            _setOffsetX = setOffsetX ?? throw new ArgumentNullException(nameof(setOffsetX));
            _setOffsetY = setOffsetY ?? throw new ArgumentNullException(nameof(setOffsetY));
            _updateWorldTransform = updateWorldTransform ?? throw new ArgumentNullException(nameof(updateWorldTransform));
        }

        /// <summary>
        /// 在控件尺寸变化后执行居中：
        /// - 读取当前视口尺寸；
        /// - 读取世界尺寸与当前 scale；
        /// - 计算世界在屏幕上的像素尺寸；
        /// - 重新计算 offset，使世界居中；
        /// - 更新 WorldTransform 并触发重绘。
        /// </summary>
        public void CenterGrid()
        {
            int clientWidth = _host.ClientSize.Width;
            int clientHeight = _host.ClientSize.Height;

            if (clientWidth <= 0 || clientHeight <= 0)
                return;

            double worldWidth = _getWorldWidthM();
            double worldHeight = _getWorldHeightM();
            double scale = _getScale();               // Resize 不改变缩放：仅保持当前 scale 并重算 offset

            double gridPixelWidth = worldWidth * scale;
            double gridPixelHeight = worldHeight * scale;

            double newOffsetX = (clientWidth - gridPixelWidth) / 2.0;
            double newOffsetY = (clientHeight - gridPixelHeight) / 2.0;

            _setOffsetX(newOffsetX);
            _setOffsetY(newOffsetY);

            _updateWorldTransform(scale, (float)newOffsetX, (float)newOffsetY);              // 更新 WorldTransform

            _host.Invalidate();            // 触发重绘
        }
    }
}
