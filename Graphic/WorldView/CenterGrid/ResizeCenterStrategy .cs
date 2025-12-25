using System;
using System.Windows.Forms;

namespace GridDemo.WorldView.CenterGrid
{
    /// <summary>
    /// 尺寸变化时的网格居中策略：
    /// - 在宿主控件尺寸发生变化（Resize）后，根据新的视口尺寸重新计算缩放比例；
    /// - 让整个世界网格尽量完整地显示在视口内，并留出一定边距；
    /// - 同时重新计算 offset，使网格始终居中显示。
    /// </summary>
    internal class ResizeCenterStrategy : ICenterStrategy
    {
        private readonly Control _host;

        private readonly Func<double> _getWorldWidthM;
        private readonly Func<double> _getWorldHeightM;

        private readonly Action<double> _setScale;
        private readonly Action<double> _setOffsetX;
        private readonly Action<double> _setOffsetY;

        private readonly Action<double, float, float> _updateWorldTransform;

        public ResizeCenterStrategy(
            Control host,
            Func<double> getWorldWidthM,
            Func<double> getWorldHeightM,
            Action<double> setScale,
            Action<double> setOffsetX,
            Action<double> setOffsetY,
            Action<double, float, float> updateWorldTransform)
        {
            _host = host ?? throw new ArgumentNullException(nameof(host));
            _getWorldWidthM = getWorldWidthM ?? throw new ArgumentNullException(nameof(getWorldWidthM));
            _getWorldHeightM = getWorldHeightM ?? throw new ArgumentNullException(nameof(getWorldHeightM));
            _setScale = setScale ?? throw new ArgumentNullException(nameof(setScale));
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
            double margin = 40; // 与 LoadCenterStrategy 保持一致：避免网格顶到边缘

            // 分别计算横向/纵向能容纳完整世界的缩放，取较小者保证完全显示。
            // 额外做下限保护，避免窗口非常小时出现负数/0。
            double availableW = Math.Max(1.0, clientWidth - margin * 2);
            double availableH = Math.Max(1.0, clientHeight - margin * 2);

            double scaleX = availableW / worldWidth;
            double scaleY = availableH / worldHeight;
            double newScale = Math.Min(scaleX, scaleY);

            // 当前缩放下，网格在屏幕上的像素尺寸
            double gridPixelWidth = worldWidth * newScale;
            double gridPixelHeight = worldHeight * newScale;

            // 让左上角偏移重新计算成居中
            double newOffsetX = (clientWidth - gridPixelWidth) / 2.0;
            double newOffsetY = (clientHeight - gridPixelHeight) / 2.0;

            _setScale(newScale);
            _setOffsetX(newOffsetX);
            _setOffsetY(newOffsetY);

            _updateWorldTransform(newScale, (float)newOffsetX, (float)newOffsetY);              // 更新 WorldTransform

            _host.Invalidate();            // 触发重绘
        }
    }
}
