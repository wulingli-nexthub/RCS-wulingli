using System;
using System.Windows.Forms;

namespace Graphic.WorldView.CenterGrid
{
    internal class LoadCenterStrategy : ICenterStrategy
    {
        private readonly Control _host;

        private readonly Func<double> _getWorldWidthM;
        private readonly Func<double> _getWorldHeightM;

        private readonly Func<double> _getScale;
        private readonly Action<double> _setScale;

        private readonly Action<double> _setOffsetX;
        private readonly Action<double> _setOffsetY;

        private readonly Action<double> _setInitialScale;
        private readonly Action<double> _setInitialOffsetX;
        private readonly Action<double> _setInitialOffsetY;

        private readonly Action<double, float, float> _updateWorldTransform;

        public LoadCenterStrategy(
            Control host,
            Func<double> getWorldWidthM,
            Func<double> getWorldHeightM,
            Func<double> getScale,
            Action<double> setScale,
            Action<double> setOffsetX,
            Action<double> setOffsetY,
            Action<double> setInitialScale,
            Action<double> setInitialOffsetX,
            Action<double> setInitialOffsetY,
            Action<double, float, float> updateWorldTransform)
        {
            _host = host ?? throw new ArgumentNullException(nameof(host));
            _getWorldWidthM = getWorldWidthM ?? throw new ArgumentNullException(nameof(getWorldWidthM));
            _getWorldHeightM = getWorldHeightM ?? throw new ArgumentNullException(nameof(getWorldHeightM));
            _getScale = getScale ?? throw new ArgumentNullException(nameof(getScale));
            _setScale = setScale ?? throw new ArgumentNullException(nameof(setScale));
            _setOffsetX = setOffsetX ?? throw new ArgumentNullException(nameof(setOffsetX));
            _setOffsetY = setOffsetY ?? throw new ArgumentNullException(nameof(setOffsetY));
            _setInitialScale = setInitialScale ?? throw new ArgumentNullException(nameof(setInitialScale));
            _setInitialOffsetX = setInitialOffsetX ?? throw new ArgumentNullException(nameof(setInitialOffsetX));
            _setInitialOffsetY = setInitialOffsetY ?? throw new ArgumentNullException(nameof(setInitialOffsetY));
            _updateWorldTransform = updateWorldTransform ?? throw new ArgumentNullException(nameof(updateWorldTransform));
        }

        public void CenterGrid()
        {
            int clientWidth = _host.ClientSize.Width;
            int clientHeight = _host.ClientSize.Height;

            if (clientWidth <= 0 || clientHeight <= 0)
                return;

            double worldWidth = _getWorldWidthM();
            double worldHeight = _getWorldHeightM();

            // 边距，避免网格顶到边缘
            double margin = 40;
            double scaleX = (clientWidth - margin * 2) / worldWidth;
            double scaleY = (clientHeight - margin * 2) / worldHeight;
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

            // 记录初始状态
            _setInitialScale(newScale);
            _setInitialOffsetX(newOffsetX);
            _setInitialOffsetY(newOffsetY);

            _updateWorldTransform(newScale, (float)newOffsetX, (float)newOffsetY);

            _host.Invalidate();
        }
    }
}
