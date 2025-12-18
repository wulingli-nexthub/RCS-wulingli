using System;
using System.Windows.Forms;

namespace Graphic.WorldView.CenterGrid
{
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

        public void CenterGrid()
        {
            int clientWidth = _host.ClientSize.Width;
            int clientHeight = _host.ClientSize.Height;

            if (clientWidth <= 0 || clientHeight <= 0)
                return;

            double worldWidth = _getWorldWidthM();
            double worldHeight = _getWorldHeightM();
            double scale = _getScale();

            double gridPixelWidth = worldWidth * scale;
            double gridPixelHeight = worldHeight * scale;

            double newOffsetX = (clientWidth - gridPixelWidth) / 2.0;
            double newOffsetY = (clientHeight - gridPixelHeight) / 2.0;

            _setOffsetX(newOffsetX);
            _setOffsetY(newOffsetY);

            _updateWorldTransform(scale, (float)newOffsetX, (float)newOffsetY);

            _host.Invalidate();
        }
    }
}
