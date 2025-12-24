using System;
using System.Windows.Forms;

namespace GridDemo.WorldView.CenterGrid
{
    internal class ResetCenterStrategy : ICenterStrategy
    {
        private readonly Control _host;

        private readonly Func<double> _getWorldWidthM;
        private readonly Func<double> _getWorldHeightM;

        private readonly Func<double> _getInitialScale;
        private readonly Action<double> _setScale;

        private readonly Action<double> _setOffsetX;
        private readonly Action<double> _setOffsetY;

        private readonly Action<double, float, float> _updateWorldTransform;

        public ResetCenterStrategy(
            Control host,
            Func<double> getWorldWidthM,
            Func<double> getWorldHeightM,
            Func<double> getInitialScale,
            Action<double> setScale,
            Action<double> setOffsetX,
            Action<double> setOffsetY,
            Action<double, float, float> updateWorldTransform)
        {
            _host = host ?? throw new ArgumentNullException(nameof(host));
            _getWorldWidthM = getWorldWidthM ?? throw new ArgumentNullException(nameof(getWorldWidthM));
            _getWorldHeightM = getWorldHeightM ?? throw new ArgumentNullException(nameof(getWorldHeightM));
            _getInitialScale = getInitialScale ?? throw new ArgumentNullException(nameof(getInitialScale));
            _setScale = setScale ?? throw new ArgumentNullException(nameof(setScale));
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

            double initScale = _getInitialScale();
            _setScale(initScale);

            double gridPixelWidth = worldWidth * initScale;
            double gridPixelHeight = worldHeight * initScale;

            double newOffsetX = (clientWidth - gridPixelWidth) / 2.0;
            double newOffsetY = (clientHeight - gridPixelHeight) / 2.0;

            _setOffsetX(newOffsetX);
            _setOffsetY(newOffsetY);

            _updateWorldTransform(initScale, (float)newOffsetX, (float)newOffsetY);

            _host.Invalidate();
        }
    }
}
