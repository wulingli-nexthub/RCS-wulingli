using System.Drawing;

namespace Graphic.WorldView
{
    internal class WorldTransform
    {
        private readonly object _syncRoot = new object();

        private double _scale;
        private float _offsetX;
        private float _offsetY;

        internal WorldTransform(double scale, float offsetX, float offsetY)
        {
            _scale = scale;
            _offsetX = offsetX;
            _offsetY = offsetY;
        }

        internal void Update(double scale, float offsetX, float offsetY)
        {
            lock (_syncRoot)
            {
                _scale = scale;
                _offsetX = offsetX;
                _offsetY = offsetY;
            }
        }

        internal PointF WorldToScreen(double wx, double wy)
        {
            lock (_syncRoot)
            {
                float sx = (float)(wx * _scale + _offsetX);
                float sy = (float)(wy * _scale + _offsetY);
                return new PointF(sx, sy);
            }
        }

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
