using System;
using System.Drawing;
using System.Windows.Forms;

namespace Graphic.Events
{
    internal class MousePan
    {
        private readonly Form _form;

        // 当前是否正在拖拽
        private bool _isPanning;
        private Point _lastMousePos;

        // 外部传入的状态访问 / 修改接口
        private readonly Func<(double offsetX, double offsetY, double scale)> _getState;
        private readonly Action<double, double> _setOffset;
        private readonly Action<double, float, float> _updateWorldTransform;

        public MousePan(
            Form form,
            Func<(double offsetX, double offsetY, double scale)> getState,
            Action<double, double> setOffset,
            Action<double, float, float> updateWorldTransform)
        {
            _form = form ?? throw new ArgumentNullException(nameof(form));
            _getState = getState ?? throw new ArgumentNullException(nameof(getState));
            _setOffset = setOffset ?? throw new ArgumentNullException(nameof(setOffset));
            _updateWorldTransform = updateWorldTransform ?? throw new ArgumentNullException(nameof(updateWorldTransform));
        }

        public void MouseDown(MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Left)
            {
                _isPanning = true;
                _lastMousePos = e.Location;
                _form.Cursor = Cursors.Hand;
            }
        }

        public void MouseMove(MouseEventArgs e)
        {
            if (!_isPanning) return;

            int dx = e.X - _lastMousePos.X;
            int dy = e.Y - _lastMousePos.Y;
            _lastMousePos = e.Location;

            var (offsetX, offsetY, scale) = _getState();

            double newOffsetX = offsetX + dx;
            double newOffsetY = offsetY + dy;

            _setOffset(newOffsetX, newOffsetY);

            // 同步到 WorldTransform
            _updateWorldTransform(scale, (float)newOffsetX, (float)newOffsetY);
        }

        public void MouseUp(MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Left)
            {
                _isPanning = false;
                _form.Cursor = Cursors.Default;
            }
        }
    }
}
