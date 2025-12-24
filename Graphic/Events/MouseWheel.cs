using System;
using System.Drawing;
using System.Windows.Forms;

namespace GridDemo.Events
{
    internal class MouseWheel
    {
        private readonly Form _form;

        // 通过引用方式操作这些字段，方便与 Form1 内原有字段同步
        private readonly Action<double, double> _setOffset;  // 设置 _offsetX / _offsetY
        private readonly Func<(double scale, double offsetX, double offsetY)> _getState;
        private readonly Action<double> _setScale;           // 设置 _scale
        private readonly Action<double, float, float> _updateWorldTransform; // _worldTransform.Update

        /// <summary>
        /// 字段初始化并做空值检查
        /// </summary>
        /// <param name="form"></param>
        /// <param name="getState"></param>
        /// <param name="setScale"></param>
        /// <param name="setOffset"></param>
        /// <param name="updateWorldTransform"></param>
        /// <exception cref="ArgumentNullException"></exception>
        public MouseWheel(
            Form form,
            Func<(double scale, double offsetX, double offsetY)> getState,
            Action<double> setScale,
            Action<double, double> setOffset,
            Action<double, float, float> updateWorldTransform)
        {
            _form = form ?? throw new ArgumentNullException(nameof(form));
            _getState = getState ?? throw new ArgumentNullException(nameof(getState));
            _setScale = setScale ?? throw new ArgumentNullException(nameof(setScale));
            _setOffset = setOffset ?? throw new ArgumentNullException(nameof(setOffset));
            _updateWorldTransform = updateWorldTransform ?? throw new ArgumentNullException(nameof(updateWorldTransform));
        }

        public void Wheel(MouseEventArgs e, Func<PointF, PointF> screenToWorld)
        {
            if (e == null) return;
            if (screenToWorld == null) return;

            // 1. 获取当前状态
            var (scale, offsetX, offsetY) = _getState();

            // 当前鼠标的屏幕坐标
            var mouseScreen = new PointF(e.X, e.Y);

            // 使用外部提供的转换函数把屏幕坐标转换为缩放前的世界坐标
            var mouseWorldBefore = screenToWorld(mouseScreen);

            // 2. 计算新的缩放
            double zoomFactor = (e.Delta > 0) ? 1.1 : 0.9;
            double newScale = scale * zoomFactor;

            // 限制缩放范围
            if (newScale < 5)
                newScale = 5;
            if (newScale > 200)
                newScale = 200;

            _setScale(newScale);

            // 3. 根据缩放调整 offset，使缩放后鼠标位置不“飘”
            double newOffsetX = mouseScreen.X - mouseWorldBefore.X * newScale;
            double newOffsetY = mouseScreen.Y - mouseWorldBefore.Y * newScale;
            _setOffset(newOffsetX, newOffsetY);

            // 4. 同步到 WorldTransform
            _updateWorldTransform(newScale, (float)newOffsetX, (float)newOffsetY);
        }
    }
}
