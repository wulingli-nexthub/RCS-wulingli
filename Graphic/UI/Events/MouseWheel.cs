using System;
using System.Drawing;
using System.Windows.Forms;

namespace GridDemo.Events
{
    /// <summary>
    /// 鼠标滚轮缩放控制器：
    /// - 读取当前视图状态（scale/offset）；
    /// - 根据滚轮方向计算新的缩放比例；
    /// - 调整偏移量 offsetX/offsetY，使“鼠标所在的世界点”在缩放前后映射到同一屏幕位置（以鼠标为缩放中心）。
    /// 说明：本类不直接持有 scale/offset 的存储，而是通过委托与 Form 的字段同步，便于解耦与复用。
    /// </summary>
    internal class MouseWheel
    {
        private readonly Form _form;

        // 通过引用方式操作这些字段，方便与 Form1 内原有字段同步
        private readonly Action<double, double> _setOffset;  // 设置 _offsetX / _offsetY
        private readonly Func<(double scale, double offsetX, double offsetY)> _getState;
        private readonly Action<double> _setScale;           // 设置 _scale
        private readonly Action<double, float, float> _updateWorldTransform; // _worldTransform.Update

        /// <summary>
        /// 注入外部状态读写与 WorldTransform 更新接口，并做空值检查。
        /// </summary>
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

        /// <summary>
        /// 处理滚轮事件，实现“以鼠标点为中心”的缩放。
        /// 关键公式：
        /// 屏幕坐标与世界坐标关系（以 X 为例）：screenX = worldX * scale + offsetX
        /// 因此在已知 mouseScreenX 和 mouseWorldX 的情况下，可反求新的 offsetX：
        /// offsetX = mouseScreenX - mouseWorldX * newScale
        /// 这样缩放后 mouseWorldX 仍对应到同一 mouseScreenX，避免视图“飘移”。
        /// </summary>
        public void Wheel(MouseEventArgs e, Func<PointF, PointF> screenToWorld)
        {
            if (e == null)
                return;
            if (screenToWorld == null)
                return;

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

            _setScale(newScale);    // 更新缩放比例

            // 3. 根据缩放调整 offset，使缩放后鼠标位置不“飘”
            double newOffsetX = mouseScreen.X - mouseWorldBefore.X * newScale;
            double newOffsetY = mouseScreen.Y - mouseWorldBefore.Y * newScale;
            _setOffset(newOffsetX, newOffsetY);

            // 4. 同步到 WorldTransform
            _updateWorldTransform(newScale, (float)newOffsetX, (float)newOffsetY);
        }
    }
}
