using System;
using System.Drawing;
using System.Windows.Forms;

namespace GridDemo.Events
{
    /// <summary>
    /// 鼠标拖拽平移（Pan）控制器：
    /// - 按下鼠标左键开始拖拽；
    /// - 移动鼠标时按位移增量更新视图偏移（offsetX/offsetY）；
    /// - 松开左键结束拖拽，并恢复鼠标指针样式。
    /// 说明：本类不直接持有 offset/scale 的存储，而是通过委托访问/修改外部状态，便于与窗体/视图逻辑解耦。
    /// </summary>
    internal class MousePan
    {
        private readonly Form _form;

        private bool _isPanning;        // 当前是否正在拖拽
        private Point _lastMousePos;         // 上一次鼠标位置（屏幕坐标）

        // 外部传入的状态访问 / 修改接口
        private readonly Func<(double offsetX, double offsetY, double scale)> _getState;
        private readonly Action<double, double> _setOffset;
        private readonly Action<double, float, float> _updateWorldTransform;

        /// <summary>
        /// 创建鼠标平移控制器实例。
        /// </summary>
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

        /// <summary>
        /// 鼠标按下：左键进入拖拽模式。
        /// 记录按下时的位置，并将鼠标指针切换为 Hand，提示正在平移视图。
        /// </summary>
        public void MouseDown(MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Left)
            {
                _isPanning = true;
                _lastMousePos = e.Location;
                _form.Cursor = Cursors.Hand;
            }
        }

        /// <summary>
        /// 鼠标移动：若处于拖拽中，则根据鼠标位移增量更新偏移量。
        /// dx/dy 是像素增量：
        /// - dx &gt; 0 表示鼠标向右移动，视图偏移 X 增大（画面整体向右“跟着走”）
        /// - dy &gt; 0 表示鼠标向下移动，视图偏移 Y 增大
        /// </summary>
        public void MouseMove(MouseEventArgs e)
        {
            if (!_isPanning)
                return;

            int dx = e.X - _lastMousePos.X;         // 计算位移增量
            int dy = e.Y - _lastMousePos.Y;
            _lastMousePos = e.Location;             // 更新上次位置

            var (offsetX, offsetY, scale) = _getState();

            double newOffsetX = offsetX + dx;            // 更新偏移量
            double newOffsetY = offsetY + dy;

            _setOffset(newOffsetX, newOffsetY);            // 应用新的偏移量

            // 同步到 WorldTransform
            _updateWorldTransform(scale, (float)newOffsetX, (float)newOffsetY);
        }

        /// <summary>
        /// /// 鼠标松开：左键退出拖拽模式并恢复默认指针样式。
        /// </summary>
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
