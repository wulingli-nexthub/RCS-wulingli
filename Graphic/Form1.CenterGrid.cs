using System;
using System.Drawing;
using System.Windows.Forms;

namespace Graphic
{
    public partial class Form1
    {
        // 窗体加载时，居中显示网格
        private void Form1_Load(object sender, EventArgs e)
        {
            CentreGrid();
        }

        private void CentreGrid()
        {
            // 可用的屏幕大小（像素），这里直接用 ClientSize
            int clientWidth = this.ClientSize.Width;
            int clientHeight = this.ClientSize.Height;

            if (clientWidth <= 0 || clientHeight <= 0)
                return;

            // 计算一个合适的缩放，使网格能完整出现在窗口中，并稍微留一点边距
            double margin = 40;  // 像素边距
            double scaleX = (clientWidth - margin * 2) / _worldWidthM;
            double scaleY = (clientHeight - margin * 2) / _worldHeightM;
            double newScale = Math.Min(scaleX, scaleY);

            // 根据缩放计算网格在屏幕上的像素尺寸
            double gridPixelWidth = _worldWidthM * newScale;
            double gridPixelHeight = _worldHeightM * newScale;

            // 让网格矩形居中：offset 决定“世界(0,0)”映射到哪里
            double newOffsetX = (clientWidth - gridPixelWidth) / 2.0;
            double newOffsetY = (clientHeight - gridPixelHeight) / 2.0;

            // 更新当前值
            _scale = newScale;
            _offsetX = newOffsetX;
            _offsetY = newOffsetY;

            // 保存为初始状态，供“重置”按钮使用
            _initialScale = _scale;
            _initialOffsetX = _offsetX;
            _initialOffsetY = _offsetY;

            this.Invalidate();   // 重绘
        }

        /// <summary>
        /// 鼠标滚轮缩放
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Form1_MouseWheel(object sender, MouseEventArgs e)
        {
            // 当前鼠标的屏幕坐标、对应的世界坐标
            var mouseScreen = new PointF(e.X, e.Y);
            var mouseWorldBefore = ScreenToWorld(mouseScreen.X, mouseScreen.Y);

            // 计算新的缩放
            double zoomFactor = (e.Delta > 0) ? 1.1 : 0.9;            //e.Delta为鼠标滚动，向上大于0
            double newScale = _scale * zoomFactor;                    //缩放后记得修改像素

            // 限制缩放范围
            if (newScale < 5) newScale = 5;       // 最小
            if (newScale > 200) newScale = 200;   // 最大

            // 调整 offset 使缩放中心为鼠标所在点
            _scale = newScale;
            // 使世界坐标 mouseWorldBefore 仍然映射到原来的 mouseScreen
            _offsetX = mouseScreen.X - mouseWorldBefore.X * _scale;
            _offsetY = mouseScreen.Y - mouseWorldBefore.Y * _scale;

            this.Invalidate(); // 触发重绘
        }

        // 鼠标按下：准备拖动
        private void Form1_MouseDown(object sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Left)
            {
                _isPanning = true;
                _lastMousePos = e.Location;         //获取当前鼠标位置
                this.Cursor = Cursors.Hand;
            }
        }

        // 鼠标移动：拖动画布
        private void Form1_MouseMove(object sender, MouseEventArgs e)
        {
            if (_isPanning)
            {
                int dx = e.X - _lastMousePos.X;          //计算位移
                int dy = e.Y - _lastMousePos.Y;
                _lastMousePos = e.Location;

                _offsetX += dx;                          //拖动后修改起点坐标
                _offsetY += dy;

                this.Invalidate(); // 重绘
            }
        }

        // 鼠标松开：结束拖动
        private void Form1_MouseUp(object sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Left)
            {
                _isPanning = false;
                this.Cursor = Cursors.Default;
            }
        }

        private void Form1_Resize(object sender, EventArgs e)
        {
            int clientWidth = this.ClientSize.Width;
            int clientHeight = this.ClientSize.Height;

            if (clientWidth <= 0 || clientHeight <= 0)
                return;

            // 当前缩放下，整个世界网格区域在屏幕上的像素宽高
            double gridPixelWidth = _worldWidthM * _scale;
            double gridPixelHeight = _worldHeightM * _scale;

            // 让左上角偏移重新计算成居中
            _offsetX = (clientWidth - gridPixelWidth) / 2.0;
            _offsetY = (clientHeight - gridPixelHeight) / 2.0;

            this.Invalidate();
        }

        private void btnReset_Click(object sender, EventArgs e)
        {
            // 将当前缩放和偏移恢复到初始值
            _scale = _initialScale;

            int clientWidth = this.ClientSize.Width;
            int clientHeight = this.ClientSize.Height;

            if (clientWidth <= 0 || clientHeight <= 0)
                return;

            // 当前缩放下，整个世界网格区域在屏幕上的像素宽高
            double gridPixelWidth = _worldWidthM * _scale;
            double gridPixelHeight = _worldHeightM * _scale;

            // 让左上角偏移重新计算成居中
            _offsetX = (clientWidth - gridPixelWidth) / 2.0;
            _offsetY = (clientHeight - gridPixelHeight) / 2.0;

            this.Invalidate(); // 触发重绘
        }
    }
}
