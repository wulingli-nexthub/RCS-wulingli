using System;
using System.Windows.Forms;

namespace Graphic
{
    public partial class Form1
    {
        // 窗体加载时，居中显示网格
        private void Form1_Load(object sender, EventArgs e)
        {
            // 先简单把世界(0,0)放在窗口中心，避免初始全白
            _offsetX = this.ClientSize.Width / 2.0;
            _offsetY = this.ClientSize.Height / 2.0;

            // 记录初始值
            _initialScale = _scale;
            _initialOffsetX = _offsetX;
            _initialOffsetY = _offsetY;

            // 初始化
            Initialize();

            // 让网格真正居中铺满窗口
            CentreGrid();

            // CentreGrid 会更新 _scale/_offsetX/_offsetY，这里同步一次到 WorldTransform
            _worldTransform.Update(_scale, (float)_offsetX, (float)_offsetY);
        }

        /// <summary>
        /// 根据当前窗口大小和世界尺寸，让网格整体居中显示
        /// </summary>
        private void CentreGrid()
        {
            int clientWidth = this.ClientSize.Width;
            int clientHeight = this.ClientSize.Height;

            if (clientWidth <= 0 || clientHeight <= 0)
            {
                return;
            }

            // 边距，避免网格顶到边缘
            double margin = 40;
            double scaleX = (clientWidth - margin * 2) / _worldWidthM;
            double scaleY = (clientHeight - margin * 2) / _worldHeightM;
            double newScale = Math.Min(scaleX, scaleY);

            // 当前缩放下，网格在屏幕上的像素尺寸
            double gridPixelWidth = _worldWidthM * newScale;
            double gridPixelHeight = _worldHeightM * newScale;

            // 让左上角偏移重新计算成居中
            double newOffsetX = (clientWidth - gridPixelWidth) / 2.0;
            double newOffsetY = (clientHeight - gridPixelHeight) / 2.0;

            _scale = newScale;
            _offsetX = newOffsetX;
            _offsetY = newOffsetY;

            // 保存为初始状态，供“重置”按钮使用
            _initialScale = _scale;
            _initialOffsetX = _offsetX;
            _initialOffsetY = _offsetY;

            // 同步到 WorldTransform
            _worldTransform.Update(_scale, (float)_offsetX, (float)_offsetY);

            Invalidate();   // 重绘
        }

        /// <summary>
        /// 鼠标滚轮缩放（以鼠标所在点为缩放中心）
        /// </summary>
        private void Form1_MouseWheel(object sender, MouseEventArgs e)
        {
            // 把 screen->world 的逻辑通过委托传给 handler
            _mouseWheel.Wheel(
                e,
                screen =>
                {
                    // 使用现有的 WorldTransform 做坐标转换
                    return _worldTransform.ScreenToWorld(screen.X, screen.Y);
                });

            Invalidate(); // 触发重绘
        }

        // 鼠标按下：准备拖动
        private void Form1_MouseDown(object sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Left)
            {
                _isPanning = true;
                _lastMousePos = e.Location;
                Cursor = Cursors.Hand;
            }
        }

        // 鼠标移动：拖动画布
        private void Form1_MouseMove(object sender, MouseEventArgs e)
        {
            if (_isPanning)
            {
                int dx = e.X - _lastMousePos.X;
                int dy = e.Y - _lastMousePos.Y;
                _lastMousePos = e.Location;

                _offsetX += dx;
                _offsetY += dy;

                // 同步到 WorldTransform
                _worldTransform.Update(_scale, (float)_offsetX, (float)_offsetY);

                Invalidate(); // 重绘
            }
        }

        // 鼠标松开：结束拖动
        private void Form1_MouseUp(object sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Left)
            {
                _isPanning = false;
                Cursor = Cursors.Default;
            }
        }

        private void Form1_Resize(object sender, EventArgs e)
        {
            int clientWidth = ClientSize.Width;
            int clientHeight = ClientSize.Height;

            if (clientWidth <= 0 || clientHeight <= 0)
            {
                return;
            }

            // 当前缩放下，整个世界网格区域在屏幕上的像素宽高
            double gridPixelWidth = _worldWidthM * _scale;
            double gridPixelHeight = _worldHeightM * _scale;

            // 让网格在新的窗口大小下仍然居中
            _offsetX = (clientWidth - gridPixelWidth) / 2.0;
            _offsetY = (clientHeight - gridPixelHeight) / 2.0;

            // 同步到 WorldTransform
            _worldTransform.Update(_scale, (float)_offsetX, (float)_offsetY);

            Invalidate();
        }

        private void btnReset_Click(object sender, EventArgs e)
        {
            int clientWidth = ClientSize.Width;
            int clientHeight = ClientSize.Height;

            if (clientWidth <= 0 || clientHeight <= 0)
            {
                return;
            }

            // 恢复初始缩放
            _scale = _initialScale;

            // 当前缩放下，整个世界网格区域在屏幕上的像素宽高
            double gridPixelWidth = _worldWidthM * _scale;
            double gridPixelHeight = _worldHeightM * _scale;

            // 让左上角偏移重新计算成居中
            _offsetX = (clientWidth - gridPixelWidth) / 2.0;
            _offsetY = (clientHeight - gridPixelHeight) / 2.0;

            // 同步到 WorldTransform
            _worldTransform.Update(_scale, (float)_offsetX, (float)_offsetY);

            Invalidate(); // 触发重绘
        }
    }
}