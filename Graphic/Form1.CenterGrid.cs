using System;

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
    }
}
