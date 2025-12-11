using System;
using System.Drawing;
using System.Windows.Forms;

namespace Graphic
{
    public partial class Form1 : Form
    {
        private double _initialScale;
        private double _initialOffsetX;
        private double _initialOffsetY;

        private const int GridCount = 30;               // 网格数30
        private const double CellSizeM = 0.55;          // 一格代表距离0.55米

        private double _scale = 30.0;                   // 1米 = 30像素
        private double _offsetX;                  //将(0,0)放在屏幕的(100,100)
        private double _offsetY;

        // 拖动(pan)相关
        private bool _isPanning = false;
        private Point _lastMousePos;                    //鼠标当前位置

        public Form1()
        {
            InitializeComponent();

            // 启用双缓冲减少闪烁
            this.DoubleBuffered = true;

            // 允许 MouseWheel 事件（有时需要设置）
            this.SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.UserPaint | ControlStyles.OptimizedDoubleBuffer, true);

            // 注册事件（也可在设计器里绑定）
            this.Load += Form1_Load;
            this.Paint += Form1_Paint;                  //重绘
            this.MouseWheel += Form1_MouseWheel;        //鼠标滚动
            this.MouseDown += Form1_MouseDown;          //鼠标按下
            this.MouseMove += Form1_MouseMove;          //鼠标移动
            this.MouseUp += Form1_MouseUp;              //鼠标弹起
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            // 网格在世界坐标中的总宽高（米）
            double worldWidthM = GridCount * CellSizeM;
            double worldHeightM = GridCount * CellSizeM;

            // 可用的屏幕大小（像素），这里直接用 ClientSize
            int clientWidth = this.ClientSize.Width;
            int clientHeight = this.ClientSize.Height;

            if (clientWidth <= 0 || clientHeight <= 0)
                return;

            // 计算一个合适的缩放，使网格能完整出现在窗口中，并稍微留一点边距
            double margin = 40;  // 像素边距
            double scaleX = (clientWidth - margin * 2) / worldWidthM;
            double scaleY = (clientHeight - margin * 2) / worldHeightM;
            double newScale = Math.Min(scaleX, scaleY);

            // 根据缩放计算网格在屏幕上的像素尺寸
            double gridPixelWidth = worldWidthM * newScale;
            double gridPixelHeight = worldHeightM * newScale;

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

        // 世界坐标(米) -> 屏幕坐标(像素)
        private PointF WorldToScreen(double wx, double wy)
        {
            float sx = (float)(wx * _scale + _offsetX);
            float sy = (float)(wy * _scale + _offsetY);
            return new PointF(sx, sy);
        }

        // 屏幕 -> 世界
        private PointF ScreenToWorld(float sx, float sy)
        {
            float wx = (float)((sx - _offsetX) / _scale);
            float wy = (float)((sy - _offsetY) / _scale);
            return new PointF(wx, wy);
        }

        // 负责绘图
        private void Form1_Paint(object sender, PaintEventArgs e)
        {
            Graphics g = e.Graphics;
            g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;

            // 背景填充（也可依赖窗体 BackColor）
            g.Clear(Color.White);

            // 网格总物理尺寸
            double worldWidthM = GridCount * CellSizeM;
            double worldHeightM = GridCount * CellSizeM;

            using (var thinPen = new Pen(Color.LightGray, 1f))
            using (var thickPen = new Pen(Color.Gray, 1.5f))
            {
                // 画竖线
                for (int i = 0; i <= GridCount; i++)
                {
                    double xWorld = i * CellSizeM;
                    PointF p1 = WorldToScreen(xWorld, 0);
                    PointF p2 = WorldToScreen(xWorld, worldHeightM);

                    Pen pen = (i % 5 == 0) ? thickPen : thinPen;            //区分每五格线
                    g.DrawLine(pen, p1, p2);
                }

                // 画横线
                for (int j = 0; j <= GridCount; j++)
                {
                    double yWorld = j * CellSizeM;
                    PointF p1 = WorldToScreen(0, yWorld);
                    PointF p2 = WorldToScreen(worldWidthM, yWorld);

                    Pen pen = (j % 5 == 0) ? thickPen : thinPen;
                    g.DrawLine(pen, p1, p2);
                }
            }

            // 可选：在左上角显示当前缩放信息
            using (var font = new Font("Consolas", 10))
            using (var brush = new SolidBrush(Color.Black))
            {
                string info = $"Scale: {_scale:F1} px/m   Offset: ({_offsetX:F0}, {_offsetY:F0})";
                g.DrawString(info, font, brush, new PointF(10, 10));
            }
        }

        // 鼠标滚轮：缩放
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

        private void btnReset_Click(object sender, EventArgs e)
        {
            // 将当前缩放和偏移恢复到初始值
            _scale = _initialScale;
            _offsetX = _initialOffsetX;
            _offsetY = _initialOffsetY;

            this.Invalidate(); // 触发重绘
        }
    }
}
