using System;
using System.Drawing;
using System.Threading;
using System.Windows.Forms;

namespace Graphic
{
    public partial class Form1 : Form
    {
        //记录初始的缩放大小和偏移量，方便后续重置
        private double _initialScale;
        private double _initialOffsetX;
        private double _initialOffsetY;

        private const int GridCount = 30;               // 网格数30
        private const double CellSizeM = 0.55;          // 一格代表距离0.55米

        // 网格在世界坐标中的总宽高（米）
        private double _worldWidthM = GridCount * CellSizeM;
        private double _worldHeightM = GridCount * CellSizeM;

        private double _scale = 30.0;                   // 1米 = 30像素

        //世界坐标对应屏幕坐标偏移量
        private double _offsetX;
        private double _offsetY;

        // 拖动(pan)相关
        private bool _isPanning = false;
        private Point _lastMousePos;                    //鼠标当前位置

        //------------------------------机器人相关变量------------------------------//
        //机器人初始世界坐标
        private double _robotX = 0.0;
        private double _robotY = 0.0;

        //机器人的初始速度和加速度
        private double _robotSpeed = 1.5;
        private double _robotAcc = 0;

        //机器人移动方向枚举
        private enum EnumMoveDirection
        {
            Right,
            Left,
            Down,
            Up
        }
        //机器人当前移动方向
        private EnumMoveDirection _moveDirection = EnumMoveDirection.Right;

        private readonly double _dt = 0.02;  //固定时间模拟步长0.02秒

        //自制定时器
        private Thread _workerThread;
        private volatile bool _isRunning = false;   //线程运行标志
        private readonly object _robotLock = new object();
        //------------------------------机器人相关变量------------------------------//

        public Form1()
        {
            InitializeComponent();

            // 启用双缓冲减少闪烁
            this.DoubleBuffered = true;

            // 允许 MouseWheel 事件（有时需要设置）
            this.SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.UserPaint | ControlStyles.OptimizedDoubleBuffer, true);

            // 注册事件（也可在设计器里绑定）
            this.Load += Form1_Load;                    //页面加载
            this.Paint += Form1_Paint;                  //重绘
            this.MouseWheel += Form1_MouseWheel;        //鼠标滚动
            this.MouseDown += Form1_MouseDown;          //鼠标按下
            this.MouseMove += Form1_MouseMove;          //鼠标移动
            this.MouseUp += Form1_MouseUp;              //鼠标弹起
            this.FormClosing += Form1_FormClosing;

            // 启动后台线程，用线程＋sleep实现定时器
            _isRunning = true;
            _workerThread = new Thread(_Thread_Loop);
            _workerThread.IsBackground = true;
            _workerThread.Start();
        }

        //窗体关闭时，安全停止线程
        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            _isRunning = false;
            if (_workerThread != null && _workerThread.IsAlive)
            {
                // 等一会儿退出
                _workerThread.Join(200);
            }
        }

        /// <summary>
        /// 线程主循环，相当于逻辑帧定时器
        /// </summary>
        private void _Thread_Loop()
        {
            while (_isRunning)
            {
                _Thread_UpdateRobot();

                // 通知 UI 线程重绘
                try
                {
                    if (!this.IsDisposed)
                    {
                        this.BeginInvoke(new Action(() =>
                        {
                            this.Invalidate();
                        }));
                    }
                }
                catch
                {
                    // 窗口关闭时可能抛异常，简单吞掉
                }

                // Thread.Sleep(_dt * 1000) 相当于一个简单“定时器”：每 20ms 触发一次逻辑更新
                int sleepMs = (int)(_dt * 1000);
                if (sleepMs < 1) sleepMs = 1;
                Thread.Sleep(sleepMs);
            }
        }

        /// <summary>
        /// 机器人状态更新
        /// </summary>
        /// <param name="dt"></param>
        private void _Thread_UpdateRobot()
        {
            lock (_robotLock)
            {
                // 速度更新
                _robotSpeed += _robotAcc * _dt;

                if (_robotSpeed < 0)
                {
                    _robotSpeed = 0;
                }

                // 根据方向，用 robotSpeed 更新位置
                switch (_moveDirection)
                {
                    case EnumMoveDirection.Right:
                        _robotX += _robotSpeed * _dt;
                        break;

                    case EnumMoveDirection.Left:
                        _robotX -= _robotSpeed * _dt;
                        break;

                    case EnumMoveDirection.Down:
                        _robotY += _robotSpeed * _dt;
                        break;

                    case EnumMoveDirection.Up:
                        _robotY -= _robotSpeed * _dt;
                        break;
                }

                // 根据位置决定是否转向（右→下→左→上→右...）
                switch (_moveDirection)
                {
                    case EnumMoveDirection.Right:
                        if (_robotX >= _worldWidthM)
                        {
                            _robotX = _worldWidthM;
                            _moveDirection = EnumMoveDirection.Down;
                        }
                        break;

                    case EnumMoveDirection.Down:
                        if (_robotY >= _worldHeightM)
                        {
                            _robotY = _worldHeightM;
                            _moveDirection = EnumMoveDirection.Left;
                        }
                        break;

                    case EnumMoveDirection.Left:
                        if (_robotX <= 0.0)
                        {
                            _robotX = 0.0;
                            _moveDirection = EnumMoveDirection.Up;
                        }
                        break;

                    case EnumMoveDirection.Up:
                        if (_robotY <= 0.0)
                        {
                            _robotY = 0.0;
                            _moveDirection = EnumMoveDirection.Right;
                        }
                        break;
                }
            }
        }

        /// <summary>
        /// 窗体加载，确保每次运行网格处于屏幕中心
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Form1_Load(object sender, EventArgs e)
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

        /// <summary>
        /// GDI+绘图核心：Paint事件
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Form1_Paint(object sender, PaintEventArgs e)
        {
            Graphics g = e.Graphics;
            //抗锯齿
            g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;

            //清屏
            g.Clear(Color.White);

            //绘制网格（世界坐标 → 屏幕坐标）
            using (var thinPen = new Pen(Color.LightGray, 1f))
            using (var thickPen = new Pen(Color.Gray, 1.5f))
            {
                // 画竖线
                for (int i = 0; i <= GridCount; i++)
                {
                    double xWorld = i * CellSizeM;
                    PointF p1 = WorldToScreen(xWorld, 0);
                    PointF p2 = WorldToScreen(xWorld, _worldHeightM);

                    Pen pen = (i % 5 == 0) ? thickPen : thinPen;            //区分每五格线
                    g.DrawLine(pen, p1, p2);
                }

                // 画横线
                for (int j = 0; j <= GridCount; j++)
                {
                    double yWorld = j * CellSizeM;
                    PointF p1 = WorldToScreen(0, yWorld);
                    PointF p2 = WorldToScreen(_worldWidthM, yWorld);

                    Pen pen = (j % 5 == 0) ? thickPen : thinPen;
                    g.DrawLine(pen, p1, p2);
                }
            }

            //画机器人
            DrawRobot(g);

            //在左上角显示当前数据信息
            using (var font = new Font("宋体", 10))
            using (var brush = new SolidBrush(Color.Black))
            {
                double robotX, robotY, robotV, robotA;
                lock (_robotLock)
                {
                    robotX = _robotX;
                    robotY = _robotY;
                    robotV = _robotSpeed;
                    robotA = _robotAcc;
                }

                string info = $"Scale: {_scale:F1} px/m   Offset: ({_offsetX:F0}, {_offsetY:F0})";
                string infoRobot = $"Robot: x={robotX:F2}m, y={robotY:F2}m, v={robotV:F2}m/s, a={robotA:F2}m/s2";
                g.DrawString(info, font, brush, new PointF(10, 10));
                g.DrawString(infoRobot, font, brush, new PointF(10, 25));
            }
        }

        /// <summary>
        /// 画机器人（世界坐标 → 屏幕坐标），一个实心的圆形
        /// </summary>
        /// <param name="g"></param>
        private void DrawRobot(Graphics g)
        {
            double robotX, robotY;
            lock (_robotLock)
            {
                robotX = _robotX;
                robotY = _robotY;
            }

            PointF screenPos = WorldToScreen(robotX, robotY);

            // 为保证机器人清晰，将像素设置为缩放大小的四分之一
            float radiusPx = (float)(_scale / 4);

            RectangleF rect = new RectangleF(
                screenPos.X - radiusPx,
                screenPos.Y - radiusPx,
                radiusPx * 2,
                radiusPx * 2);

            using (var brush = new SolidBrush(Color.Red))
            using (var pen = new Pen(Color.Black, 1.5f))
            {
                g.FillEllipse(brush, rect);
                g.DrawEllipse(pen, rect);
            }
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

        private void btnReset_Click(object sender, EventArgs e)
        {
            // 将当前缩放和偏移恢复到初始值
            _scale = _initialScale;
            _offsetX = _initialOffsetX;
            _offsetY = _initialOffsetY;

            //将机器人恢复到初始状态
            lock (_robotLock)
            {
                _robotX = 0.0;
                _robotY = 0.0;

                _robotSpeed = 1.5;
                _robotAcc = 0.0;

                _moveDirection = EnumMoveDirection.Right;
            }

            //将调节框恢复到初始状态
            numericAcc.Value = 0.0M;
            numericVinit.Value = 1.5M;

            this.Invalidate(); // 触发重绘
        }

        private void numericAcc_ValueChanged(object sender, EventArgs e)
        {
            lock (_robotLock)
            {
                _robotAcc = (double)((NumericUpDown)sender).Value;
            }
        }

        private void numericVinit_ValueChanged(object sender, EventArgs e)
        {
            lock (_robotLock)
            {
                _robotSpeed = (double)((NumericUpDown)sender).Value;
            }
        }
    }
}
