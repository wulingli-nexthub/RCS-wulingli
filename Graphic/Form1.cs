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
        private double _robotX = CellSizeM / 2;
        private double _robotY = CellSizeM / 2;

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
            this.Resize += Form1_Resize;

            // 启动后台线程，用线程＋sleep实现定时器
            _isRunning = true;
            _workerThread = new Thread(_Thread_Loop);
            _workerThread.IsBackground = true;
            _workerThread.Start();
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

            //将机器人恢复到初始状态
            lock (_robotLock)
            {
                _robotX = CellSizeM / 2;
                _robotY = CellSizeM / 2;

                _robotSpeed = 1.5;
                _robotAcc = 0.0;

                _moveDirection = EnumMoveDirection.Right;
            }

            //将调节框恢复到初始状态
            numericAcc.Value = 0.0M;
            numericVinit.Value = 1.5M;


            this.Invalidate(); // 触发重绘
        }
    }
}
