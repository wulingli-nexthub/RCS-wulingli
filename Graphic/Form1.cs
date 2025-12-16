using Graphic.Draws;
using Graphic.WorldView;
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
        private double _robotSpeed = 0;
        private double _robotMaxSpeed = 1.5;
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

        private WorldTransform _worldTransform;
        private DrawGrid _drawGrid;
        private DrawRobot _drawRobot;

        private void Form1_Paint(object sender, PaintEventArgs e)
        {
            Graphics g = e.Graphics;

            // 由 DrawGrid 绘制网格（内部已经做了抗锯齿和清屏）
            _drawGrid.Grid(g);

            // 由 DrawRobot 绘制机器人
            _drawRobot.Robot(g);

            // 在左上角显示当前数据信息（这一段沿用你原来的逻辑）
            using (var font = new Font("宋体", 10))
            using (var brush = new SolidBrush(Color.Black))
            {
                double robotX;
                double robotY;
                double robotV;
                double robotA;

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
        // 替换 InitializeDrawing 方法中的 DrawRobot 构造函数调用，修正 lambda 返回类型
        private void InitializeDrawing()
        {
            _worldTransform = new WorldTransform(_scale, (float)_offsetX, (float)_offsetY);
            _drawGrid = new DrawGrid(_worldTransform, _worldWidthM, _worldHeightM);
            _drawRobot = new DrawRobot(
                _worldTransform,
                _robotLock,
                () => (_robotX, _robotY), // 保持此行
                () => _scale              // 新增此行，传递获取 scale 的委托
            );
        }

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



        private void numericAcc_ValueChanged(object sender, EventArgs e)
        {
            lock (_robotLock)
            {
                _robotAcc = (double)((NumericUpDown)sender).Value;
            }
        }



        private void numericVmax_ValueChanged(object sender, EventArgs e)
        {
            lock (_robotLock)
            {
                _robotMaxSpeed = (double)((NumericUpDown)sender).Value;
            }
        }
    }
}
