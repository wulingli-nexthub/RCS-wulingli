using Graphic.Draws;
using Graphic.Events;
using Graphic.RobotRuns;
using Graphic.WorldView;
using Graphic.WorldView.CenterGrid;
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

        private readonly double _dt = 0.02;  //固定时间模拟步长0.02秒

        //自制定时器
        private Thread _workerThread;
        private volatile bool _isRunning = false;   //线程运行标志
        private readonly object _robotLock = new object();
        //------------------------------机器人相关变量------------------------------//

        private WorldTransform _worldTransform;
        private DrawGrid _drawGrid;
        private DrawRobot _drawRobot;
        private MouseWheel _mouseWheel;
        private MousePan _mousePan;
        private CenterGridManager _centerGridManager;
        // 机器人对象：加速度、最大速度、方向
        private Robot _robot;
        private RobotMove _robotMove;
        private RobotSimulator _robotSimulator;

        private void Form1_Paint(object sender, PaintEventArgs e)
        {
            Graphics g = e.Graphics;

            // 由 DrawGrid 绘制网格
            _drawGrid.Draw(g);

            // 由 DrawRobot 绘制机器人
            _drawRobot.Draw(g);

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
                string infoRobot = $"Robot: ({robotX:F2}, {robotY:F2}), v={robotV:F2}m/s, a={robotA:F2}m/s2";
                g.DrawString(info, font, brush, new PointF(10, 10));
                g.DrawString(infoRobot, font, brush, new PointF(10, 25));
            }
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
            this.KeyDown += Form1_KeyDown;
            this.KeyUp += Form1_KeyUp;
            this.KeyPreview = true;                 //确保窗体能接收键盘事件

            // 统一给数值框加 KeyDown 处理
            numericAcc.KeyDown += Numeric_KeyDown_OnEnter;
            numericVinit.KeyDown += Numeric_KeyDown_OnEnter;
            cmbChooseModel.SelectedIndexChanged += cmbChooseModel_SelectedIndexChanged;
        }

        private void cmbChooseModel_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (cmbChooseModel.SelectedIndex == 1)
            {
                _robotManual.Disable();
                _robotAutoNavigator.Enable();
            }
            else
            {
                cmbChooseModel.SelectedIndex = 0;
                _robotAutoNavigator.Disable();
                _robotManual.Enable();

                // ① 修复：切回手动时强制焦点回到窗体
                this.ActiveControl = null;
                BeginInvoke(new Action(() => Focus()));
            }

            // 同步一次（②：避免模式切换瞬间出现“箭头与实际方向不同”）
            lock (_robotLock)
            {
                double angle = Robot.DirectionToAngle(_robot.Direction);
                _robot.OrientationAngle = angle;
                _robot.TargetOrientationAngle = angle;
            }
        }

        private void Numeric_KeyDown_OnEnter(object sender, KeyEventArgs e)
        {
            if (e.KeyCode == Keys.Enter)
            {
                this.ActiveControl = null;  // 让数值框失去焦点，触发 ValueChanged 事件
                this.Focus();                 // 焦点回到窗体，W/A/D 立刻可用
                e.Handled = true;
                e.SuppressKeyPress = true;    // 防止系统“叮”一声
            }
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
            _mousePan.MouseDown(e);
        }

        // 鼠标移动：拖动画布
        private void Form1_MouseMove(object sender, MouseEventArgs e)
        {
            _mousePan.MouseMove(e);
            Invalidate(); // 触发重绘
        }

        // 鼠标松开：结束拖动
        private void Form1_MouseUp(object sender, MouseEventArgs e)
        {
            _mousePan.MouseUp(e);
        }

        // 窗体加载
        private void Form1_Load(object sender, EventArgs e)
        {
            _offsetX = this.ClientSize.Width / 2.0;
            _offsetY = this.ClientSize.Height / 2.0;

            _initialScale = _scale;
            _initialOffsetX = _offsetX;
            _initialOffsetY = _offsetY;

            Initialize();  // 你的原有初始化

            _centerGridManager.LoadCenter.CenterGrid();
        }

        // 窗口大小改变
        private void Form1_Resize(object sender, EventArgs e)
        {
            _centerGridManager.ResizeCenter.CenterGrid();
        }

        // W/A/D 控制
        // W/A/D 控制
        private void Form1_KeyDown(object sender, KeyEventArgs e)
        {
            if (_robot == null || _robotMove == null)
            {
                return;
            }

            switch (e.KeyCode)
            {
                case Keys.W:
                    lock (_robotLock)
                    {
                        _robot.IsForwardKeyDown = true;
                        e.Handled = true;
                        // 如果当前没有在转向，直接给出数值框配置的加速度
                        if (!_robot.IsTurning)
                        {
                            _robot.Acc = _robotAcc;
                        }
                    }
                    break;

                case Keys.A:
                    _robotMove.TurnController.StartTurnLeft();
                    e.Handled = true;
                    break;

                case Keys.D:
                    _robotMove.TurnController.StartTurnRight();
                    e.Handled = true;
                    break;
            }
        }

        private void Form1_KeyUp(object sender, KeyEventArgs e)
        {
            if (_robot == null)
            {
                return;
            }

            if (e.KeyCode == Keys.W)
            {
                lock (_robotLock)
                {
                    _robot.IsForwardKeyDown = false;
                    _robotSpeed = 0.0;
                    _robot.Acc = 0.0;
                }
            }
        }

        // Reset 按钮
        private void btnReset_Click(object sender, EventArgs e)
        {
            _centerGridManager.ResetCenter.CenterGrid();
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

        private void numericAcc_ValueChanged(object sender, EventArgs e)
        {
            lock (_robotLock)
            {
                // 数值框决定“前进时使用的加速度”
                _robotAcc = (double)((NumericUpDown)sender).Value;

                // 如果此时前进键按着且没有在转向，可以立即更新当前加速度
                if (_robot != null && _robot.IsForwardKeyDown && !_robot.IsTurning)
                {
                    _robot.Acc = _robotAcc;
                }
            }
        }

        private void numericVmax_ValueChanged(object sender, EventArgs e)
        {
            lock (_robotLock)
            {
                _robot.MaxSpeed = (double)((NumericUpDown)sender).Value;
            }
        }
    }
}
