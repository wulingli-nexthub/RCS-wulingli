using Graphic.Draws;
using Graphic.Events;
using Graphic.RobotModels.Pathfinding;
using Graphic.RobotRuns;
using Graphic.WorldView;
using Graphic.WorldView.CenterGrid;
using SkiaSharp;
using SkiaSharp.Views.Desktop;
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

        private void SkControl_PaintSurface(object sender, SKPaintSurfaceEventArgs e)
        {
            SKCanvas canvas = e.Surface.Canvas;

            _drawGrid.Draw(canvas);
            _drawRobot.Draw(canvas);

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

            using (var textPaint = new SKPaint
            {
                Color = SKColors.Black,
                IsAntialias = true,
            })
            {
                using (var font = new SKFont())
                {
                    font.Size = 15;
                    //string info = $"Scale: {_scale:F1} px/m   Offset: ({_offsetX:F0}, {_offsetY:F0})";
                    string infoRobot = $"Robot: ({robotX:F2}, {robotY:F2}), v={robotV:F2}m/s, a={robotA:F2}m/s2";
                    //canvas.DrawText(info, 10, 25, SKTextAlign.Left, font, textPaint);
                    canvas.DrawText(infoRobot, 10, 25, SKTextAlign.Left, font, textPaint);
                }
            }
        }

        public Form1()
        {
            InitializeComponent();

            this.Load += Form1_Load;
            this.FormClosing += Form1_FormClosing;
            this.Resize += Form1_Resize;
            this.KeyDown += Form1_KeyDown;
            this.KeyUp += Form1_KeyUp;
            this.KeyPreview = true;

            // 鼠标事件应绑定到 skControl，否则在画布上操作可能不会触发 Form 的鼠标事件
            this.skControl.MouseWheel += Form1_MouseWheel;
            this.skControl.MouseDown += Form1_MouseDown;
            this.skControl.MouseMove += Form1_MouseMove;
            this.skControl.MouseUp += Form1_MouseUp;

            this.skControl.PaintSurface += SkControl_PaintSurface;

            numericAcc.KeyDown += Numeric_KeyDown_OnEnter;
            numericVinit.KeyDown += Numeric_KeyDown_OnEnter;
            cmbChooseModel.SelectedIndexChanged += cmbChooseModel_SelectedIndexChanged;
            cmbPathAlgorithm.SelectedIndexChanged += cmbPathAlgorithm_SelectedIndexChanged;
        }

        private void cmbPathAlgorithm_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (_robotAutoNavigator == null)
            {
                return;
            }

            if (cmbPathAlgorithm.SelectedIndex == 0)
            {
                _robotAutoNavigator.Algorithm = EnumPathfindingAlgorithm.Dijkstra;
            }
            else
            {
                _robotAutoNavigator.Algorithm = EnumPathfindingAlgorithm.AStar;
            }

            // 自动巡航中：立刻用新算法重新规划（否则可能沿用旧路径/已结束路径导致停住）
            if (_robotAutoNavigator.IsEnabled)
            {
                _robotAutoNavigator.RebuildPath();
            }
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
            _mouseWheel.Wheel(
                e,
                screen => _worldTransform.ScreenToWorld(screen.X, screen.Y));

            skControl.Invalidate();
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
            skControl.Invalidate();
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

            // ① 默认手动控制
            cmbChooseModel.SelectedIndexChanged -= cmbChooseModel_SelectedIndexChanged;
            cmbChooseModel.SelectedIndex = 0;
            cmbChooseModel.SelectedIndexChanged += cmbChooseModel_SelectedIndexChanged;
            _robotAutoNavigator.Algorithm = EnumPathfindingAlgorithm.AStar;

            // 默认寻路算法：A*
            cmbPathAlgorithm.SelectedIndexChanged -= cmbPathAlgorithm_SelectedIndexChanged;
            cmbPathAlgorithm.SelectedIndex = 1; // 0=Dijkstra, 1=A*
            cmbPathAlgorithm.SelectedIndexChanged += cmbPathAlgorithm_SelectedIndexChanged;
            _robotAutoNavigator.Algorithm = EnumPathfindingAlgorithm.AStar;

            _robotAutoNavigator.Disable();
            _robotManual.Enable();
            ActiveControl = null;
            BeginInvoke(new Action(() => Focus()));
        }

        // 窗口大小改变
        private void Form1_Resize(object sender, EventArgs e)
        {
            _centerGridManager.ResizeCenter.CenterGrid();
        }

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
            _robotSimulator?._Thead_Stop();
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
