using GridDemo.RobotModels.Pathfinding;
using GridDemo.RobotRuns;
using SkiaSharp;
using SkiaSharp.Views.Desktop;
using System;
using System.Windows.Forms;

namespace GridDemo
{
    public partial class Form1 : Form
    {
        private const int GridCount = 30;               // 网格数30
        private const double CellSizeM = 0.55;          // 一格代表距离0.55米

        private double _worldWidthM = GridCount * CellSizeM;        // 网格在世界坐标中的总宽高（米）
        private double _worldHeightM = GridCount * CellSizeM;

        private double _scale;
        private double _offsetX;        //世界坐标对应屏幕坐标偏移量
        private double _offsetY;

        private double _robotX = CellSizeM / 2;        //机器人初始世界坐标
        private double _robotY = CellSizeM / 2;

        private double _robotSpeed = 0;        //机器人的初始速度和加速度
        private double _robotMaxSpeed = 1.5;
        private double _robotAcc = 0;

        private readonly double _dt = 0.02;      //固定时间模拟步长0.02秒：用于仿真线程按固定频率推进运动更新
        private readonly object _robotLock = new object();   // 机器人共享状态锁：保护 _robotX/_robotY/_robotSpeed/_robotAcc/_robot 等多线程读写

        public Form1()
        {
            InitializeComponent();

            this.Load += Form1_Load;
            this.FormClosing += Form1_FormClosing;
            this.Resize += Form1_Resize;
            this.KeyDown += Form1_KeyDown;
            this.KeyUp += Form1_KeyUp;
            this.KeyPreview = true;     // 允许窗体截获按键，即使当前焦点在子控件

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

        /// <summary>
        /// 窗体加载：
        /// - 初始化偏移/初始视图数据；
        /// - 调用 Initialize() 创建各模块；
        /// - 执行加载居中；
        /// - 设置默认控制模式与默认寻路算法；
        /// - 将焦点切回窗体，确保键盘控制可用。
        /// </summary>
        private void Form1_Load(object sender, EventArgs e)
        {
            Initialize();  // 初始化
            _centerGrid.Center();    // 加载居中

            cmbChooseModel.SelectedIndexChanged -= cmbChooseModel_SelectedIndexChanged;            // 默认手动控制
            cmbChooseModel.SelectedIndex = 0;
            cmbChooseModel.SelectedIndexChanged += cmbChooseModel_SelectedIndexChanged;
            _robotAutoNavigator.Algorithm = EnumPathfindingAlgorithm.AStar;

            cmbPathAlgorithm.SelectedIndexChanged -= cmbPathAlgorithm_SelectedIndexChanged;            // 默认寻路算法：A*
            cmbPathAlgorithm.SelectedIndex = 1; // 0=Dijkstra, 1=A*
            cmbPathAlgorithm.SelectedIndexChanged += cmbPathAlgorithm_SelectedIndexChanged;
            _robotAutoNavigator.Algorithm = EnumPathfindingAlgorithm.AStar;

            _robotAutoNavigator.Disable();
            _robotManual.Enable();

            ActiveControl = null;      // 把焦点回到窗体（避免下拉框/数值框占用焦点导致按键无效）
            BeginInvoke(new Action(() => Focus()));
        }

        /// <summary>
        /// 窗体关闭：安全停止仿真线程，避免后台线程访问已释放的控件。
        /// </summary>
        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            _robotSimulator?._Thead_Stop();
        }

        /// <summary>
        /// 窗口大小改变：保持当前缩放不变，重新计算 offset 实现居中。
        /// </summary>
        private void Form1_Resize(object sender, EventArgs e)
        {
            _centerGrid.Center();
        }

        /// <summary>
        /// 键盘按下：W/A/D 控制。
        /// - W：开始前进（设置 IsForwardKeyDown，并在未转向时施加加速度）
        /// - A：左转（通过 RobotTurn 启动转向动画）
        /// - D：右转（通过 RobotTurn 启动转向动画）
        /// </summary>
        private void Form1_KeyDown(object sender, KeyEventArgs e)
        {
            if (_robot == null || _robotMove == null)
            { // 空值检查
                return;
            }

            switch (e.KeyCode)
            {
                case Keys.W:
                    lock (_robotLock)
                    {
                        _robot.IsForwardKeyDown = true;         // 标记前进按键按下
                        e.Handled = true;             // 标记事件已处理
                        if (!_robot.IsTurning)
                        { // 如果当前没有在转向，直接给出数值框配置的加速度
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

        /// <summary>
        /// 键盘抬起：松开 W 则停止前进（清零速度与加速度）。
        /// </summary>
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

        /// <summary>
        /// 鼠标按下：
        /// - 若在自动模式下用于拾取目的地（DestinationPicker），则优先处理并触发重绘；
        /// - 否则进入拖拽平移模式（MousePan）。
        /// </summary>
        private void Form1_MouseDown(object sender, MouseEventArgs e)
        {
            if (_destinationPicker != null && _destinationPicker.TryPick(e))
            {
                skControl.Invalidate();
                return;
            }

            _mousePan.MouseDown(e);
        }

        /// <summary>
        /// 鼠标移动：拖拽平移画布（若 MousePan 处于拖拽状态才会生效）。
        /// </summary>
        private void Form1_MouseMove(object sender, MouseEventArgs e)
        {
            _mousePan.MouseMove(e);
            skControl.Invalidate();
        }

        /// <summary>
        /// 鼠标松开：结束拖拽平移。
        /// </summary>
        private void Form1_MouseUp(object sender, MouseEventArgs e)
        {
            _mousePan.MouseUp(e);
        }

        /// <summary>
        /// Skia 绘制回调：绘制当前帧。
        /// 注意：绘制过程中会读取机器人位置/速度等共享状态，因此通过 _robotLock 保证一致性。
        /// </summary>
        private void SkControl_PaintSurface(object sender, SKPaintSurfaceEventArgs e)
        {
            SKCanvas canvas = e.Surface.Canvas;

            _drawGrid.Draw(canvas);             // 绘制顺序：先网格，再路径，再机器人，保证“路径/机器人盖在网格之上”
            _drawPath.Draw(canvas);
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

            using (var textPaint = new SKPaint              // 绘制文本样式（机器人坐标、速度、加速度）
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

        /// <summary>
        /// 加速度数值变更：更新“前进时使用的加速度”。
        /// 若此时 W 正按着且不在转向中，则立即作用到当前 Acc。
        /// </summary>
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

        /// <summary>
        /// 最大速度数值变更：实时更新 Robot 的 MaxSpeed（影响后续速度夹紧）。
        /// </summary>
        private void numericVmax_ValueChanged(object sender, EventArgs e)
        {
            lock (_robotLock)
            {
                _robot.MaxSpeed = (double)((NumericUpDown)sender).Value;
            }
        }

        /// <summary>
        /// NumericUpDown 在按 Enter 时主动失焦：
        /// - 保证 ValueChanged 能触发并应用参数；
        /// - 将焦点还给窗体，确保 W/A/D 键立即可用；
        /// - suppressKeyPress 防止系统“叮”的提示音。
        /// </summary>
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
        /// 控制模式切换：手动/自动。
        /// - 自动：禁用手动控制，启用自动导航
        /// - 手动：禁用自动导航，启用手动控制，并把焦点回到窗体以便按键立即生效
        /// </summary>
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

                // 切回手动时强制焦点回到窗体
                this.ActiveControl = null;
                BeginInvoke(new Action(() => Focus()));
            }

            // 同步一次（确保方向一致）
            lock (_robotLock)
            {
                // 使用当前实际方向计算角度
                double angle = Robot.DirectionToAngle(_robot.Direction);
                _robot.OrientationAngle = angle;
                _robot.TargetOrientationAngle = angle;

                // 确保转向状态重置
                _robot.IsTurning = false;
            }
        }

        /// <summary>
        /// 寻路算法下拉框切换：更新自动导航模块使用的算法，并在自动模式下即时重规划路径。
        /// </summary>
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

        /// <summary>
        /// Reset 按钮：将视图恢复到初始缩放并居中显示。
        /// </summary>
        private void btnReset_Click(object sender, EventArgs e)
        {
            _centerGrid.Center();
        }
    }
}
