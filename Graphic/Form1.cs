using GridDemo.RobotModels.Pathfinding;
using GridDemo.Robots;
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

        private bool _isObstacleEditMode;

        // 业务引擎与仿真循环
        private RobotEngine _engine;
        private RobotSimulationLoop _simulation;

        // 画图相关（基本保持原有）
        private WorldView.WorldTransform _worldTransform;
        private Draws.DrawGrid _drawGrid;
        private Draws.DrawObstacles _drawObstacles;
        private Draws.DrawPath _drawPath;
        private Draws.DrawRobot _drawRobot;
        private Events.MouseWheel _mouseWheel;
        private Events.MousePan _mousePan;
        private WorldView.CenterGrid.CenterGrid _centerGrid;
        private Events.DestinationPicker _destinationPicker;
        private EnumPathfindingAlgorithm _currentAlgorithm;
        private readonly double _dt = 0.02;  // 仿真步长 20ms

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
            // 1. 创建业务引擎
            _engine = new RobotEngine(
                gridCount: GridCount,
                cellSizeM: CellSizeM,
                dt: _dt,
                initialMaxSpeed: 1.5,
                initialDirection: EnumMoveDirection.Right);

            _worldWidthM = _engine.WorldWidthM;
            _worldHeightM = _engine.WorldHeightM;

            // 2. 初始化 WorldTransform 与 Draw 层
            _scale = 50;          // 比如一个默认缩放，可以重用你原来的初始值
            _offsetX = 0;
            _offsetY = 0;

            _worldTransform = new WorldView.WorldTransform(_scale, (float)_offsetX, (float)_offsetY);

            _drawGrid = new Draws.DrawGrid(_worldTransform, _worldWidthM, _worldHeightM);

            _drawObstacles = new Draws.DrawObstacles(
                _worldTransform,
                getObstacleSnapshot: () => _engine.GetObstacleSnapshot(),
                getCellSizeM: () => _engine.CellSizeM);

            _drawPath = new Draws.DrawPath(
                _worldTransform,
                getPathPointsSnapshot: () => _engine.GetPathWorldPointsSnapshot());

            _drawRobot = new Draws.DrawRobot(
                _worldTransform,
                robotLock: new object(), // DrawRobot 内部只在绘制时读位置，不需要真实锁，可传一个 dummy
                getRobotPosition: () =>
                {
                    var s = _engine.GetStateSnapshot();
                    return (s.X, s.Y);
                },
                getScale: () => _scale,
                getOrientationAngle: () => _engine.GetStateSnapshot().OrientationAngle);

            // 3. 鼠标缩放/平移
            _mouseWheel = new Events.MouseWheel(
                form: this,
                getState: () => (_scale, _offsetX, _offsetY),
                setScale: s => _scale = s,
                setOffset: (ox, oy) =>
                {
                    _offsetX = ox;
                    _offsetY = oy;
                },
                updateWorldTransform: (s, ox, oy) => _worldTransform.Update(s, ox, oy)
            );

            _mousePan = new Events.MousePan(
                form: this,
                getState: () => (_offsetX, _offsetY, _scale),
                setOffset: (ox, oy) =>
                {
                    _offsetX = ox;
                    _offsetY = oy;
                },
                updateWorldTransform: (s, ox, oy) => _worldTransform.Update(s, ox, oy)
            );

            _centerGrid = new WorldView.CenterGrid.CenterGrid(
                host: skControl,
                getWorldWidthM: () => _worldWidthM,
                getWorldHeightM: () => _worldHeightM,
                setScale: s => _scale = s,
                setOffsetX: x => _offsetX = x,
                setOffsetY: y => _offsetY = y,
                updateWorldTransform: (s, ox, oy) => _worldTransform.Update(s, ox, oy)
            );

            _destinationPicker = new Events.DestinationPicker(
                transform: _worldTransform,
                navigator: _engine.AutoNavigator,           // 直接传引擎内部的导航器
                getWorldWidthM: () => _worldWidthM,
                getWorldHeightM: () => _worldHeightM,
                cellSizeM: _engine.CellSizeM);

            // 4. 创建仿真循环
            _simulation = new RobotSimulationLoop(_engine, skControl, _dt);
            _simulation.Start();

            // 5. 默认模式与算法
            _centerGrid.Center();

            cmbChooseModel.SelectedIndexChanged -= cmbChooseModel_SelectedIndexChanged;
            cmbChooseModel.SelectedIndex = 1;  // Auto
            cmbChooseModel.SelectedIndexChanged += cmbChooseModel_SelectedIndexChanged;
            _engine.EnableAuto();

            cmbPathAlgorithm.SelectedIndexChanged -= cmbPathAlgorithm_SelectedIndexChanged;
            cmbPathAlgorithm.SelectedIndex = 2; // A*
            cmbPathAlgorithm.SelectedIndexChanged += cmbPathAlgorithm_SelectedIndexChanged;
            _engine.Algorithm = EnumPathfindingAlgorithm.Serpentine;
            _currentAlgorithm = EnumPathfindingAlgorithm.Serpentine;
            ActiveControl = null;
            BeginInvoke(new Action(() => Focus()));
        }

        /// <summary>
        /// 窗体关闭：安全停止仿真线程，避免后台线程访问已释放的控件。
        /// </summary>
        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            _simulation?.Stop();
        }

        /// <summary>
        /// 窗口大小改变：保持当前缩放不变，重新计算 offset 实现居中。
        /// </summary>
        private void Form1_Resize(object sender, EventArgs e)
        {
            _centerGrid.Center();
        }

        /// <summary>
        /// 键盘输入控制手动模式下机器人移动：W 前进，A 左转，D 右转。
        ///  - 注意：只修改按键状态，实际加速度/转向在仿真循环中应用。
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Form1_KeyDown(object sender, KeyEventArgs e)
        {
            if (_engine == null) return;

            switch (e.KeyCode)
            {
                case Keys.W:
                    _engine.ManualForwardKey(true);
                    e.Handled = true;
                    break;
                case Keys.A:
                    _engine.ManualTurnLeftKey(true);
                    e.Handled = true;
                    break;
                case Keys.D:
                    _engine.ManualTurnRightKey(true);
                    e.Handled = true;
                    break;
            }
        }

        private void Form1_KeyUp(object sender, KeyEventArgs e)
        {
            if (_engine == null) return;

            switch (e.KeyCode)
            {
                case Keys.W:
                    _engine.ManualForwardKey(false);
                    e.Handled = true;
                    break;
                case Keys.A:
                    _engine.ManualTurnLeftKey(false);
                    e.Handled = true;
                    break;
                case Keys.D:
                    _engine.ManualTurnRightKey(false);
                    e.Handled = true;
                    break;
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
        /// - 若在自动模式下且不是蛇形（因为蛇形模式固定目标点是右下角）用于拾取目的地（DestinationPicker），则优先处理并触发重绘；
        /// - 否则进入拖拽平移模式（MousePan）。
        /// </summary>
        private void Form1_MouseDown(object sender, MouseEventArgs e)
        {
            if (_isObstacleEditMode && e.Button == MouseButtons.Left)
            {
                TryToggleObstacleAtMouse(e);
                skControl.Invalidate();
                return;
            }
            if (_currentAlgorithm != EnumPathfindingAlgorithm.Serpentine
                && _destinationPicker != null
                && _destinationPicker.TryPick(e))
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

            _drawGrid.Draw(canvas);
            _drawObstacles.Draw(canvas);

            // 蛇形模式不画路径
            if (_currentAlgorithm != EnumPathfindingAlgorithm.Serpentine)
            {
                _drawPath.Draw(canvas);
            }

            _drawRobot.Draw(canvas);

            RobotStateSnapshot s = default;
            if (_engine != null)
            {
                s = _engine.GetStateSnapshot();
            }

            using (var textPaint = new SKPaint
            {
                Color = SKColors.Black,
                IsAntialias = true,
            })
            using (var font = new SKFont { Size = 15 })
            {
                string infoRobot =
                    $"Robot: ({s.X:F2}, {s.Y:F2}), v={s.Speed:F2}m/s, a={s.Acc:F2}m/s2";

                canvas.DrawText(infoRobot, 10, 25, SKTextAlign.Left, font, textPaint);
            }
        }

        /// <summary>
        /// 加速度数值变更：更新“前进时使用的加速度”。
        /// 若此时 W 正按着且不在转向中，则立即作用到当前 Acc。
        /// </summary>
        private void numericAcc_ValueChanged(object sender, EventArgs e)
        {
            if (_engine == null)
            {
                return;
            }

            _engine.SetForwardAcc((double)((NumericUpDown)sender).Value);
        }

        /// <summary>
        /// 最大速度数值变更：实时更新 Robot 的 MaxSpeed（影响后续速度夹紧）。
        /// </summary>
        private void numericVmax_ValueChanged(object sender, EventArgs e)
        {
            if (_engine == null)
            {
                return;
            }

            _engine.SetMaxSpeed((double)((NumericUpDown)sender).Value);
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
            if (_engine == null)
            {
                return;
            }

            if (cmbChooseModel.SelectedIndex == 1)
            {
                _engine.EnableAuto();
            }
            else
            {
                cmbChooseModel.SelectedIndex = 0;
                _engine.EnableManual();
                this.ActiveControl = null;
                BeginInvoke(new Action(() => Focus()));
            }
        }

        /// <summary>
        /// 寻路算法下拉框切换：更新自动导航模块使用的算法，并在自动模式下即时重规划路径。
        /// </summary>
        private void cmbPathAlgorithm_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (_engine == null)
            {
                return;
            }

            // 0: Dijkstra, 1: A*, 2: Serpentine
            if (cmbPathAlgorithm.SelectedIndex == 0)
            {
                _engine.Algorithm = EnumPathfindingAlgorithm.Dijkstra;
                _currentAlgorithm = EnumPathfindingAlgorithm.Dijkstra;
            }
            else if (cmbPathAlgorithm.SelectedIndex == 1)
            {
                _engine.Algorithm = EnumPathfindingAlgorithm.AStar;
                _currentAlgorithm = EnumPathfindingAlgorithm.AStar;
            }
            else if (cmbPathAlgorithm.SelectedIndex == 2)
            {
                _engine.Algorithm = EnumPathfindingAlgorithm.Serpentine;
                _currentAlgorithm = EnumPathfindingAlgorithm.Serpentine;

                // 蛇形模式：目标点自动设置为右下角，不需要鼠标右键
                _engine.AutoNavigator.SetGoal(new GridPos(GridCount - 1, GridCount - 1), rebuildIfEnabled: true);
            }


            if (_engine.AutoEnabled)
            {
                _engine.RebuildPath();
            }
        }

        /// <summary>
        /// Reset 按钮：将视图恢复到初始缩放并居中显示。
        /// </summary>
        private void btnReset_Click(object sender, EventArgs e)
        {
            _centerGrid.Center();
        }

        private void btnObstacle_Click(object sender, EventArgs e)
        {
            _isObstacleEditMode = !_isObstacleEditMode;

            btnObstacle.Text = _isObstacleEditMode ? "设置障碍物：开" : "设置障碍物：关";

            // 通知业务引擎进入/退出障碍物编辑模式
            if (_engine != null)
            {
                _engine.SetObstacleEditMode(_isObstacleEditMode);
            }

            // 避免设置障碍物时误触发平移的 Hand 光标残留
            if (!_isObstacleEditMode)
            {
                this.Cursor = Cursors.Default;
            }

            skControl.Invalidate();
        }

        private void TryToggleObstacleAtMouse(MouseEventArgs e)
        {
            if (_engine == null || _worldTransform == null)
            {
                return;
            }

            var world = _worldTransform.ScreenToWorld(e.X, e.Y);

            int gx = (int)Math.Floor(world.X / _engine.CellSizeM);
            int gy = (int)Math.Floor(world.Y / _engine.CellSizeM);

            if (gx < 0 || gy < 0 || gx >= _engine.GridCount || gy >= _engine.GridCount)
            {
                return;
            }

            _engine.ToggleObstacle(new GridPos(gx, gy));
            skControl.Invalidate();
        }

        private void btnClearObstacle_Click(object sender, EventArgs e)
        {
            if (_engine == null)
            {
                return;
            }

            _engine.ClearObstacles();

            // 清空障碍物后，保持“设置障碍物模式”的 UI 不变，只刷新画面即可
            skControl.Invalidate();
        }
    }
}
