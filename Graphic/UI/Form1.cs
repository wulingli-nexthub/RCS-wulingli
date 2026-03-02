using GridDemo.Models.Pathfinding;
using GridDemo.Models;
using SkiaSharp;
using SkiaSharp.Views.Desktop;
using System;
using System.Diagnostics;
using System.IO;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace GridDemo
{
    public partial class Form1 : Form
    {
        private static readonly object _logLock = new object();
        private static string _logFilePath;

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

        // UI 定时刷新（在 UI 线程执行）
        private System.Windows.Forms.Timer _renderTimer;

        // 画图相关（基本保持原有）
        private WorldView.WorldTransform _worldTransform;
        private Draws.DrawGrid _drawGrid;
        private Draws.DrawObstacles _drawObstacles;
        private Draws.DrawPath _drawPath;
        private Draws.DrawRobot _drawRobot;
        private Draws.DrawClaimedCells _drawClaimedCells;
        private Events.MouseWheel _mouseWheel;
        private Events.MousePan _mousePan;
        private WorldView.CenterGrid.CenterGrid _centerGrid;
        private Events.DestinationPicker _destinationPicker;
        private EnumPathfindingAlgorithm _currentAlgorithm;
        private readonly double _dt = 0.02;  // 仿真步长 20ms

        private const double SelectHitRadiusPx = 18.0;   // 鼠标选中机器人时的命中半径（像素） 

        public Form1()
        {
            InitializeComponent();

            InitLogging();
            HookUnhandledExceptions();
            Log("Form1.ctor: init, dt=" + _dt);

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
            numericAddRobot.KeyDown += Numeric_KeyDown_OnEnter;
            cmbPathAlgorithm.SelectedIndexChanged += cmbPathAlgorithm_SelectedIndexChanged;

            numericAddRobot.ValueChanged += numericAddRobot_ValueChanged;
            btnResetRobot.Click += btnResetRobot_Click;
            btnStart.Click += btnStart_Click;
            btnStop.Click += btnStop_Click;

            if (lvRobotStates != null)
            {
                lvRobotStates.ItemSelectionChanged += lvRobotStates_ItemSelectionChanged;
                lvRobotStates.MouseDoubleClick += lvRobotStates_MouseDoubleClick;
            }
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
            Log("Form1_Load: begin");

            // 1. 创建业务引擎
            _engine = new RobotEngine(
                gridCount: GridCount,
                cellSizeM: CellSizeM,
                dt: _dt,
                initialMaxSpeed: 1.5,
                initialDirection: EnumMoveDirection.Right);

            // 窗体加载后：不默认选中任何机器人
            _engine.ClearSelectedRobot();

            _worldWidthM = _engine.WorldWidthM;
            _worldHeightM = _engine.WorldHeightM;

            Log("Engine created. GridCount=" + _engine.GridCount
                + ", CellSizeM=" + _engine.CellSizeM
                + ", World=(" + _worldWidthM.ToString("F2") + "m," + _worldHeightM.ToString("F2") + "m)");

            // 2. 初始化 WorldTransform 与 Draw 层
            _scale = 50;
            _offsetX = 0;
            _offsetY = 0;

            Log("Init view: scale=" + _scale.ToString("F2") + ", offset=(" + _offsetX.ToString("F2") + "," + _offsetY.ToString("F2") + ")");

            _worldTransform = new WorldView.WorldTransform(_scale, (float)_offsetX, (float)_offsetY);

            _drawGrid = new Draws.DrawGrid(_worldTransform, _worldWidthM, _worldHeightM);

            _drawObstacles = new Draws.DrawObstacles(
                _worldTransform,
                getObstacleSnapshot: () => _engine.GetObstacleSnapshot(),
                getCellSizeM: () => _engine.CellSizeM);

            _drawPath = new Draws.DrawPath(
                _worldTransform,
                getPathPointsSnapshot: () => _engine.GetPathWorldPointsSnapshot(),
                getRobotWorldPos: () =>
                {
                    var s = _engine.GetStateSnapshot();
                    return (s.X, s.Y);
                });

            _drawClaimedCells = new Draws.DrawClaimedCells(
                _worldTransform,
                getClaimedCellsSnapshot: () => _engine.GetClaimedCellsSnapshot(),
                getCellSizeM: () => _engine.CellSizeM);


            _drawRobot = new Draws.DrawRobot(
                _worldTransform,
                robotLock: new object(),
                getRobotsSnapshot: () =>
                {
                    var states = _engine.GetRobotStatesSnapshot();
                    var list = new System.Collections.Generic.List<(int Id, double X, double Y, double Angle)>(states.Count);
                    for (int i = 0; i < states.Count; i++)
                    {
                        list.Add((i, states[i].X, states[i].Y, states[i].OrientationAngle));
                    }
                    return list;
                },
                getGoalsSnapshot: () => _engine.GetRobotsGoalWorldSnapshot(),
                getSelectedId: () => _engine.SelectedRobotId,
                getScale: () => _scale);

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

            // 4. 创建仿真循环
            _simulation = new RobotSimulationLoop(_engine, _dt);
            _simulation.Start();
            Log("Simulation started. dt=" + _dt);
            // 4.1 UI 线程定时重绘
            int intervalMs = (int)Math.Max(1.0, Math.Round(_dt * 1000));
            _renderTimer = new System.Windows.Forms.Timer();
            _renderTimer.Interval = intervalMs;
            _renderTimer.Tick += (s, ev) =>
            {
                if (!skControl.IsDisposed)
                {
                    skControl.Invalidate();
                }

                UpdateRobotStatesList();
            };
            _renderTimer.Start();
            Log("Render timer started. intervalMs=" + intervalMs);

            // 5. 默认模式与算法
            _engine.EnableAuto();
            Log("Default control mode: Auto");

            // 新增：默认暂停，必须点击“启动”才开始运动
            _engine.PauseAll();
            Log("Form1_Load: paused by default, wait for Start");

            cmbPathAlgorithm.SelectedIndexChanged -= cmbPathAlgorithm_SelectedIndexChanged;
            cmbPathAlgorithm.SelectedIndex = 1; // A*
            cmbPathAlgorithm.SelectedIndexChanged += cmbPathAlgorithm_SelectedIndexChanged;
            _engine.Algorithm = EnumPathfindingAlgorithm.AStar;
            _currentAlgorithm = EnumPathfindingAlgorithm.AStar;

            Log("Default algorithm: " + _currentAlgorithm);

            ActiveControl = null;
            BeginInvoke(new Action(() =>
            {
                _centerGrid.Center();
                skControl.Invalidate();
            }));

            Log("Form1_Load: end");
        }

        /// <summary>
        /// 窗体关闭：安全停止仿真线程，避免后台线程访问已释放的控件。
        /// </summary>
        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            Log("Form1_FormClosing: begin");

            if (_renderTimer != null)
            {
                _renderTimer.Stop();
                _renderTimer.Dispose();
                _renderTimer = null;
                Log("Form1_FormClosing: render timer stopped");
            }

            _simulation?.Stop();
            Log("Form1_FormClosing: simulation stop requested");
        }

        /// <summary>
        /// 窗口大小改变：保持当前缩放不变，重新计算 offset 实现居中。
        /// </summary>
        private void Form1_Resize(object sender, EventArgs e)
        {
            Log("Form1_Resize: client=(" + ClientSize.Width + "," + ClientSize.Height + "), scale=" + _scale.ToString("F2"));
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
            if (_engine == null)
            {
                return;
            }

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
            if (_worldTransform != null)
            {
                var before = _worldTransform.ScreenToWorld(e.X, e.Y);
                Log("MouseWheel: delta=" + e.Delta
                    + ", mouse=(" + e.X + "," + e.Y + ")"
                    + ", worldBefore=(" + before.X.ToString("F2") + "," + before.Y.ToString("F2") + ")"
                    + ", scaleBefore=" + _scale.ToString("F2"));
            }

            _mouseWheel.Wheel(
                e,
                screen => _worldTransform.ScreenToWorld(screen.X, screen.Y));

            Log("MouseWheel: scaleAfter=" + _scale.ToString("F2") + ", offset=(" + _offsetX.ToString("F2") + "," + _offsetY.ToString("F2") + ")");

            skControl.Invalidate();
        }

        /// <summary>
        /// 鼠标按下：
        /// - 若在自动模式下且不是蛇形（因为蛇形模式固定目标点是右下角）用于拾取目的地（DestinationPicker），则优先处理并触发重绘；
        /// - 否则进入拖拽平移模式（MousePan）。
        /// </summary>
        private void Form1_MouseDown(object sender, MouseEventArgs e)
        {
            // 仅记录关键点击，不记录 MouseMove
            Log("MouseDown: button=" + e.Button + ", pos=(" + e.X + "," + e.Y + "), obstacleEdit=" + _isObstacleEditMode);

            if (_isObstacleEditMode && e.Button == MouseButtons.Left)
            {
                TryToggleObstacleAtMouse(e);
                skControl.Invalidate();
                return;
            }
            // 左键：优先选中机器人
            if (e.Button == MouseButtons.Left)
            {
                if (TrySelectRobotAtMouse(e.X, e.Y))
                {
                    skControl.Invalidate();
                    return;
                }
                else
                {
                    // 点击空白区域，取消选中
                    _engine.ClearSelectedRobot();
                    Log("ClearSelectedRobot: click on empty area");
                    skControl.Invalidate();
                }
            }

            // 右键：仅在非蛇形时，给“选中机器人”选目的地
            if (e.Button == MouseButtons.Right && _currentAlgorithm != EnumPathfindingAlgorithm.Serpentine)
            {
                var world = _worldTransform.ScreenToWorld(e.X, e.Y);

                int gx = (int)Math.Floor(world.X / _engine.CellSizeM);
                int gy = (int)Math.Floor(world.Y / _engine.CellSizeM);

                if (gx >= 0 && gy >= 0 && gx < _engine.GridCount && gy < _engine.GridCount)
                {
                    _engine.TrySetSelectedRobotGoal(new GridPos(gx, gy));
                    Log("SetGoal for selected robot: (" + gx + "," + gy + ")");
                    skControl.Invalidate();
                    return;
                }
            }

            _mousePan.MouseDown(e);
        }

        /// <summary>
        /// 鼠标左键选中机器人。
        /// </summary>
        private bool TrySelectRobotAtMouse(int mouseX, int mouseY)
        {
            if (_engine == null || _worldTransform == null)
            {
                return false;
            }

            var states = _engine.GetRobotStatesSnapshot();

            double bestD2 = double.MaxValue;
            int bestId = -1;

            for (int i = 0; i < states.Count; i++)
            {
                var s = states[i];
                var sp = _worldTransform.WorldToScreen(s.X, s.Y);

                double dx = sp.X - mouseX;
                double dy = sp.Y - mouseY;
                double d2 = dx * dx + dy * dy;

                if (d2 < bestD2)
                {
                    bestD2 = d2;
                    bestId = i;
                }
            }

            if (bestId >= 0 && bestD2 <= SelectHitRadiusPx * SelectHitRadiusPx)
            {
                _engine.SelectRobot(bestId);
                Log("SelectRobot by mouse: " + (bestId + 1));
                return true;
            }

            return false;
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
            Log("MouseUp: button=" + e.Button + ", pos=(" + e.X + "," + e.Y + "), offset=(" + _offsetX.ToString("F2") + "," + _offsetY.ToString("F2") + ")");
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

            // 先画抢占格子（在路径与机器人之下）
            _drawClaimedCells.Draw(canvas);

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

            double a = (double)((NumericUpDown)sender).Value;
            Log("numericAcc_ValueChanged: acc=" + a.ToString("F3"));
            _engine.SetForwardAcc(a);
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

            double v = (double)((NumericUpDown)sender).Value;
            Log("numericVmax_ValueChanged: vmax=" + v.ToString("F3"));
            _engine.SetMaxSpeed(v);
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
                Log("Numeric Enter: focus->form");
                this.ActiveControl = null;  // 让数值框失去焦点，触发 ValueChanged 事件
                this.Focus();              // 焦点回到窗体，W/A/D 立刻可用
                e.Handled = true;
                e.SuppressKeyPress = true;    // 防止系统“叮”一声
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

            EnumPathfindingAlgorithm old = _currentAlgorithm;

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

                //// 蛇形模式：目标点自动设置为右下角，不需要鼠标右键
                //_engine.AutoNavigator.SetGoal(new GridPos(GridCount - 1, GridCount - 1), rebuildIfEnabled: true);
                //Log("Serpentine goal forced: (" + (GridCount - 1) + "," + (GridCount - 1) + ")");
            }

            Log("Algorithm changed: " + old + " -> " + _currentAlgorithm + ", autoEnabled=" + _engine.AutoEnabled);

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
            Log("btnReset_Click");
            _centerGrid.Center();
        }

        private void btnObstacle_Click(object sender, EventArgs e)
        {
            _isObstacleEditMode = !_isObstacleEditMode;

            btnObstacle.Text = _isObstacleEditMode ? "开" : "关";

            Log("btnObstacle_Click: obstacleEdit=" + _isObstacleEditMode);

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

        /// <summary>
        /// 鼠标左键点击切换障碍物状态。
        /// </summary>
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
            Log("ToggleObstacle: (" + gx + "," + gy + ")");
            skControl.Invalidate();
        }

        /// <summary>
        /// 清空障碍物按钮：清空所有障碍物。
        /// </summary>
        private void btnClearObstacle_Click(object sender, EventArgs e)
        {
            if (_engine == null)
            {
                return;
            }

            _engine.ClearObstacles();
            Log("btnClearObstacle_Click: cleared");

            // 清空障碍物后，保持“设置障碍物模式”的 UI 不变，只刷新画面即可
            skControl.Invalidate();
        }


        private static void InitLogging()
        {
            string dir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "logs");
            Directory.CreateDirectory(dir);

            _logFilePath = Path.Combine(dir, "app.log");

            Log("=== App start ===");
            Log("BaseDirectory=" + AppDomain.CurrentDomain.BaseDirectory);
        }

        private static void HookUnhandledExceptions()
        {
            Application.ThreadException += (s, e) =>
            {
                Log("Application.ThreadException: " + e.Exception);
            };

            AppDomain.CurrentDomain.UnhandledException += (s, e) =>
            {
                Log("AppDomain.UnhandledException: " + (e.ExceptionObject as Exception));
            };

            TaskScheduler.UnobservedTaskException += (s, e) =>
            {
                Log("TaskScheduler.UnobservedTaskException: " + e.Exception);
                e.SetObserved();
            };
        }

        private static void Log(string message)
        {
            string line = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fff") + " " + message;

            // 1) 输出到 VS 输出窗口（调试/运行时可见）
            Debug.WriteLine(line);

            // 2) 输出到文件（发布后可追溯）
            lock (_logLock)
            {
                File.AppendAllText(_logFilePath, line + Environment.NewLine, Encoding.UTF8);
            }
        }

        private void numericAddRobot_ValueChanged(object sender, EventArgs e)
        {
            if (_engine == null)
            {
                return;
            }

            int count = (int)numericAddRobot.Value;
            Log("numericAddRobot_ValueChanged: count=" + count);

            _engine.SetRobotCount(count, initialMaxSpeed: (double)numericVinit.Value, initialDirection: EnumMoveDirection.Right);

            UpdateRobotStatesList();
            skControl.Invalidate();
        }

        private void btnResetRobot_Click(object sender, EventArgs e)
        {
            if (_engine == null)
            {
                return;
            }

            Log("btnResetRobot_Click: reset single robot roam");

            numericAddRobot.ValueChanged -= numericAddRobot_ValueChanged;
            numericAddRobot.Value = 1;
            numericAddRobot.ValueChanged += numericAddRobot_ValueChanged;

            _engine.ResetToSingleRobotRandomRoam(initialMaxSpeed: (double)numericVinit.Value, initialDirection: EnumMoveDirection.Right);
            _engine.ClearSelectedRobot();

            UpdateRobotStatesList();
            skControl.Invalidate();
        }

        private void btnStart_Click(object sender, EventArgs e)
        {
            if (_engine == null)
            {
                return;
            }

            _engine.StartAll();
            Log("btnStart_Click: StartAll");

            UpdateRobotStatesList();
            skControl.Invalidate();
        }

        private void btnStop_Click(object sender, EventArgs e)
        {
            if (_engine == null)
            {
                return;
            }

            _engine.PauseAll();
            Log("btnStop_Click: PauseAll");

            UpdateRobotStatesList();
            skControl.Invalidate();
        }

        /// <summary> 
        /// 定时刷新机器人状态列表，包括模式列（自动/手动）。
        /// 模式列显示"自动"或"手动"，用户可双击切换。
        /// 更新选中行时临时解绑 ItemSelectionChanged，避免"更新→事件→更新"的循环导致闪烁。
        /// </summary>
        private void UpdateRobotStatesList()
        {
            if (_engine == null || lvRobotStates == null || lvRobotStates.IsDisposed)
                return;
             
            var states = _engine.GetRobotStatesSnapshot();
            int selectedId = _engine.SelectedRobotId;

            // 临时解绑选中事件，避免程序化更改选中行时触发回调
            lvRobotStates.ItemSelectionChanged -= lvRobotStates_ItemSelectionChanged;

            lvRobotStates.BeginUpdate();
            try
            {
                // 每行有 6 列：Id, 模式, (X,Y), V, A, Ang
                while (lvRobotStates.Items.Count < states.Count)
                {
                    var item = new ListViewItem();
                    item.SubItems.Add("");  // colMode
                    item.SubItems.Add("");  // colPos
                    item.SubItems.Add("");  // colSpeed
                    item.SubItems.Add("");  // colAcc
                    item.SubItems.Add("");  // colAngle
                    lvRobotStates.Items.Add(item);
                }

                while (lvRobotStates.Items.Count > states.Count)
                {
                    lvRobotStates.Items.RemoveAt(lvRobotStates.Items.Count - 1);
                }

                for (int i = 0; i < states.Count; i++)
                {
                    var s = states[i];
                    var item = lvRobotStates.Items[i];

                    string idText = (i + 1).ToString();
                    string modeText = s.IsAutoMode ? "自动" : "手动";
                    string posText = string.Format("{0:F2},{1:F2}", s.X, s.Y);
                    string vText = s.Speed.ToString("F2");
                    string aText = s.Acc.ToString("F2");
                    string angText = s.OrientationAngle.ToString("F1");

                    if (item.Text != idText) item.Text = idText;
                    if (item.SubItems[1].Text != modeText) item.SubItems[1].Text = modeText;
                    if (item.SubItems[2].Text != posText) item.SubItems[2].Text = posText;
                    if (item.SubItems[3].Text != vText) item.SubItems[3].Text = vText;
                    if (item.SubItems[4].Text != aText) item.SubItems[4].Text = aText;
                    if (item.SubItems[5].Text != angText) item.SubItems[5].Text = angText;

                    item.Tag = i;
                }

                if (selectedId >= 0 && selectedId < lvRobotStates.Items.Count)
                {
                    var selItem = lvRobotStates.Items[selectedId];
                    if (!selItem.Selected)
                    {
                        lvRobotStates.SelectedIndices.Clear();
                        selItem.Selected = true;
                    }
                }
                else
                {
                    if (lvRobotStates.SelectedIndices.Count > 0)
                        lvRobotStates.SelectedIndices.Clear();
                }
            }
            finally
            {
                lvRobotStates.EndUpdate();

                // 恢复选中事件绑定
                lvRobotStates.ItemSelectionChanged += lvRobotStates_ItemSelectionChanged;
            }
        }

        /// <summary>
        /// 单击选中机器人。
        /// </summary>
        private void lvRobotStates_ItemSelectionChanged(object sender, ListViewItemSelectionChangedEventArgs e)
        {
            if (!e.IsSelected)
                return;

            if (_engine == null)
                return;

            int id = e.ItemIndex;
            _engine.SelectRobot(id);
            skControl.Invalidate();
        }

        /// <summary>
        /// 双击列表行：切换该机器人的自动/手动模式。
        /// - 当前为自动 → 切换为手动（同时选中该机器人，使 W/A/D 键对其生效）；
        /// - 当前为手动 → 切换为自动。
        /// </summary>
        private void lvRobotStates_MouseDoubleClick(object sender, MouseEventArgs e)
        {
            if (_engine == null)
                return;

            ListViewHitTestInfo hit = lvRobotStates.HitTest(e.Location);
            if (hit.Item == null)
                return;

            int robotId = hit.Item.Index;

            // 先选中该机器人
            _engine.SelectRobot(robotId);

            // 读取当前模式并切换
            bool isAuto = _engine.IsRobotAutoMode(robotId);
            if (isAuto)
            {
                _engine.EnableManualForRobot(robotId);
                Log("ToggleMode: robot " + (robotId + 1) + " -> 手动");

                // 手动模式需要焦点回到窗体，确保 W/A/D 键立即可用
                this.ActiveControl = null;
                BeginInvoke(new Action(() => Focus()));
            }
            else
            {
                _engine.EnableAutoForRobot(robotId);
                Log("ToggleMode: robot " + (robotId + 1) + " -> 自动");
            }

            UpdateRobotStatesList();
            skControl.Invalidate();
        }

        /// <summary>
        /// "保存地图"按钮点击：
        /// 仅在障碍物编辑模式关闭时可用，将当前障碍物信息保存为 JSON 文件。
        /// 弹出保存文件对话框，由用户选择保存位置和文件名。
        /// </summary>
        private void btnSaveMap_Click(object sender, EventArgs e)
        {
            if (_engine == null) return;

            // 仅在障碍物设置关闭后才允许保存
            if (_isObstacleEditMode)
            {
                MessageBox.Show("请先关闭障碍物设置再保存地图。", "提示",
                    MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }

            using (var dlg = new SaveFileDialog())
            {
                dlg.Title = "保存地图文件";
                dlg.Filter = "JSON 地图文件 (*.json)|*.json|所有文件 (*.*)|*.*";
                dlg.DefaultExt = "json";
                dlg.FileName = "map.json";

                if (dlg.ShowDialog(this) != DialogResult.OK)
                    return;

                try
                {
                    _engine.SaveMap(dlg.FileName);
                    Log("SaveMap: " + dlg.FileName);
                    MessageBox.Show("地图已保存到：\n" + dlg.FileName, "保存成功",
                        MessageBoxButtons.OK, MessageBoxIcon.Information);
                }
                catch (Exception ex)
                {
                    Log("SaveMap error: " + ex);
                    MessageBox.Show("保存地图失败：\n" + ex.Message, "错误",
                        MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
            }
        }

        /// <summary>
        /// "载入地图"按钮点击：
        /// 从 JSON 文件读取障碍物信息并应用到当前地图。
        /// 载入会清空当前障碍物，然后按文件内容重新设置。
        /// </summary>
        private void btnLoadMap_Click(object sender, EventArgs e)
        {
            if (_engine == null) return;

            using (var dlg = new OpenFileDialog())
            {
                dlg.Title = "载入地图文件";
                dlg.Filter = "JSON 地图文件 (*.json)|*.json|所有文件 (*.*)|*.*";

                if (dlg.ShowDialog(this) != DialogResult.OK)
                    return;

                try
                {
                    _engine.LoadMap(dlg.FileName);
                    Log("LoadMap: " + dlg.FileName);
                    skControl.Invalidate();
                    MessageBox.Show("地图载入成功！", "载入成功",
                        MessageBoxButtons.OK, MessageBoxIcon.Information);
                }
                catch (Exception ex)
                {
                    Log("LoadMap error: " + ex);
                    MessageBox.Show("载入地图失败：\n" + ex.Message, "错误",
                        MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
            }
        }

    }
}
