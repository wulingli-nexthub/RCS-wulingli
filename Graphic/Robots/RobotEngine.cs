using Graphic.Maps;
using GridDemo.RobotModels;
using GridDemo.RobotModels.Pathfinding;
using GridDemo.RobotRuns;
using System;
using System.Collections.Generic;

namespace GridDemo.Robots
{
    /// <summary>
    /// 流程状态枚举
    /// </summary>
    internal enum EnumRobotProcessState
    {
        Idle,                  // 待命状态
        AutoNavigating,       // 自动导航中，拉取指令进入指令队列
        ManualControl,        // 手动控制中，响应按键输入
        ObstacleEditing,      // 障碍物编辑中，进入该状态立即停车
        Error                  // 错误状态（停机）
    }

    /// <summary>
    /// 业务引擎（无 UI 依赖）：
    /// - 统一封装机器人运动学（Move/Turn）、自动寻路（AutoNavigator）、手动控制（Manual）、障碍物地图（ObstacleMap）；
    /// - 对 UI 暴露“控制接口 + 快照接口”，UI 不直接接触底层执行器细节；
    /// - 通过一把共享锁 <see cref="_robotLock"/> 保护所有机器人状态的一致性（位置/速度/加速度/转向/指令等）。
    /// 
    /// 说明：
    /// - Tick()：先逻辑层调度指令，再物理层积分更新位置与朝向。
    /// </summary>
    internal sealed class RobotEngine
    {
        private EnumRobotProcessState _processState = EnumRobotProcessState.Idle;
        private readonly object _robotLock = new object();

        private readonly RobotManager _robotManager;
        private readonly RobotMove _robotMove;
        private readonly RobotAutoNavigator _robotAutoNavigator;
        private readonly RobotManual _robotManual;
        private readonly ObstacleMap _obstacleMap;

        private readonly double _cellSizeM;
        private readonly double _dt;
        private readonly int _gridCount;

        private readonly double _worldWidthM;
        private readonly double _worldHeightM;

        // 机器人状态（内存中）
        private double _robotX;
        private double _robotY;
        private double _robotSpeed;
        private double _robotAcc;

        public RobotEngine(int gridCount, double cellSizeM, double dt,
            double initialMaxSpeed, EnumMoveDirection initialDirection)
        {
            _gridCount = gridCount;
            _cellSizeM = cellSizeM;
            _dt = dt;
            _worldWidthM = gridCount * cellSizeM;
            _worldHeightM = gridCount * cellSizeM;

            // 初始在(0,0)格中心
            _robotX = cellSizeM / 2.0;
            _robotY = cellSizeM / 2.0;
            _robotSpeed = 0;
            _robotAcc = 0;

            _robotManager = new RobotManager(
                acc: _robotAcc,
                maxSpeed: initialMaxSpeed,
                direction: initialDirection);

            _robotMove = new RobotMove(
                robotLock: _robotLock,
                getRobotX: () => _robotX,
                setRobotX: x => _robotX = x,
                getRobotY: () => _robotY,
                setRobotY: y => _robotY = y,
                getRobotSpeed: () => _robotSpeed,
                setRobotSpeed: v => _robotSpeed = v,
                robotManager: _robotManager,
                getWorldWidthM: () => _worldWidthM,
                getWorldHeightM: () => _worldHeightM,
                cellSizeM: _cellSizeM,
                dt: _dt,
                isWorldWalkable: (wx, wy) =>
                {
                    int gx = (int)Math.Floor(wx / _cellSizeM);
                    int gy = (int)Math.Floor(wy / _cellSizeM);

                    if (gx < 0 || gy < 0 || gx >= _gridCount || gy >= _gridCount)
                    {
                        return false;
                    }

                    return !_obstacleMap.IsObstacle(new GridPos(gx, gy));
                }
            );

            _obstacleMap = new ObstacleMap(gridCount, gridCount);

            _robotAutoNavigator = new RobotAutoNavigator(
                robotLock: _robotLock,
                getRobotX: () => _robotX,
                getRobotY: () => _robotY,
                setRobotSpeed: v => _robotSpeed = v, // 兼容旧构造参数
                getWorldWidthM: () => _worldWidthM,
                getWorldHeightM: () => _worldHeightM,
                cellSizeM: _cellSizeM,
                robotManager: _robotManager);

            _robotAutoNavigator.SetIsWalkableProvider(p => !_obstacleMap.IsObstacle(p));

            _robotManual = new RobotManual(
                robotLock: _robotLock,
                robotManager: _robotManager);

            // Robot 绑定运行时（自动指令源接入）
            _robotManager.BindRuntime(
                robotLock: _robotLock,
                move: _robotMove,
                turn: _robotMove.TurnController,
                autoCommandProvider: () => _robotAutoNavigator.TryBuildNextCommand(),
                getForwardAcc: () => _robotAcc);

            _robotManager.BindManualCommandProvider(() => _robotManual.TryBuildNextCommand());

            // 默认进入自动导航流程（也可以先 Idle，等 UI 触发）
            _processState = EnumRobotProcessState.AutoNavigating;
            _robotManager.SetMode(EnumRobotControlMode.Auto);
            _robotAutoNavigator.Enable();
        }

        #region 公共属性/方法（供 UI 调用）

        public double WorldWidthM => _worldWidthM;
        public double WorldHeightM => _worldHeightM;
        public double CellSizeM => _cellSizeM;
        public int GridCount => _gridCount;
        public RobotAutoNavigator AutoNavigator => _robotAutoNavigator;

        public EnumPathfindingAlgorithm Algorithm
        {
            get => _robotAutoNavigator.Algorithm;
            set => _robotAutoNavigator.Algorithm = value;
        }

        public bool AutoEnabled => _robotAutoNavigator.IsEnabled;

        /// <summary>
        /// 改变流程状态：
        /// </summary>
        /// <param name="newState"></param>
        private void ChangeProcessState(EnumRobotProcessState newState)
        {
            lock (_robotLock)
            {
                if (_processState == newState)
                {
                    return;
                }

                // 离开旧状态时的清理
                switch (_processState)
                {
                    case EnumRobotProcessState.AutoNavigating:
                        _robotAutoNavigator.Disable();
                        _robotManager.ResetAutoCommands();
                        _robotMove.StopImmediately_NoLock();
                        break;

                    case EnumRobotProcessState.ManualControl:
                        _robotManual.Disable();
                        _robotMove.StopImmediately_NoLock();
                        break;

                    case EnumRobotProcessState.ObstacleEditing:
                        // ObstacleEditing 离开时，不需要额外清理
                        break;

                    case EnumRobotProcessState.Error:
                        // Error 离开由外部调用 Reset 之类来处理
                        break;

                    case EnumRobotProcessState.Idle:
                        break;
                }

                _processState = newState;

                // 进入新状态时的初始化
                switch (newState)
                {
                    case EnumRobotProcessState.AutoNavigating:
                        _robotManager.SetMode(EnumRobotControlMode.Auto);
                        _robotAutoNavigator.Enable();
                        _robotAutoNavigator.RebuildPath();
                        _robotManager.ResetAutoCommands();
                        _robotManager.AlignOrientationToDirectionWithTurn();
                        break;

                    case EnumRobotProcessState.ManualControl:
                        _robotManager.SetMode(EnumRobotControlMode.Manual);
                        _robotManual.Enable();
                        break;

                    case EnumRobotProcessState.ObstacleEditing:
                        // 进入障碍编辑：立即停车 + 关闭自动导航
                        _robotSpeed = 0.0;
                        _robotManager.Acc = 0.0;
                        _robotMove.StopImmediately_NoLock();

                        if (_robotAutoNavigator.IsEnabled)
                        {
                            _robotAutoNavigator.Disable();
                        }
                        _robotManager.ResetAutoCommands();
                        break;

                    case EnumRobotProcessState.Idle:
                        _robotManager.SetMode(EnumRobotControlMode.Auto);
                        _robotAutoNavigator.Disable();
                        _robotMove.StopImmediately_NoLock();
                        break;

                    case EnumRobotProcessState.Error:
                        _robotMove.StopImmediately_NoLock();
                        _robotAutoNavigator.Disable();
                        _robotManual.Disable();
                        _robotManager.ResetAutoCommands();
                        break;
                }
            }
        }

        /// <summary>
        /// 障碍物编辑模式开关：
        /// - 开启：暂停自动导航 + 清空自动指令 + 立即停车；
        /// - 关闭：恢复自动导航 + 重规划路径 + 刷新自动指令，下一帧开始运动。
        /// </summary>
        public void SetObstacleEditMode(bool enabled)
        {
            lock (_robotLock)
            {
                if (enabled)
                {
                    ChangeProcessState(EnumRobotProcessState.ObstacleEditing);
                }
                else
                {
                    // 退出编辑：恢复自动导航 + 重规划路径
                    ChangeProcessState(EnumRobotProcessState.AutoNavigating);
                    _robotAutoNavigator.RebuildPath();
                    _robotManager.ResetAutoCommands();
                }
            }
        }

        /// <summary>
        /// 切换到自动模式：
        /// - Manager 置为 Auto（清理队列/停车/同步角度等）；
        /// - AutoNavigator Enable；并清空目标等待 UI 重新选择；
        /// - ResetAutoCommands：保证下一帧会从 provider 重新拉取指令；
        /// - 禁用手动控制器。
        /// </summary>
        public void EnableAuto()
        {
            lock (_robotLock)
            {
                ChangeProcessState(EnumRobotProcessState.AutoNavigating);

                // 切换到自动后清空目标：
                // - 避免沿用旧目标导致“模式切换后机器人突然跑走”
                // - 需要 UI 重新 SetGoal 才会开始规划/运动
                _robotAutoNavigator.ClearGoal();

                // 关键：清空队列/当前指令，让下一帧从 provider 拉取矫正队列里的指令
                _robotManager.ResetAutoCommands();

                // 切到自动后，让箭头通过转向动画对齐到最近的离散方向
                _robotManager.AlignOrientationToDirectionWithTurn();
            }
        }

        /// <summary>
        /// 切换到手动模式：
        /// - Manager 置为 Manual；
        /// - 禁用 AutoNavigator（不再产生命令）；
        /// - 启用 Manual（清理按键输入状态）。
        /// </summary>
        public void EnableManual()
        {
            lock (_robotLock)
            {
                ChangeProcessState(EnumRobotProcessState.ManualControl);
            }
        }

        /// <summary>
        /// 外部主动要求重建路径：用于障碍物变更/算法切换。
        /// </summary>
        public void RebuildPath()
        {
            _robotAutoNavigator.RebuildPath();
        }

        /// <summary>
        /// 获取路径点（世界坐标）快照：用于 UI 绘制路径线。
        /// </summary>
        public List<(double X, double Y)> GetPathWorldPointsSnapshot()
        {
            return _robotAutoNavigator.GetPathWorldPointsSnapshot();
        }

        /// <summary>
        /// 获取障碍物网格的快照：用于 UI 绘制障碍物。
        /// </summary>
        public bool[,] GetObstacleSnapshot()
        {
            return _obstacleMap.GetSnapshot();
        }

        /// <summary>
        /// 切换指定格子的障碍物状态：
        /// - Toggle 成为障碍（返回 true）时，若自动模式打开则触发重规划；
        /// - 取消障碍（返回 false）时，这里不触发重规划（当前逻辑仅在设置障碍时触发）。
        /// </summary>
        public void ToggleObstacle(GridPos p)
        {
            _obstacleMap.Toggle(p);

            // 设置障碍物模式：只改地图，不重规划；退出模式时再统一重规划一次
            if (_processState == EnumRobotProcessState.ObstacleEditing)
            {
                return;
            }

            if (_robotAutoNavigator.IsEnabled)
            {
                _robotAutoNavigator.RebuildPath();
            }
        }

        /// <summary>
        /// 清空所有障碍物。
        /// </summary>
        public void ClearObstacles()
        {
            _obstacleMap.Clear();

            // 设置障碍物模式：只改地图，不重规划；退出模式时再统一重规划一次
            if (_processState == EnumRobotProcessState.ObstacleEditing)
            {
                return;
            }

            if (_robotAutoNavigator.IsEnabled)
            {
                _robotAutoNavigator.RebuildPath();
            }
        }

        /// <summary>
        /// 设置前进加速度参数：
        /// - 自动模式 MoveDistance 下发时会读取该值；
        /// - 手动模式 W 按住时每帧 Tick 也会读取该值。
        /// </summary>
        public void SetForwardAcc(double acc)
        {
            lock (_robotLock)
            {
                _robotAcc = acc;
            }
        }

        /// <summary>
        /// 设置最大速度：由 RobotMove.Update 在每帧积分后进行夹紧。
        /// </summary>
        public void SetMaxSpeed(double vmax)
        {
            lock (_robotLock)
            {
                _robotManager.MaxSpeed = vmax;
            }
        }

        // 手动控制输入
        public void ManualForwardKey(bool down)
        {
            _robotManual.InputForwardKey(down);
            _robotManager.ResetManualCommands();
        }

        public void ManualTurnLeftKey(bool down)
        {
            _robotManual.InputTurnLeftKey(down);
            _robotManager.ResetManualCommands();
        }

        public void ManualTurnRightKey(bool down)
        {
            _robotManual.InputTurnRightKey(down);
            _robotManager.ResetManualCommands();
        }

        /// <summary>
        /// 仿真步进：
        /// - ① 逻辑层：生成/调度指令（自动/手动）；
        /// - ② 物理层：根据 Acc/Speed/方向，积分更新位置和转向动画。
        /// </summary>
        public void Tick()
        {
            // 根据流程状态做时间片调度
            switch (_processState)
            {
                case EnumRobotProcessState.ObstacleEditing:
                    // 编辑模式下：不推进逻辑/物理，只靠 UI 重绘
                    return;

                case EnumRobotProcessState.Idle:
                    // 空闲状态：也不推进逻辑/物理
                    return;

                case EnumRobotProcessState.Error:
                    // 错误状态：停机不动
                    return;

                case EnumRobotProcessState.AutoNavigating:
                case EnumRobotProcessState.ManualControl:
                    // ① 逻辑：调度指令（自动/手动都通过 RobotManager 管）
                    _robotManager.Tick(_dt, () => _robotAcc);

                    // ② 物理：根据指令与加速度、速度等积分
                    _robotMove.Update();
                    break;
            }
        }

        /// <summary>
        /// UI 绘制/文本显示用的状态快照：
        /// - 在 lock 内复制一份值类型快照，UI 可在锁外安全读。
        /// </summary>
        public RobotStateSnapshot GetStateSnapshot()
        {
            lock (_robotLock)
            {
                return new RobotStateSnapshot(
                    X: _robotX,
                    Y: _robotY,
                    Speed: _robotSpeed,
                    Acc: _robotAcc,
                    OrientationAngle: _robotManager.OrientationAngle);
            }
        }

        #endregion
    }

    /// <summary>
    /// 机器人状态快照（值类型）：用于 UI/绘制读取。
    /// </summary>
    internal readonly struct RobotStateSnapshot
    {
        /// <summary>
        /// 构造快照：一次性拷贝当前帧需要展示的状态。
        /// </summary>
        public RobotStateSnapshot(double X, double Y, double Speed, double Acc, double OrientationAngle)
        {
            this.X = X;
            this.Y = Y;
            this.Speed = Speed;
            this.Acc = Acc;
            this.OrientationAngle = OrientationAngle;
        }

        public double X { get; }
        public double Y { get; }
        public double Speed { get; }
        public double Acc { get; }
        public double OrientationAngle { get; }
    }
}