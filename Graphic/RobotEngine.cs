using Graphic.Maps;
using GridDemo.RobotModels;
using GridDemo.RobotModels.Pathfinding;
using GridDemo.RobotRuns;
using System;
using System.Collections.Generic;

namespace GridDemo.Core
{
    /// <summary>
    /// 业务引擎（无 UI 依赖）：
    /// - 统一封装机器人运动学（Move/Turn）、自动寻路（AutoNavigator）、手动控制（Manual）、障碍物地图（ObstacleMap）；
    /// - 对 UI 暴露“控制接口 + 快照接口”，UI 不直接接触底层执行器细节；
    /// - 通过一把共享锁 <see cref="_robotLock"/> 保护所有机器人状态的一致性（位置/速度/加速度/转向/指令等）。
    /// </summary>
    internal sealed class RobotEngine
    {
        private readonly object _robotLock = new object();

        private readonly RobotManager _robotManager;
        private readonly RobotMove _robotMove;
        private readonly RobotAutoNavigator _robotAutoNavigator;
        private readonly RobotManual _robotManual;
        private readonly RobotMotionFacade _motionFacade;
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
                });

            _motionFacade = new RobotMotionFacade(
                robotLock: _robotLock,
                move: _robotMove,
                robotManager: _robotManager,
                dt: _dt,
                getForwardAcc: () => _robotAcc);

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
                _robotManager.SetMode(EnumRobotControlMode.Auto);
                _robotAutoNavigator.Enable();

                // 切换到自动后清空目标：
                // - 避免沿用旧目标导致“模式切换后机器人突然跑走”
                // - 需要 UI 重新 SetGoal 才会开始规划/运动
                _robotAutoNavigator.ClearGoal();

                // 关键：清空队列/当前指令，让下一帧从 provider 拉取矫正队列里的指令
                _robotManager.ResetAutoCommands();
                _robotManual.Disable();
            }
        }

        public void StartSerpentineToBottomRight()
        {
            lock (_robotLock)
            {
                _robotManager.SetMode(EnumRobotControlMode.Auto);

                // 先配置算法与目标（确保 Enable() 内的首次 RebuildPath 能拿到 goal）
                _robotAutoNavigator.Algorithm = EnumPathfindingAlgorithm.Serpentine;

                var goal = new GridPos(_gridCount - 1, _gridCount - 1);
                _robotAutoNavigator.SetGoal(goal, rebuildIfEnabled: false);

                // 再启用（Enable 内会 RebuildPath_NoLock，并生成对齐队列/路径）
                _robotAutoNavigator.Enable();

                // 保证下一帧必定从 provider 拉取最新的对齐/蛇形指令
                _robotManager.ResetAutoCommands();
                _robotManual.Disable();
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
                _robotManager.SetMode(EnumRobotControlMode.Manual);
                _robotAutoNavigator.Disable();
                _robotManual.Enable();
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
            if (_obstacleMap.Toggle(p) && _robotAutoNavigator.IsEnabled)
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
        public void ManualForwardKey(bool down) => _robotManager.InputManualForwardKey(down);
        public void ManualTurnLeftKey(bool down) => _robotManager.InputManualTurnLeftKey(down);
        public void ManualTurnRightKey(bool down) => _robotManager.InputManualTurnRightKey(down);

        /// <summary>
        /// 仿真步进（供后台线程调用）
        /// </summary>
        public void Tick()
        {
            _motionFacade.Update();
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