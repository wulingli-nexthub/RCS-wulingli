using Graphic.Maps;
using GridDemo.RobotModels;
using GridDemo.RobotModels.Pathfinding;
using GridDemo.RobotRuns;
using System;
using System.Collections.Generic;

namespace GridDemo.Core
{
    /// <summary>
    /// 封装机器人运动/寻路/障碍物等业务，不依赖任何 UI。
    /// UI 通过本类提供的接口/快照进行交互。
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
                getCellSizeM: () => _cellSizeM,
                robotManager: _robotManager);

            // Robot 绑定运行时（自动指令源接入）
            _robotManager.BindRuntime(
                robotLock: _robotLock,
                move: _robotMove,
                turn: _robotMove.TurnController,
                autoCommandProvider: () => _robotAutoNavigator.TryBuildNextCommand());
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

        public void EnableAuto()
        {
            lock (_robotLock)
            {
                _robotManager.SetMode(EnumRobotControlMode.Auto);
                _robotAutoNavigator.Enable();
                _robotManual.Disable();
            }
        }

        public void EnableManual()
        {
            lock (_robotLock)
            {
                _robotManager.SetMode(EnumRobotControlMode.Manual);
                _robotAutoNavigator.Disable();
                _robotManual.Enable();
            }
        }

        public void SetGoal(GridPos gridGoal)
        {
            _robotAutoNavigator.SetGoal(gridGoal, rebuildIfEnabled: true);
        }

        public void RebuildPath()
        {
            _robotAutoNavigator.RebuildPath();
        }

        public List<(double X, double Y)> GetPathWorldPointsSnapshot()
        {
            return _robotAutoNavigator.GetPathWorldPointsSnapshot();
        }

        public bool[,] GetObstacleSnapshot()
        {
            return _obstacleMap.GetSnapshot();
        }

        public void ToggleObstacle(GridPos p)
        {
            if (_obstacleMap.Toggle(p) && _robotAutoNavigator.IsEnabled)
            {
                _robotAutoNavigator.RebuildPath();
            }
        }

        public void SetForwardAcc(double acc)
        {
            lock (_robotLock)
            {
                _robotAcc = acc;
            }
        }

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
        /// UI 绘制和文本显示用的状态快照
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

    internal readonly struct RobotStateSnapshot
    {
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