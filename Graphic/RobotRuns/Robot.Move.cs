using System;

namespace GridDemo.RobotRuns
{
    internal class RobotMove
    {
        private readonly object _robotLock;
        private readonly Func<double> _getRobotX;
        private readonly Action<double> _setRobotX;
        private readonly Func<double> _getRobotY;
        private readonly Action<double> _setRobotY;
        private readonly Func<double> _getRobotSpeed;
        private readonly Action<double> _setRobotSpeed;
        private readonly Robot _robot;
        private readonly Func<double> _getWorldWidthM;
        private readonly Func<double> _getWorldHeightM;
        private readonly double _cellSizeM;
        private readonly double _dt;

        private readonly RobotTurn _turnController;

        // 指令执行状态：MoveDistance
        private bool _moveDistanceActive;
        private double _moveDistanceRemainM;

        public RobotMove(
            object robotLock,
            Func<double> getRobotX,
            Action<double> setRobotX,
            Func<double> getRobotY,
            Action<double> setRobotY,
            Func<double> getRobotSpeed,
            Action<double> setRobotSpeed,
            Robot robot,
            Func<double> getWorldWidthM,
            Func<double> getWorldHeightM,
            double cellSizeM,
            double dt)
        {
            _robotLock = robotLock ?? throw new ArgumentNullException(nameof(robotLock));
            _getRobotX = getRobotX ?? throw new ArgumentNullException(nameof(getRobotX));
            _setRobotX = setRobotX ?? throw new ArgumentNullException(nameof(setRobotX));
            _getRobotY = getRobotY ?? throw new ArgumentNullException(nameof(getRobotY));
            _setRobotY = setRobotY ?? throw new ArgumentNullException(nameof(setRobotY));
            _getRobotSpeed = getRobotSpeed ?? throw new ArgumentNullException(nameof(getRobotSpeed));
            _setRobotSpeed = setRobotSpeed ?? throw new ArgumentNullException(nameof(setRobotSpeed));
            _robot = robot ?? throw new ArgumentNullException(nameof(robot));
            _getWorldWidthM = getWorldWidthM ?? throw new ArgumentNullException(nameof(getWorldWidthM));
            _getWorldHeightM = getWorldHeightM ?? throw new ArgumentNullException(nameof(getWorldHeightM));
            _cellSizeM = cellSizeM;
            _dt = dt;

            _turnController = new RobotTurn(_robotLock, _robot, this, _dt);
        }

        public RobotTurn TurnController
        {
            get { return _turnController; }
        }

        public void StopForTurn()
        {
            lock (_robotLock)
            {
                _setRobotSpeed(0.0);
                _robot.Acc = 0.0;

                // 转向会打断前进指令
                _moveDistanceActive = false;
                _moveDistanceRemainM = 0.0;
            }
        }

        public void StopImmediately_NoLock()
        {
            _setRobotSpeed(0.0);
            _robot.Acc = 0.0;
            _moveDistanceActive = false;
            _moveDistanceRemainM = 0.0;
        }

        /// <summary>
        /// 开始执行“前进位移”指令（米）。
        /// </summary>
        public void StartMoveDistance_NoLock(double distanceM, double forwardAcc)
        {
            if (distanceM <= 0)
            {
                _moveDistanceActive = false;
                _moveDistanceRemainM = 0.0;
                return;
            }

            // 转向中不允许开始移动
            if (_robot.IsTurning)
            {
                _moveDistanceActive = false;
                _moveDistanceRemainM = 0.0;
                return;
            }

            _robot.Acc = forwardAcc;
            _moveDistanceActive = true;
            _moveDistanceRemainM = distanceM;
        }

        public bool IsMoveDistanceDone_NoLock()
        {
            return !_moveDistanceActive;
        }

        /// <summary>
        /// 每帧更新运动学：仅在存在 MoveDistance 指令且不在转向时推进位移。
        /// </summary>
        public void Update()
        {
            lock (_robotLock)
            {
                // 先更新转向动画（转向期间不移动）
                // 注意：RobotTurn.Update 内部也 lock，同一把锁会死锁，所以这里不提前调用
                // 转向 Update 移到锁外执行

                if (_robot.IsTurning)
                {
                    // 转向期间不走位移
                }
                else if (_moveDistanceActive)
                {
                    double v = _getRobotSpeed();
                    double a = _robot.Acc;
                    double vmax = _robot.MaxSpeed;
                    double x = _getRobotX();
                    double y = _getRobotY();
                    EnumMoveDirection dir = _robot.Direction;

                    v += a * _dt;
                    if (v < 0) v = 0;
                    if (v > vmax) v = vmax;

                    double step = v * _dt;
                    if (step > _moveDistanceRemainM)
                    {
                        step = _moveDistanceRemainM;
                    }

                    switch (dir)
                    {
                        case EnumMoveDirection.Right:
                            x += step;
                            break;
                        case EnumMoveDirection.Left:
                            x -= step;
                            break;
                        case EnumMoveDirection.Down:
                            y += step;
                            break;
                        case EnumMoveDirection.Up:
                            y -= step;
                            break;
                    }

                    // 边界夹紧（保持原逻辑）
                    double worldWidth = _getWorldWidthM();
                    double worldHeight = _getWorldHeightM();
                    double halfCell = _cellSizeM / 2.0;

                    if (x < halfCell) x = halfCell;
                    if (y < halfCell) y = halfCell;
                    if (x > worldWidth - halfCell) x = worldWidth - halfCell;
                    if (y > worldHeight - halfCell) y = worldHeight - halfCell;

                    _moveDistanceRemainM -= step;

                    _setRobotSpeed(v);
                    _setRobotX(x);
                    _setRobotY(y);

                    if (_moveDistanceRemainM <= 0.000001)
                    {
                        _moveDistanceActive = false;
                        _moveDistanceRemainM = 0.0;
                        _setRobotSpeed(0.0);
                        _robot.Acc = 0.0;
                    }
                }
            }

            _turnController.Update();
        }
    }
}