using System;

namespace Graphic.RobotRuns
{
    internal sealed class RobotAutoMotionState
    {
        public static readonly RobotAutoMotionState Disabled = new RobotAutoMotionState(
            enabled: false,
            direction: EnumMoveDirection.Right,
            acc: 0.0,
            suppressEdgeTurning: false,
            clampOnBounds: false,
            requestTurnLeft: false,
            requestTurnToDirection: null);

        public RobotAutoMotionState(
            bool enabled,
            EnumMoveDirection direction,
            double acc,
            bool suppressEdgeTurning,
            bool clampOnBounds,
            bool requestTurnLeft,
            EnumMoveDirection? requestTurnToDirection)
        {
            Enabled = enabled;
            Direction = direction;
            Acc = acc;
            SuppressEdgeTurning = suppressEdgeTurning;
            ClampOnBounds = clampOnBounds;
            RequestTurnLeft = requestTurnLeft;
            RequestTurnToDirection = requestTurnToDirection;
        }

        public bool Enabled { get; }
        public EnumMoveDirection Direction { get; }
        public double Acc { get; }
        public bool SuppressEdgeTurning { get; }
        public bool ClampOnBounds { get; }
        public bool RequestTurnLeft { get; }
        public EnumMoveDirection? RequestTurnToDirection { get; }
    }

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

        private readonly Func<double> _getForwardAcc;

        private readonly Func<RobotAutoMotionState> _getAutoMotionState;

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
            double dt,
            Func<double> getForwardAcc,
            Func<RobotAutoMotionState> getAutoMotionState)
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
            _getForwardAcc = getForwardAcc ?? throw new ArgumentNullException(nameof(getForwardAcc));
            _getAutoMotionState = getAutoMotionState ?? throw new ArgumentNullException(nameof(getAutoMotionState));

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
            }
        }

        public void ResumeForwardAfterTurn()
        {
            lock (_robotLock)
            {
                if (!_robot.IsForwardKeyDown)
                {
                    return;
                }

                double forwardAcc = _getForwardAcc();
                _robot.Acc = forwardAcc;
            }
        }

        public void Update()
        {
            RobotAutoMotionState autoState = _getAutoMotionState();

            lock (_robotLock)
            {
                if (autoState.Enabled)
                {
                    // 自动模式：方向/加速度由 autoState 控制
                    _robot.Acc = autoState.Acc;

                    if (autoState.RequestTurnLeft && !_robot.IsTurning)
                    {
                        _turnController.StartTurnLeft();
                    }

                    if (autoState.RequestTurnToDirection.HasValue && !_robot.IsTurning)
                    {
                        _turnController.StartTurnTo(autoState.RequestTurnToDirection.Value);
                    }
                }

                double v = _getRobotSpeed();
                double a = _robot.Acc;
                double vmax = _robot.MaxSpeed;
                double x = _getRobotX();
                double y = _getRobotY();
                EnumMoveDirection dir = _robot.Direction;

                if (!_robot.IsTurning)
                {
                    v += a * _dt;
                    if (v < 0) v = 0;
                    if (v > vmax) v = vmax;

                    switch (dir)
                    {
                        case EnumMoveDirection.Right:
                            x += v * _dt;
                            break;
                        case EnumMoveDirection.Left:
                            x -= v * _dt;
                            break;
                        case EnumMoveDirection.Down:
                            y += v * _dt;
                            break;
                        case EnumMoveDirection.Up:
                            y -= v * _dt;
                            break;
                    }

                    double worldWidth = _getWorldWidthM();
                    double worldHeight = _getWorldHeightM();
                    double halfCell = _cellSizeM / 2.0;

                    // 关键：运动时吸附到网格中心线，避免沿网格线（边界）滑行
                    // 水平运动锁定 y 到当前格子中心；垂直运动锁定 x 到当前格子中心
                    if (dir == EnumMoveDirection.Right || dir == EnumMoveDirection.Left)
                    {
                        int gy = (int)System.Math.Floor((y - halfCell) / _cellSizeM);
                        if (gy < 0) gy = 0;
                        int maxGy = (int)System.Math.Floor((worldHeight - halfCell) / _cellSizeM);
                        if (gy > maxGy) gy = maxGy;

                        y = gy * _cellSizeM + halfCell;
                    }
                    else if (dir == EnumMoveDirection.Up || dir == EnumMoveDirection.Down)
                    {
                        int gx = (int)System.Math.Floor((x - halfCell) / _cellSizeM);
                        if (gx < 0) gx = 0;
                        int maxGx = (int)System.Math.Floor((worldWidth - halfCell) / _cellSizeM);
                        if (gx > maxGx) gx = maxGx;

                        x = gx * _cellSizeM + halfCell;
                    }

                    if (autoState.Enabled && autoState.ClampOnBounds)
                    {
                        if (x < halfCell) x = halfCell;
                        if (y < halfCell) y = halfCell;
                        if (x > worldWidth - halfCell) x = worldWidth - halfCell;
                        if (y > worldHeight - halfCell) y = worldHeight - halfCell;
                    }

                    // 夹紧,确保机器人在边界内运动
                    if (!(autoState.Enabled && autoState.SuppressEdgeTurning))
                    {
                        switch (dir)
                        {
                            case EnumMoveDirection.Right:
                                if (x >= worldWidth - halfCell)
                                {
                                    x = worldWidth - halfCell;
                                    v = 0.0;
                                }
                                break;

                            case EnumMoveDirection.Down:
                                if (y >= worldHeight - halfCell)
                                {
                                    y = worldHeight - halfCell;
                                    v = 0.0;
                                }
                                break;

                            case EnumMoveDirection.Left:
                                if (x <= 0.0 + halfCell)
                                {
                                    x = halfCell;
                                    v = 0.0;
                                }
                                break;

                            case EnumMoveDirection.Up:
                                if (y <= 0.0 + halfCell)
                                {
                                    y = halfCell;
                                    v = 0.0;
                                }
                                break;
                        }
                    }

                    _setRobotSpeed(v);
                    _setRobotX(x);
                    _setRobotY(y);
                    _robot.Direction = dir;
                }
            }

            _turnController.Update();
        }
    }
}