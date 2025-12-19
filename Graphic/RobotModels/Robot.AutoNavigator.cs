using Graphic.RobotRuns;
using System;

namespace Graphic.RobotModels
{
    internal sealed class RobotAutoNavigator
    {
        private readonly object _robotLock;
        private readonly Func<double> _getRobotX;
        private readonly Func<double> _getRobotY;
        private readonly Action<double> _setRobotSpeed;

        private readonly Func<double> _getWorldWidthM;
        private readonly Func<double> _getWorldHeightM;

        private readonly double _cellSizeM;
        private readonly Robot _robot;

        private const double ArriveEpsilonM = 0.02;

        public RobotAutoNavigator(
            object robotLock,
            Func<double> getRobotX,
            Func<double> getRobotY,
            Action<double> setRobotSpeed,
            Func<double> getWorldWidthM,
            Func<double> getWorldHeightM,
            double cellSizeM,
            Robot robot)
        {
            _robotLock = robotLock ?? throw new ArgumentNullException(nameof(robotLock));
            _getRobotX = getRobotX ?? throw new ArgumentNullException(nameof(getRobotX));
            _getRobotY = getRobotY ?? throw new ArgumentNullException(nameof(getRobotY));
            _setRobotSpeed = setRobotSpeed ?? throw new ArgumentNullException(nameof(setRobotSpeed));
            _getWorldWidthM = getWorldWidthM ?? throw new ArgumentNullException(nameof(getWorldWidthM));
            _getWorldHeightM = getWorldHeightM ?? throw new ArgumentNullException(nameof(getWorldHeightM));
            _cellSizeM = cellSizeM;
            _robot = robot ?? throw new ArgumentNullException(nameof(robot));
        }

        public bool IsEnabled { get; private set; }

        public void Enable()
        {
            lock (_robotLock)
            {
                IsEnabled = true;

                _robot.IsForwardKeyDown = true;

                double angle = Robot.DirectionToAngle(_robot.Direction);
                _robot.OrientationAngle = angle;
                _robot.TargetOrientationAngle = angle;
            }
        }

        public void Disable()
        {
            lock (_robotLock)
            {
                IsEnabled = false;

                _robot.IsForwardKeyDown = false;
                _robot.Acc = 0.0;
                _setRobotSpeed(0.0);
            }
        }

        public RobotAutoMotionState GetAutoMotionState(Func<double> getForwardAcc)
        {
            if (getForwardAcc == null) throw new ArgumentNullException(nameof(getForwardAcc));

            lock (_robotLock)
            {
                if (!IsEnabled)
                {
                    return RobotAutoMotionState.Disabled;
                }

                double worldWidth = _getWorldWidthM();
                double worldHeight = _getWorldHeightM();
                double halfCell = _cellSizeM / 2.0;

                double x = _getRobotX();
                double y = _getRobotY();

                // ② 到右下角自动停止（右下角 cell 中心点）
                double targetX = worldWidth - halfCell;
                double targetY = worldHeight - halfCell;

                bool arriveX = Math.Abs(x - targetX) <= ArriveEpsilonM;
                bool arriveY = Math.Abs(y - targetY) <= ArriveEpsilonM;

                if (arriveX && arriveY)
                {
                    _robot.IsForwardKeyDown = false;
                    _robot.Acc = 0.0;
                    _setRobotSpeed(0.0);

                    return new RobotAutoMotionState(
                        enabled: true,
                        direction: _robot.Direction,
                        acc: 0.0,
                        suppressEdgeTurning: true,
                        clampOnBounds: true,
                        requestTurnLeft: false);
                }

                EnumMoveDirection dir = _robot.Direction;

                bool willHitBoundary;
                switch (dir)
                {
                    case EnumMoveDirection.Right:
                        willHitBoundary = x >= worldWidth - halfCell;
                        break;
                    case EnumMoveDirection.Down:
                        willHitBoundary = y >= worldHeight - halfCell;
                        break;
                    case EnumMoveDirection.Left:
                        willHitBoundary = x <= halfCell;
                        break;
                    case EnumMoveDirection.Up:
                        willHitBoundary = y <= halfCell;
                        break;
                    default:
                        willHitBoundary = false;
                        break;
                }

                if (willHitBoundary)
                {
                    _robot.IsForwardKeyDown = true;
                    _robot.Acc = 0.0;
                    _setRobotSpeed(0.0);

                    return new RobotAutoMotionState(
                        enabled: true,
                        direction: dir,
                        acc: 0.0,
                        suppressEdgeTurning: false,
                        clampOnBounds: true,
                        requestTurnLeft: true);
                }

                double acc = getForwardAcc();
                _robot.IsForwardKeyDown = true;
                _robot.Acc = acc;

                double angle2 = Robot.DirectionToAngle(dir);
                _robot.OrientationAngle = angle2;
                _robot.TargetOrientationAngle = angle2;

                return new RobotAutoMotionState(
                    enabled: true,
                    direction: dir,
                    acc: acc,
                    suppressEdgeTurning: true,
                    clampOnBounds: true,
                    requestTurnLeft: false);
            }
        }
    }
}