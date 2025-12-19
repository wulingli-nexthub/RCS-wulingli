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

                // 自动模式：保持“当前方向”开始直行（满足“切到自动先沿当前方向走到边界”）
                _robot.IsForwardKeyDown = true;

                // 修复“箭头与运动方向不一致”：同步角度到当前离散方向
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

        /// <summary>
        /// 给 RobotMove 使用：自动模式下每帧提供“应该怎么走”
        /// </summary>
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

                EnumMoveDirection dir = _robot.Direction;

                // 下一步是否会越界：越界则触发“原地左转”
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
                    // 到边界：停住 + 请求左转（RobotMove 会调用 TurnController.StartTurnLeft）
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

                // 未到边界：持续前进
                double acc = getForwardAcc();
                _robot.IsForwardKeyDown = true;
                _robot.Acc = acc;

                // 自动模式下直接同步角度，保证箭头与运动严格一致
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