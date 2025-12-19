using System;

namespace Graphic.RobotRuns
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

        // 直接持有运动参数对象
        private readonly Robot _robot;

        private readonly Func<double> _getWorldWidthM;
        private readonly Func<double> _getWorldHeightM;

        private readonly double _cellSizeM;
        private readonly double _dt;

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
        }

        public void Update()
        {
            lock (_robotLock)
            {
                double v = _getRobotSpeed();
                double a = _robot.Acc;
                double vmax = _robot.MaxSpeed;
                double x = _getRobotX();
                double y = _getRobotY();
                EnumMoveDirection dir = _robot.Direction;

                // 速度更新
                v += a * _dt;
                if (v < 0) v = 0;
                if (v > vmax) v = vmax;

                // 根据方向更新位置
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

                // 边界转向
                switch (dir)
                {
                    case EnumMoveDirection.Right:
                        if (x >= worldWidth - halfCell)
                        {
                            x = worldWidth - halfCell;
                            dir = EnumMoveDirection.Down;
                        }
                        break;

                    case EnumMoveDirection.Down:
                        if (y >= worldHeight - halfCell)
                        {
                            y = worldHeight - halfCell;
                            dir = EnumMoveDirection.Left;
                        }
                        break;

                    case EnumMoveDirection.Left:
                        if (x <= 0.0 + halfCell)
                        {
                            x = halfCell;
                            dir = EnumMoveDirection.Up;
                        }
                        break;

                    case EnumMoveDirection.Up:
                        if (y <= 0.0 + halfCell)
                        {
                            y = halfCell;
                            dir = EnumMoveDirection.Right;
                        }
                        break;
                }

                // 写回
                _setRobotSpeed(v);
                _setRobotX(x);
                _setRobotY(y);
                _robot.Direction = dir;
            }
        }
    }
}
