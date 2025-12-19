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

        private readonly Robot _robot;

        private readonly Func<double> _getWorldWidthM;
        private readonly Func<double> _getWorldHeightM;

        private readonly double _cellSizeM;
        private readonly double _dt;

        private readonly RobotTurn _turnController;

        // 数值框配置的“前进加速度”
        private readonly Func<double> _getForwardAcc;

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
            Func<double> getForwardAcc)
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

            _turnController = new RobotTurn(_robotLock, _robot, this, _dt);
        }

        public RobotTurn TurnController
        {
            get { return _turnController; }
        }

        /// <summary>
        /// 原地转向开始前调用：停止速度和加速。
        /// </summary>
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

                // 使用 Form1 传进来的“前进加速度”
                double forwardAcc = _getForwardAcc();
                _robot.Acc = forwardAcc;
            }
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

                // 若正在原地转向，不移动位置，只更新角度（在 turn update 里）
                if (!_robot.IsTurning)
                {
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

                    // 边界转向保持原有逻辑
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

                    _setRobotSpeed(v);
                    _setRobotX(x);
                    _setRobotY(y);
                    _robot.Direction = dir;
                }
            }

            // 无论是否在转向，都需要更新角度动画
            _turnController.Update();
        }
    }
}