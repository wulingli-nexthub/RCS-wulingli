using System;

namespace Graphic.RobotRuns
{
    internal class RobotTurn
    {
        private readonly object _robotLock;
        private readonly Robot _robot;
        private readonly RobotMove _move;
        private readonly double _dt;

        public RobotTurn(object robotLock, Robot robot, RobotMove move, double dt)
        {
            _robotLock = robotLock ?? throw new ArgumentNullException(nameof(robotLock));
            _robot = robot ?? throw new ArgumentNullException(nameof(robot));
            _move = move ?? throw new ArgumentNullException(nameof(move));
            _dt = dt;
        }

        /// <summary>
        /// 每一帧由 RobotMove 调用，更新转向动画。
        /// </summary>
        public void Update()
        {
            lock (_robotLock)
            {
                if (!_robot.IsTurning)
                {
                    return;
                }

                double cur = _robot.OrientationAngle;
                double target = _robot.TargetOrientationAngle;
                double maxStep = _robot.TurnAngularSpeed * _dt;

                double delta = NormalizeAngle(target - cur);

                if (System.Math.Abs(delta) <= maxStep)
                {
                    // 旋转完成
                    _robot.OrientationAngle = target;
                    _robot.IsTurning = false;

                    // 根据最终角度更新离散方向
                    _robot.Direction = AngleToDirection(target);

                    // 若前进键当前处于按下状态，恢复加速度让其继续前进
                    if (_robot.IsForwardKeyDown)
                    {
                        _move.ResumeForwardAfterTurn();
                    }

                    return;
                }

                double step = delta > 0 ? maxStep : -maxStep;
                _robot.OrientationAngle = cur + step;
            }
        }

        /// <summary>
        /// 开始一次 90° 左转
        /// </summary>
        public void StartTurnLeft()
        {
            StartTurnInternal(-System.Math.PI / 2.0);
        }

        /// <summary>
        /// 开始一次 90° 右转
        /// </summary>
        public void StartTurnRight()
        {
            StartTurnInternal(System.Math.PI / 2.0);
        }

        public void StartTurnTo(EnumMoveDirection targetDirection)
        {
            lock (_robotLock)
            {
                if (_robot.IsTurning)
                {
                    return;
                }

                double targetAngle = Robot.DirectionToAngle(targetDirection);

                // 以当前朝向为准，生成“等价目标角”（避免从 +pi 转到 -pi 产生大角度旋转）
                double cur = _robot.OrientationAngle;
                double delta = NormalizeAngle(targetAngle - cur);
                double finalTarget = cur + delta;

                // 原地转向：立即把速度清零并停止加速
                _move.StopForTurn();

                _robot.TargetOrientationAngle = finalTarget;
                _robot.IsTurning = true;
            }
        }

        private void StartTurnInternal(double deltaAngle)
        {
            lock (_robotLock)
            {
                if (_robot.IsTurning)
                {
                    // 正在转向中，忽略新的命令（也可以排队，这里简单处理）
                    return;
                }

                // 原地转向：立即把速度清零并停止加速
                _move.StopForTurn();

                // 以当前 "目标角" 为基准叠加 90°
                double curTarget = _robot.TargetOrientationAngle;
                _robot.TargetOrientationAngle = curTarget + deltaAngle;
                _robot.IsTurning = true;
            }
        }

        private static double NormalizeAngle(double angle)
        {
            while (angle > System.Math.PI) angle -= 2 * System.Math.PI;
            while (angle < -System.Math.PI) angle += 2 * System.Math.PI;
            return angle;
        }

        private static EnumMoveDirection AngleToDirection(double angle)
        {
            // 将角度归一化到 [-pi, pi]
            angle = NormalizeAngle(angle);

            // 按象限粗略映射到 4 个方向
            if (angle >= -System.Math.PI / 4 && angle < System.Math.PI / 4)
            {
                return EnumMoveDirection.Right;
            }

            if (angle >= System.Math.PI / 4 && angle < 3 * System.Math.PI / 4)
            {
                return EnumMoveDirection.Down;
            }

            if (angle <= -System.Math.PI / 4 && angle > -3 * System.Math.PI / 4)
            {
                return EnumMoveDirection.Up;
            }

            return EnumMoveDirection.Left;
        }
    }
}