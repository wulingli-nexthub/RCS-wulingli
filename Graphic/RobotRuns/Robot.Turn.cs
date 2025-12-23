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

                // 确保角度在 [0, 2π) 范围内计算
                cur = NormalizeAngle(cur);
                target = NormalizeAngle(target);

                // 计算最短旋转角度
                double delta = target - cur;

                // 选择最短旋转方向
                if (delta > Math.PI)
                {
                    delta -= 2 * Math.PI;
                }
                else if (delta < -Math.PI)
                {
                    delta += 2 * Math.PI;
                }

                if (Math.Abs(delta) <= maxStep)
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
                _robot.OrientationAngle = NormalizeAngle(cur + step);
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

                // 获取当前方向
                EnumMoveDirection currentDir = _robot.Direction;

                // 如果目标方向与当前方向相同，不需要转向
                if (currentDir == targetDirection)
                {
                    return;
                }

                double currentAngle = NormalizeAngle(_robot.OrientationAngle);
                double targetAngle = NormalizeAngle(Robot.DirectionToAngle(targetDirection));

                // 计算两个方向的旋转差值（选择最短路径）
                double delta = targetAngle - currentAngle;

                // 选择最短旋转方向
                if (delta > Math.PI)
                {
                    delta -= 2 * Math.PI;
                }
                else if (delta < -Math.PI)
                {
                    delta += 2 * Math.PI;
                }

                // 如果旋转角度很小，直接设置方向，不进行动画
                if (Math.Abs(delta) < 0.01) // 约0.57度
                {
                    _robot.OrientationAngle = targetAngle;
                    _robot.TargetOrientationAngle = targetAngle;
                    _robot.Direction = targetDirection;
                    return;
                }

                double finalTarget = currentAngle + delta;

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

                // 获取当前角度并归一化
                double currentAngle = NormalizeAngle(_robot.OrientationAngle);

                // 计算新角度并归一化
                double newTarget = NormalizeAngle(currentAngle + deltaAngle);

                _robot.TargetOrientationAngle = newTarget;
                _robot.IsTurning = true;
            }
        }

        private static double NormalizeAngle(double angle)
        {
            // 归一化到 [0, 2π) 范围，避免负角度带来的复杂性
            angle = angle % (2 * Math.PI);
            if (angle < 0) angle += 2 * Math.PI;
            return angle;
        }

        private static EnumMoveDirection AngleToDirection(double angle)
        {
            // 归一化到 [0, 2π)
            angle = NormalizeAngle(angle);

            // 简化逻辑：使用固定的45度边界来划分4个方向
            // 右方向: [337.5°, 22.5°)
            if (angle >= 15 * Math.PI / 8 || angle < Math.PI / 8)
            {
                return EnumMoveDirection.Right;
            }
            // 下方向: [22.5°, 112.5°)
            else if (angle >= Math.PI / 8 && angle < 5 * Math.PI / 8)
            {
                return EnumMoveDirection.Down;
            }
            // 左方向: [112.5°, 202.5°)
            else if (angle >= 5 * Math.PI / 8 && angle < 9 * Math.PI / 8)
            {
                return EnumMoveDirection.Left;
            }
            // 上方向: [202.5°, 292.5°)
            else if (angle >= 9 * Math.PI / 8 && angle < 13 * Math.PI / 8)
            {
                return EnumMoveDirection.Up;
            }
            // 右方向: [292.5°, 337.5°)
            else
            {
                return EnumMoveDirection.Right;
            }
        }
    }
}