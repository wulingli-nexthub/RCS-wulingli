using GridDemo.Robots;
using System;

namespace GridDemo.RobotRuns
{
    /// <summary>
    /// 机器人转向控制器：
    /// - 负责把“离散方向变化”（Left/Right/Up/Down）转换为“连续角度旋转动画”；
    /// - 通过 <see cref="Robot.OrientationAngle"/> 与 <see cref="Robot.TargetOrientationAngle"/> 驱动插值；
    /// - 与 <see cref="RobotMove"/> 协作：开始转向时停止平移，转向完成后（必要时）恢复加速度。
    /// </summary>
    internal class RobotTurn
    {
        private readonly object _robotLock;
        private readonly RobotManager _robotManager;
        private readonly RobotMove _move;
        private readonly double _dt;
        private bool _hasTargetAngle;

        public RobotTurn(object robotLock, RobotManager robotManager, RobotMove move, double dt)
        {
            _robotLock = robotLock ?? throw new ArgumentNullException(nameof(robotLock));
            _robotManager = robotManager ?? throw new ArgumentNullException(nameof(robotManager));
            _move = move ?? throw new ArgumentNullException(nameof(move));
            _dt = dt;
        }

        /// <summary>
        /// 每一帧由 <see cref="RobotMove.Update"/> 调用，推进转向动画：
        /// 1) 计算当前角到目标角的最短角差；
        /// 2) 按最大步长（角速度 * dt）逼近；
        /// 3) 到达目标后结束转向，并更新离散方向。
        /// </summary>
        public void Update()
        {
            lock (_robotLock)
            {
                if (!_robotManager.IsTurning)
                {
                    return;
                }

                double maxStep = _robotManager.TurnAngularSpeed * _dt;       // 每帧允许的最大旋转角度（弧度）

                if (_hasTargetAngle)
                {
                    double cur = NormalizeAngle(_robotManager.OrientationAngle);
                    double target = NormalizeAngle(_robotManager.TargetOrientationAngle);

                    double delta = target - cur;
                    if (delta > Math.PI) delta -= 2 * Math.PI;
                    else if (delta < -Math.PI) delta += 2 * Math.PI;

                    if (Math.Abs(delta) <= maxStep)
                    {
                        _robotManager.OrientationAngle = target;
                        _robotManager.IsTurning = false;
                        _hasTargetAngle = false;

                        _robotManager.Direction = AngleToDirection(target);
                        return;
                    }

                    double step = delta > 0 ? maxStep : -maxStep;
                    _robotManager.OrientationAngle = NormalizeAngle(cur + step);
                }
            }
        }

        // 统一入口：相对旋转 angleRad 弧度
        public void StartTurnByDelta(double angleRad)
        {
            lock (_robotLock)
            {
                if (Math.Abs(angleRad) < 1e-6)
                {
                    // 角度为 0 -> 立即停止转向
                    _robotManager.IsTurning = false;
                    _hasTargetAngle = false;
                    return;
                }

                // 正在转向时，你可以选择覆盖或忽略，这里用覆盖：
                _move.StopForTurn();   // 刹车后原地转

                double currentAngle = NormalizeAngle(_robotManager.OrientationAngle);
                double target = NormalizeAngle(currentAngle + angleRad);

                _robotManager.TargetOrientationAngle = target;
                _robotManager.IsTurning = true;
                _hasTargetAngle = true;
            }
        }

        public void ResetTargetAngle()
        {
            lock (_robotLock)
            {
                _hasTargetAngle = false;
            }
        }

        /// <summary>
        /// 将角度归一化到 [0, 2π)。
        /// 这样做能简化跨越 0/2π 边界时的比较与插值逻辑。
        /// </summary>
        private static double NormalizeAngle(double angle)
        {
            // 归一化到 [0, 2π) 范围，避免负角度带来的复杂性
            angle = angle % (2 * Math.PI);
            if (angle < 0)
            {
                angle += 2 * Math.PI;
            }
            return angle;
        }

        /// <summary>
        /// 将连续角度映射回离散方向。
        /// 通过 45° 分界把一圈切成四个象限，以容忍插值误差/浮点误差。
        /// </summary>
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