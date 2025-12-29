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

                double cur = _robotManager.OrientationAngle;
                double target = _robotManager.TargetOrientationAngle;
                double maxStep = _robotManager.TurnAngularSpeed * _dt;       // 每帧允许的最大旋转角度（弧度）

                // 确保角度在 [0, 2π) 范围内计算
                cur = NormalizeAngle(cur);
                target = NormalizeAngle(target);

                // 计算最短旋转角度
                double delta = target - cur;

                if (delta > Math.PI)
                { // 选择最短旋转方向：把差值映射到 (-π, π]
                    delta -= 2 * Math.PI;
                }
                else if (delta < -Math.PI)
                {
                    delta += 2 * Math.PI;
                }

                if (Math.Abs(delta) <= maxStep)
                { // 若本帧一步就能到达目标，直接对齐并结束动画
                    // 旋转完成
                    _robotManager.OrientationAngle = target;
                    _robotManager.IsTurning = false;

                    // 根据最终角度更新离散方向
                    _robotManager.Direction = AngleToDirection(target);

                    //if (_robot._manualForwardKeyDown)
                    //{ // 若前进键当前处于按下状态，恢复加速度让其继续前进
                    //    _move.ResumeForwardAfterTurn();
                    //}

                    return;
                }

                double step = delta > 0 ? maxStep : -maxStep;             // 本帧沿最短方向走一步
                _robotManager.OrientationAngle = NormalizeAngle(cur + step);
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

        /// <summary>
        /// 开始转向到指定离散方向（选择最短旋转路径）。
        /// 说明：
        /// - 若目标方向与当前方向相同，会直接返回；
        /// - 若角差很小（阈值 0.01 rad），则直接对齐不做动画；
        /// - 否则进入转向动画，并通过 <see cref="RobotMove.StopForTurn"/> 停止平移。
        /// </summary>
        /// <param name="targetDirection"></param>
        public void StartTurnTo(EnumMoveDirection targetDirection)
        {
            lock (_robotLock)
            {
                if (_robotManager.IsTurning)
                {
                    return;
                }

                // 获取当前方向
                EnumMoveDirection currentDir = _robotManager.Direction;

                if (currentDir == targetDirection)
                { // 如果目标方向与当前方向相同，不需要转向
                    return;
                }

                // 当前角与目标角都归一化到 [0, 2π)，便于做最短路径角差
                double currentAngle = NormalizeAngle(_robotManager.OrientationAngle);
                double targetAngle = NormalizeAngle(RobotManager.DirectionToAngle(targetDirection));

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

                if (Math.Abs(delta) < 0.01) // 约0.57度
                { // 如果旋转角度很小，直接设置方向，不进行动画
                    _robotManager.OrientationAngle = targetAngle;
                    _robotManager.TargetOrientationAngle = targetAngle;
                    _robotManager.Direction = targetDirection;
                    return;
                }

                double finalTarget = currentAngle + delta;   // delta 可能为负，表示逆时针（左转）；正表示顺时针（右转）

                _move.StopForTurn();                // 原地转向：立即把速度清零并停止加速

                _robotManager.TargetOrientationAngle = finalTarget;
                _robotManager.IsTurning = true;
            }
        }

        /// <summary>
        /// 内部转向入口：以“相对角度”启动一次转向动画（例如 ±90°）。
        /// </summary>
        /// <param name="deltaAngle"></param>
        private void StartTurnInternal(double deltaAngle)
        {
            lock (_robotLock)
            {
                if (_robotManager.IsTurning)
                { // 正在转向中，忽略新的命令
                    return;
                }

                _move.StopForTurn();                // 原地转向：立即把速度清零并停止加速

                // 获取当前角度并归一化
                double currentAngle = NormalizeAngle(_robotManager.OrientationAngle);

                // 计算新角度并归一化
                double newTarget = NormalizeAngle(currentAngle + deltaAngle);

                _robotManager.TargetOrientationAngle = newTarget;
                _robotManager.IsTurning = true;
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