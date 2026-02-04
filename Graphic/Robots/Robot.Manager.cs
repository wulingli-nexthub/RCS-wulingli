using GridDemo.RobotRuns;
using System;

namespace GridDemo.Robots
{
    public enum EnumMoveDirection
    {
        Right = 0,
        Down = 1,
        Left = 2,
        Up = 3
    }

    internal enum EnumRobotControlMode
    {
        Manual,
        Auto
    }

    /// <summary>
    /// 机器人基础运动模型 + 指令调度（调度层/状态中心）：
    /// - 取消“指令队列”，改为每次直接下发指令给执行器；
    /// - Manual：每帧覆盖下发（按住即时响应）；
    /// - Auto：检测到当前指令完成后才下发下一条。
    ///
    /// 停车规则：
    /// - 不通过 null 指令停车；
    /// - 停车必须下发 MoveDistance(0)（RobotMove 内部会自然刹停）。
    /// </summary>
    internal class RobotManager
    {
        private object _robotLock;

        private EnumRobotControlMode _mode = EnumRobotControlMode.Auto;

        private Func<double> _getForwardAcc;
        private Func<RobotCommand> _manualCommandProvider;
        private RobotMove _move;
        private RobotTurn _turn;

        private Func<RobotCommand> _autoCommandProvider;

        // 当前执行中的指令（用于 Auto 模式完成检测）
        private RobotCommand _currentCommand;

        public double Acc { get; set; }
        public double MaxSpeed { get; set; }
        public EnumMoveDirection Direction { get; set; }
        public double OrientationAngle { get; set; }
        public double TargetOrientationAngle { get; set; }
        public bool IsTurning { get; set; }
        public double TurnAngularSpeed { get; set; } = Math.PI;

        public RobotManager(double acc, double maxSpeed, EnumMoveDirection direction)
        {
            Acc = acc;
            MaxSpeed = maxSpeed;
            Direction = direction;

            OrientationAngle = DirectionToAngle(direction);
            TargetOrientationAngle = OrientationAngle;
        }

        public void BindRuntime(
            object robotLock,
            RobotMove move,
            RobotTurn turn,
            Func<RobotCommand> autoCommandProvider,
            Func<double> getForwardAcc
        )
        {
            _robotLock = robotLock ?? throw new ArgumentNullException(nameof(robotLock));
            _move = move ?? throw new ArgumentNullException(nameof(move));
            _turn = turn ?? throw new ArgumentNullException(nameof(turn));
            _autoCommandProvider = autoCommandProvider;
            _getForwardAcc = getForwardAcc ?? throw new ArgumentNullException(nameof(getForwardAcc));
        }

        public void BindManualCommandProvider(Func<RobotCommand> manualCommandProvider)
        {
            lock (_robotLock)
            {
                _manualCommandProvider = manualCommandProvider;
            }
        }

        public void SetMode(EnumRobotControlMode mode)
        {
            lock (_robotLock)
            {
                if (_mode == mode)
                {
                    return;
                }

                _mode = mode;
                _currentCommand = null;

                // 切模式：不直接 StopImmediately；按约束③用 0 距离指令刹停
                _move.StartMoveDistance_NoLock(0.0, 0.0);

                _turn.ResetTargetAngle();
            }
        }

        public void ResetAutoCommands()
        {
            lock (_robotLock)
            {
                if (_mode != EnumRobotControlMode.Auto)
                {
                    return;
                }

                _currentCommand = null;

                // 刷新自动：同样用 0 距离指令刹停
                _move.StartMoveDistance_NoLock(0.0, 0.0);
            }
        }

        public void AlignOrientationToDirectionWithTurn()
        {
            lock (_robotLock)
            {
                if (_mode != EnumRobotControlMode.Auto)
                {
                    return;
                }

                double delta = GetShortestDeltaToDirection(OrientationAngle, Direction);
                if (Math.Abs(delta) < 1e-6)
                {
                    return;
                }

                // 对齐前先停车（约束③）
                _move.StartMoveDistance_NoLock(0.0, 0.0);

                _currentCommand = RobotCommand.TurnAngle(delta);
                _turn.StartTurnByDelta(delta);
            }
        }

        private static double GetShortestDeltaToDirection(double currentAngle, EnumMoveDirection targetDir)
        {
            double targetAngle = DirectionToAngle(targetDir);

            currentAngle = currentAngle % (2 * Math.PI);
            if (currentAngle < 0)
            {
                currentAngle += 2 * Math.PI;
            }

            targetAngle = targetAngle % (2 * Math.PI);
            if (targetAngle < 0)
            {
                targetAngle += 2 * Math.PI;
            }

            double delta = targetAngle - currentAngle;
            if (delta > Math.PI)
            {
                delta -= 2 * Math.PI;
            }
            else if (delta < -Math.PI)
            {
                delta += 2 * Math.PI;
            }

            return delta;
        }

        public void ResetManualCommands()
        {
            lock (_robotLock)
            {
                if (_mode != EnumRobotControlMode.Manual)
                {
                    return;
                }

                _currentCommand = null;

                // 手动输入变化：按约束③用 0 距离指令刹停
                _move.StartMoveDistance_NoLock(0.0, 0.0);
                _turn.ResetTargetAngle();
            }
        }

        /// <summary>
        /// 调度核心：
        /// - Manual：每帧覆盖下发（允许抢占）。
        /// - Auto：若当前指令完成，则拉取并下发下一条。
        ///
        /// 停车：不使用 null 停车；停车必须由 Provider 返回 MoveDistance(0)。
        /// </summary>
        public void Tick(double dt, Func<double> getForwardAcc)
        {
            if (dt <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(dt));
            }
            if (getForwardAcc == null)
            {
                throw new ArgumentNullException(nameof(getForwardAcc));
            }

            lock (_robotLock)
            {
                if (_mode == EnumRobotControlMode.Manual)
                {
                    if (_manualCommandProvider == null)
                    {
                        return;
                    }

                    RobotCommand cmd = _manualCommandProvider();
                    if (cmd == null)
                    {
                        // 不把 null 当停车；保持静默
                        return;
                    }

                    _currentCommand = cmd;

                    if (cmd.Type == EnumRobotCommandType.MoveDistance)
                    {
                        _move.StartMoveDistance_NoLock(cmd.DistanceM.Value, getForwardAcc());
                    }
                    else if (cmd.Type == EnumRobotCommandType.Turn)
                    {
                        double delta = cmd.TurnAngleRad ?? 0.0;
                        _turn.StartTurnByDelta(delta);
                    }

                    return;
                }

                if (_mode == EnumRobotControlMode.Auto)
                {
                    // 1) 若有当前指令：先检测完成
                    if (_currentCommand != null)
                    {
                        bool done = false;

                        if (_currentCommand.Type == EnumRobotCommandType.MoveDistance)
                        {
                            done = _move.IsMoveDistanceDone_NoLock();
                        }
                        else if (_currentCommand.Type == EnumRobotCommandType.Turn)
                        {
                            done = !IsTurning;
                        }

                        if (done)
                        {
                            _currentCommand = null;
                        }
                    }

                    // 2) 当前无指令：下发下一条
                    if (_currentCommand == null)
                    {
                        if (_autoCommandProvider == null)
                        {
                            return;
                        }

                        RobotCommand cmd = _autoCommandProvider();
                        if (cmd == null)
                        {
                            // 不把 null 当停车；保持静默
                            return;
                        }

                        _currentCommand = cmd;

                        if (cmd.Type == EnumRobotCommandType.MoveDistance)
                        {
                            _move.StartMoveDistance_NoLock(cmd.DistanceM.Value, getForwardAcc());
                        }
                        else if (cmd.Type == EnumRobotCommandType.Turn)
                        {
                            double delta = cmd.TurnAngleRad ?? 0.0;
                            _turn.StartTurnByDelta(delta);
                        }
                    }

                    return;
                }

                // 未知模式：用 0 距离指令刹停（约束③）
                _currentCommand = null;
                _move.StartMoveDistance_NoLock(0, 0);
                _turn.ResetTargetAngle();
            }
        }

        public static double DirectionToAngle(EnumMoveDirection dir)
        {
            switch (dir)
            {
                case EnumMoveDirection.Right:
                    return 0;
                case EnumMoveDirection.Down:
                    return Math.PI / 2;
                case EnumMoveDirection.Left:
                    return Math.PI;
                case EnumMoveDirection.Up:
                    return 3 * Math.PI / 2;
                default:
                    return 0;
            }
        }
    }
}