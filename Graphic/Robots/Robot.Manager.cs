using GridDemo.RobotRuns;
using System;
using System.Collections.Generic;

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
    /// - UI（Form）只写入“模式切换”和“按键输入”；
    /// - 本类把输入或自动导航结果翻译成标准化的 <see cref="RobotCommand"/> 并下发给执行器；
    /// - 执行器（<see cref="RobotMove"/> / <see cref="RobotTurn"/>）只负责“怎么做”（积分/动画），不负责选择下一条指令；
    /// - 本类在每帧 <see cref="Tick"/> 中轮询当前执行状态，完成后切换下一条指令。
    ///
    /// 设计要点：
    /// - 指令队列（<see cref="_commandQueue"/>）负责“将来要做什么”；
    /// - <see cref="_currentCommand"/> 负责“正在做什么”；
    /// - 本类保证 Move/Turn 串行执行，避免“边转边走”或多命令叠加的不可控行为。
    /// </summary>
    internal class RobotManager
    {
        private object _robotLock;

        private EnumRobotControlMode _mode = EnumRobotControlMode.Auto;

        private Func<double> _getForwardAcc;
        // 依赖（执行落地由 Move/Turn 提供，但由 Robot 统一调度）
        private RobotMove _move;
        private RobotTurn _turn;

        public double Acc { get; set; }
        public double MaxSpeed { get; set; }
        public EnumMoveDirection Direction { get; set; }
        public double OrientationAngle { get; set; }        // 当前朝向角度（弧度，0 向右，顺时针为正）
        public double TargetOrientationAngle { get; set; }          // 目标朝向角度（弧度），用于转向动画插值
        public bool IsTurning { get; set; }           // 由 RobotTurn 控制，指示当前是否正在转向
        public double TurnAngularSpeed { get; set; } = Math.PI;         // 转向速度（弧度/秒），默认 180°/s

        public RobotManager(double acc, double maxSpeed, EnumMoveDirection direction)
        {
            Acc = acc;
            MaxSpeed = maxSpeed;
            Direction = direction;

            // 初始化角度，使离散方向与连续角度保持一致
            OrientationAngle = DirectionToAngle(direction);
            TargetOrientationAngle = OrientationAngle;
        }

        /// <summary>
        /// 绑定运动执行器（在 Form 初始化阶段调用一次）。
        /// 说明：RobotManager 只负责调度；Move/Turn 才是“实际执行器”。
        /// </summary>
        public void BindRuntime(
            object robotLock,
            RobotMove move,
            RobotTurn turn,
            Func<double> getForwardAcc
        )
        {
            _robotLock = robotLock ?? throw new ArgumentNullException(nameof(robotLock));
            _move = move ?? throw new ArgumentNullException(nameof(move));
            _turn = turn ?? throw new ArgumentNullException(nameof(turn));
            _getForwardAcc = getForwardAcc ?? throw new ArgumentNullException(nameof(getForwardAcc));
        }

        /// <summary>
        /// 切换控制模式（Auto/Manual）。
        /// 切换时会：
        /// - 清空指令队列；
        /// - 终止当前指令；
        /// - 刹停（速度/加速度归零）；
        /// - 同步角度/转向状态，避免状态“半转着”跨模式遗留。
        /// </summary>
        public void SetMode(EnumRobotControlMode mode)
        {
            lock (_robotLock)
            {
                if (_mode == mode)
                {
                    return;
                }

                _mode = mode;

                Acc = 0.0;
                _move.StopImmediately_NoLock();

                _turn.ResetTargetAngle();
            }
        }

        /// <summary>
        /// 自动模式下外部需要强制刷新导航时调用：
        /// - 清空队列与当前指令；
        /// - 立即停车；
        /// 下一个 Tick 会重新向 AutoCommandProvider 拉取新指令。
        /// </summary>
        public void ResetAutoCommands()
        {
            lock (_robotLock)
            {
                if (_mode != EnumRobotControlMode.Auto)
                {
                    return;
                }

                // 停车
                Acc = 0.0;
                _move.StopImmediately_NoLock();
            }
        }
        /// <summary>
        /// 手动模式下外部触发“输入变更”后调用（仅保留接口，兼容调用点）。
        /// </summary>
        public void ResetManualCommands()
        {
            lock (_robotLock)
            {
                if (_mode != EnumRobotControlMode.Manual)
                {
                    return;
                }
            }
        }
        /// <summary>
        /// 在自动模式下，让当前箭头朝向通过转向动画对齐到当前离散方向的标准角度。
        /// </summary>
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
                    return; // 已对齐，无需插入转向命令
                }
                DispatchDirect_NoLock(RobotCommand.TurnAngle(delta));
            }
        }
        /// <summary>
        /// 直接下发一条指令（调用方已持有 _robotLock）。
        /// - cmd 为 null 则忽略。
        /// </summary>
        public void DispatchDirect_NoLock(RobotCommand cmd)
        {
            if (cmd == null)
            {
                return;
            }
            if (_getForwardAcc == null)
            {
                throw new InvalidOperationException("RobotManager 尚未绑定运行时：缺少 getForwardAcc。");
            }

            if (cmd.Type == EnumRobotCommandType.MoveDistance)
            {
                _move.StartMoveDistance_NoLock(cmd.DistanceM.Value, _getForwardAcc());
            }
            else if (cmd.Type == EnumRobotCommandType.Turn)
            {
                double delta = cmd.TurnAngleRad ?? 0.0;
                _turn.StartTurnByDelta(delta);
            }
        }
        /// <summary>
        /// 计算从当前连续角度旋转到目标离散方向标准角度的最短角度差（弧度，[-π, π]）。
        /// 正数表示顺时针（右转），负数表示逆时针（左转）。
        /// </summary>
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

        public void Tick(double dt)
        {
            if (dt <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(dt));
            }
        }

        /// <summary>
        /// 将离散方向映射为绘制/动画使用的角度（弧度）：
        /// 0=Right，π/2=Down，π=Left，3π/2=Up。
        /// </summary>
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