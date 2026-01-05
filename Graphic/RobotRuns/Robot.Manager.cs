using System;
using System.Collections.Generic;

namespace GridDemo.RobotRuns
{
    public enum EnumMoveDirection
    {
        Right,
        Left,
        Down,
        Up
    }

    internal enum EnumRobotControlMode
    {
        Manual,
        Auto
    }

    /// <summary>
    /// 机器人基础运动模型 + 指令调度：
    /// - Form 只设置模式/输入；
    /// - Robot 将模式翻译成两类指令并下发（MoveDistance / Turn）；
    /// - Robot 定时监测指令执行情况，完成后切换下一条。
    ///
    /// 设计要点：
    /// - 指令队列（`_commandQueue`）负责“将来要做什么”；
    /// - `_currentCommand` 负责“正在做什么”；
    /// - Move/Turn 负责“怎么做”（落地执行与动画/积分），但由 Robot 串行调度，避免并发冲突。
    /// </summary>
    internal class RobotManager
    {
        private object _robotLock;

        /// <summary>
        /// 待执行指令队列（先进先出）。
        /// 说明：Robot 始终按顺序取出并串行执行，避免“边转边走”或多条移动叠加导致的不确定行为。
        /// </summary>
        private readonly Queue<RobotCommand> _commandQueue = new Queue<RobotCommand>();

        private EnumRobotControlMode _mode = EnumRobotControlMode.Auto;

        // 手动输入状态（UI 写入，Robot 读取生成指令）
        private bool _manualForwardKeyDown;
        private bool _manualTurnLeftKeyDown;
        private bool _manualTurnRightKeyDown;

        private Func<double> _getForwardAcc;
        private RobotCommand _manualCurrentMoveCommand;
        // 依赖（执行落地由 Move/Turn 提供，但由 Robot 统一调度）
        private RobotMove _move;
        private RobotTurn _turn;

        // 自动模式下的指令提供器：当自动模式且队列为空且没有当前指令时，Robot 才会尝试拉取下一条指令，避免提前“灌满队列”。
        private Func<RobotCommand> _autoCommandProvider;

        // 当前执行中的指令
        private RobotCommand _currentCommand;
        private bool _hasCurrentCommand;

        public double Acc { get; set; }
        public double MaxSpeed { get; set; }
        public EnumMoveDirection Direction { get; set; }
        public double OrientationAngle { get; set; }        // 当前朝向角度（弧度，0 向右，顺时针为正）
        public double TargetOrientationAngle { get; set; }          // 目标朝向角度（弧度），用于转向动画插值
        public bool IsTurning { get; set; }           // 由 RobotTurn 控制，指示当前是否正在转向
        public double TurnAngularSpeed { get; set; } = Math.PI;         // 转向速度（弧度/秒），默认 180°/s
        public int ManualTurnSign { get; internal set; }      // 手动转向符号：-1=左，0=不转，1=右

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
        /// 说明：Robot 只负责调度；Move/Turn 才是“实际执行器”。
        /// </summary>
        public void BindRuntime(
            object robotLock,
            RobotMove move,
            RobotTurn turn,
            Func<RobotCommand> autoCommandProvider,
            Func<double> getForwardAcc     // 新增参数
        )
        {
            _robotLock = robotLock ?? throw new ArgumentNullException(nameof(robotLock));
            _move = move ?? throw new ArgumentNullException(nameof(move));
            _turn = turn ?? throw new ArgumentNullException(nameof(turn));
            _autoCommandProvider = autoCommandProvider;
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

                // 切模式时清空队列并停止当前动作，避免“旧指令残留”
                _commandQueue.Clear();
                _hasCurrentCommand = false;
                _currentCommand = null;

                Acc = 0.0;
                _move.StopImmediately_NoLock();

                // 同步角度，确保模型一致
                double angle = DirectionToAngle(Direction);
                OrientationAngle = angle;
                TargetOrientationAngle = angle;
                IsTurning = false;
                ManualTurnSign = 0;

                _turn.ResetTargetAngle();
                _manualCurrentMoveCommand = null;
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

                _commandQueue.Clear();
                _hasCurrentCommand = false;
                _currentCommand = null;

                // 停车
                Acc = 0.0;
                _move.StopImmediately_NoLock();

                // 可选：若正在转向，也一并终止，避免“半转旧方向”
                IsTurning = false;
                double angle = DirectionToAngle(Direction);
                OrientationAngle = angle;
                TargetOrientationAngle = angle;
            }
        }

        #region UI 输入（Form 仅调用这些）
        /// <summary>
        /// UI 通知手动前进键（如 W）按下/松开。
        /// 松开时立即刹停。
        /// </summary>
        public void InputManualForwardKey(bool isDown)
        {
            lock (_robotLock)
            {
                _manualForwardKeyDown = isDown;
                if (_mode != EnumRobotControlMode.Manual)
                {
                    return;
                }

                if (isDown)
                {
                    // 手动模式：发送“前进无穷距离”——一条新的 MoveDistance 指令
                    // 这里不用队列，直接作为“当前手动指令”下发给 Move。
                    const double infiniteDist = double.MaxValue;   // 或者一个你认为合理的大值

                    _manualCurrentMoveCommand = RobotCommand.MoveDistance(infiniteDist);

                    // 下发给执行器：按当前前进加速度策略开始这条指令
                    // 加速度通过外部策略获取
                    if (_getForwardAcc == null)
                    {
                        throw new InvalidOperationException("Forward acceleration strategy (_getForwardAcc) is not set.");
                    }

                    double forwardAcc = _getForwardAcc();

                    // 将当前加速度状态写入 Manager，供后续 Move.Update 积分
                    Acc = forwardAcc;

                    _move.StartMoveDistance_NoLock(_manualCurrentMoveCommand.DistanceM.Value, forwardAcc);
                }
                else
                {
                    // 手动模式：W 松开 => 发送“前进 0 距离”的一条新指令
                    // 本质：旧的“无穷距离”指令被覆盖，不是被完成
                    _manualCurrentMoveCommand = RobotCommand.MoveDistance(0.0);

                    // 下发“0 距离”指令：由 Move 自己判定“无需前进”，并在内部把速度/加速度归零
                    _move.StartMoveDistance_NoLock(_manualCurrentMoveCommand.DistanceM.Value, 0.0);

                    // 注意：这里不直接 Speed=0，不 Acc=0
                    // 真正的停止在 RobotMove.Update 里，由“距离已完成”来触发
                }
            }
        }

        /// <summary>
        /// UI 通知左转键（如 A）按下/松开。
        /// </summary>
        public void InputManualTurnLeftKey(bool isDown)
        {
            lock (_robotLock)
            {
                _manualTurnLeftKeyDown = isDown;
                if (!isDown && !_manualTurnRightKeyDown)
                {
                    // 左右键都松开时，停止转向
                    IsTurning = false;
                    ManualTurnSign = 0;
                }
            }
        }

        /// <summary>
        /// UI 通知右转键（如 D）按下/松开。
        /// </summary>
        public void InputManualTurnRightKey(bool isDown)
        {
            lock (_robotLock)
            {
                _manualTurnRightKeyDown = isDown;
                if (!isDown && !_manualTurnLeftKeyDown)
                {
                    // 左右键都松开时，停止转向
                    IsTurning = false;
                    ManualTurnSign = 0;
                }
            }
        }
        #endregion

        /// <summary>
        /// 由仿真线程每帧调用：生成/下发/监测指令（调度核心）。
        /// 调度流程：
        /// 1) 自动模式：当“无当前指令且队列为空”时，从 provider 拉取一条新指令；
        /// 2) 手动模式：按住键盘时，一直运动（不入队指令，实时积分位移与角速度）；
        /// 3) 若当前无指令且队列非空：出队一条并下发给 Move/Turn；
        /// 4) 轮询检测当前指令是否完成：完成后清理，下一帧进入下一条。
        /// </summary>
        public void Tick(double dt, Func<double> getForwardAcc)
        {
            if (dt <= 0) throw new ArgumentOutOfRangeException(nameof(dt));
            if (getForwardAcc == null) throw new ArgumentNullException(nameof(getForwardAcc));

            lock (_robotLock)
            {
                // 1) 自动模式：按需从 provider 拉取指令进入队列
                if (_mode == EnumRobotControlMode.Auto)
                {
                    if (_autoCommandProvider != null)
                    {
                        if (!_hasCurrentCommand && _commandQueue.Count == 0)
                        {
                            RobotCommand cmd = _autoCommandProvider();
                            if (cmd != null)
                            {
                                _commandQueue.Enqueue(cmd);          // 指令入队
                            }
                        }
                    }

                    if (!_hasCurrentCommand)
                    { // 2) 取出当前指令（若没有）

                        if (_commandQueue.Count == 0)
                        {
                            return;
                        }

                        _currentCommand = _commandQueue.Dequeue();
                        _hasCurrentCommand = true;

                        // 下发给 Move/Turn（执行器内部会依据 IsTurning 等状态保护）
                        if (_currentCommand.Type == EnumRobotCommandType.MoveDistance)
                        {
                            _move.StartMoveDistance_NoLock(_currentCommand.DistanceM.Value, getForwardAcc());
                        }
                        else if (_currentCommand.Type == EnumRobotCommandType.Turn)
                        {
                            if (_currentCommand.Turn == EnumTurnCommand.Left)
                            {
                                _turn.StartTurnLeft();
                            }
                            else if (_currentCommand.Turn == EnumTurnCommand.Right)
                            {
                                _turn.StartTurnRight();
                            }
                            else
                            {
                                _turn.StartTurnTo(_currentCommand.TargetDirection.Value);
                            }
                        }
                    }

                    // 3) 监测执行完成：完成则切下一条
                    //    完成条件由执行器状态决定：Move 看自身指令状态；Turn 看 `IsTurning`。
                    if (_hasCurrentCommand)
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
                            _hasCurrentCommand = false;
                            _currentCommand = null;
                        }
                    }

                    return;  // 自动模式到此结束
                }

                // 手动模式：
                // 1) 前进：按住 W 时持续加速积分位移；松开时 Acc 归零且 Move.Stop 已在 InputManualForwardKey 做过
                if (_manualForwardKeyDown)
                {
                    Acc = getForwardAcc();
                    // 不再入队 MoveDistance，实际积分在 RobotMove.Update 中按 Acc/Speed 计算
                }
                else
                {
                    Acc = 0.0;
                }

                // 2) 转向：按住 A/D 给 Turn 写一个「当前帧目标角速度和方向」
                if (_manualTurnLeftKeyDown ^ _manualTurnRightKeyDown)
                {
                    // 只有一边按下：开始转向
                    IsTurning = true;
                    ManualTurnSign = _manualTurnLeftKeyDown ? -1 : 1;
                    // 交给 RobotTurn.Update 处理“以固定角速度持续旋转”的逻辑
                }
                else
                {
                    // 没有或两边都按：停止转向
                    IsTurning = false;
                    ManualTurnSign = 0;
                }

                // 手动模式不使用命令队列
                _commandQueue.Clear();
                _hasCurrentCommand = false;
                _currentCommand = null;
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
                case EnumMoveDirection.Right: return 0;
                case EnumMoveDirection.Down: return Math.PI / 2;
                case EnumMoveDirection.Left: return Math.PI;
                case EnumMoveDirection.Up: return 3 * Math.PI / 2;
                default: return 0;
            }
        }
    }
}