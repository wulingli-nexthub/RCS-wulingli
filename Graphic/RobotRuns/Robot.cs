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

    public enum EnumRobotStatus
    {
        TurningLeft,
        TurningRight,
        Moving
    }

    internal enum EnumRobotControlMode
    {
        Manual,
        Auto
    }

    /// <summary>
    /// 机器人基础运动模型 + 指令调度：
    /// - Form 只设置模式/输入；
    /// - Robot 将模式翻译成两类指令并下发；
    /// - Robot 定时监测指令执行情况，完成后切换下一条。
    /// </summary>
    internal class Robot
    {
        private object _robotLock;
        private readonly Queue<RobotCommand> _commandQueue = new Queue<RobotCommand>();

        private EnumRobotControlMode _mode = EnumRobotControlMode.Auto;

        // 手动输入状态（UI 写入，Robot 读取生成指令）
        private bool _manualForwardKeyDown;

        // 依赖（执行落地由 Move/Turn 提供，但由 Robot 统一调度）
        private RobotMove _move;
        private RobotTurn _turn;
        private Func<RobotCommand> _autoCommandProvider;

        // 当前执行中的指令
        private RobotCommand _currentCommand;
        private bool _hasCurrentCommand;

        public double Acc { get; set; }
        public double MaxSpeed { get; set; }
        public EnumMoveDirection Direction { get; set; }

        /// <summary>
        /// 当前朝向角度（弧度，0 向右，顺时针为正）
        /// </summary>
        public double OrientationAngle { get; set; }

        /// <summary>
        /// 目标朝向角度（弧度），用于转向动画插值
        /// </summary>
        public double TargetOrientationAngle { get; set; }

        /// <summary>
        /// 是否正在做转向动画
        /// </summary>
        public bool IsTurning { get; set; }

        /// <summary>
        /// 转向速度（弧度/秒）
        /// </summary>
        public double TurnAngularSpeed { get; set; } = Math.PI; // 默认 180°/s

        public Robot(double acc, double maxSpeed, EnumMoveDirection direction)
        {
            Acc = acc;
            MaxSpeed = maxSpeed;
            Direction = direction;

            OrientationAngle = DirectionToAngle(direction);
            TargetOrientationAngle = OrientationAngle;
        }

        /// <summary>
        /// 绑定运动执行器（在 Form 初始化阶段调用一次）。
        /// </summary>
        public void BindRuntime(
            object robotLock,
            RobotMove move,
            RobotTurn turn,
            Func<RobotCommand> autoCommandProvider)
        {
            _robotLock = robotLock ?? throw new ArgumentNullException(nameof(robotLock));
            _move = move ?? throw new ArgumentNullException(nameof(move));
            _turn = turn ?? throw new ArgumentNullException(nameof(turn));
            _autoCommandProvider = autoCommandProvider; // 允许为 null：无自动指令源时自动模式不下发
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
            }
        }

        #region UI 输入（Form 仅调用这些）
        public void InputManualForwardKey(bool isDown)
        {
            lock (_robotLock)
            {
                _manualForwardKeyDown = isDown;
                if (!isDown)
                {
                    // 松开即刹停，并清掉未执行的前进指令
                    _commandQueue.Clear();
                    _hasCurrentCommand = false;
                    _currentCommand = null;
                    Acc = 0.0;
                    _move.StopImmediately_NoLock();
                }
            }
        }

        public void InputManualTurnLeft()
        {
            EnqueueCommand(RobotCommand.TurnLeft());
        }

        public void InputManualTurnRight()
        {
            EnqueueCommand(RobotCommand.TurnRight());
        }

        /// <summary>
        /// 手动前进：按住 W 时每次补一段“位移指令”（米）。
        /// 说明：这里用“分段位移”模拟持续按键，避免引入第三类“持续速度指令”。
        /// </summary>
        public void ManualForwardPulse(double distanceM)
        {
            if (distanceM <= 0)
            {
                return;
            }

            lock (_robotLock)
            {
                if (_mode != EnumRobotControlMode.Manual)
                {
                    return;
                }

                if (!_manualForwardKeyDown)
                {
                    return;
                }

                // 转向中不下发前进（避免边转边走）
                if (IsTurning)
                {
                    return;
                }

                _commandQueue.Enqueue(RobotCommand.MoveDistance(distanceM));
            }
        }
        #endregion

        public void EnqueueCommand(RobotCommand command)
        {
            if (command == null) throw new ArgumentNullException(nameof(command));

            lock (_robotLock)
            {
                _commandQueue.Enqueue(command);
            }
        }

        /// <summary>
        /// 由仿真线程每帧调用：生成/下发/监测指令。
        /// </summary>
        public void Tick(double dt, Func<double> getForwardAcc)
        {
            if (dt <= 0) throw new ArgumentOutOfRangeException(nameof(dt));
            if (getForwardAcc == null) throw new ArgumentNullException(nameof(getForwardAcc));

            lock (_robotLock)
            {
                // 1) 自动模式：按需从 provider 拉取指令进入队列
                if (_mode == EnumRobotControlMode.Auto && _autoCommandProvider != null)
                {
                    if (!_hasCurrentCommand && _commandQueue.Count == 0)
                    {
                        RobotCommand cmd = _autoCommandProvider();
                        if (cmd != null)
                        {
                            _commandQueue.Enqueue(cmd);
                        }
                    }
                }

                // 2) 手动模式：按住 W 时持续“脉冲式”补位移指令（dt 对应一小段距离）
                if (_mode == EnumRobotControlMode.Manual && _manualForwardKeyDown)
                {
                    // 这里用“期望速度 * dt”转成位移指令，保持指令类型只有两种
                    double forwardAcc = getForwardAcc();
                    Acc = forwardAcc;

                    // 经验值：按帧补给一个小位移，避免队列堆积过快
                    // distance = v * dt，但 v 在 Move 内部积分；这里用 MaxSpeed 做上限近似，取更保守的 0.3 倍避免突进
                    double distanceM = Math.Max(0.0, Math.Min(MaxSpeed * dt * 0.3, 0.2));
                    if (distanceM > 0)
                    {
                        _commandQueue.Enqueue(RobotCommand.MoveDistance(distanceM));
                    }
                }

                // 3) 取出当前指令（若没有）
                if (!_hasCurrentCommand)
                {
                    if (_commandQueue.Count == 0)
                    {
                        return;
                    }

                    _currentCommand = _commandQueue.Dequeue();
                    _hasCurrentCommand = true;

                    // 下发给 Move/Turn
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

                // 4) 监测执行完成：完成则切下一条
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
            }
        }

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

        /// <summary>
        /// 指示当前是否有手动前进键按下。
        /// </summary>
        internal bool ManualForwardKeyDown => _manualForwardKeyDown;
    }
}