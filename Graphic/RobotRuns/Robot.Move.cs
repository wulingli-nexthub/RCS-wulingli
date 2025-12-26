using System;

namespace GridDemo.RobotRuns
{
    /// <summary>
    /// 自动导航/自动运行提供给运动层的“指令快照”。
    /// 设计目的：
    /// - 将自动模式的决策（方向、加速度、是否允许边界处理、是否请求转向）与运动学更新解耦；
    /// - 每次 <see cref="RobotMove.Update"/> 读取一次，避免在一次物理帧内多处查询导致的不一致。
    /// </summary>
    internal sealed class RobotAutoMotionState
    {
        /// <summary>
        /// 关闭自动模式时的默认状态。
        /// 注意：此处 direction 只是占位，不应在 Disabled 状态下被使用来驱动移动。
        /// </summary>
        public static readonly RobotAutoMotionState Disabled = new RobotAutoMotionState(
            enabled: false,
            direction: EnumMoveDirection.Right,
            acc: 0.0,
            suppressEdgeTurning: false,
            clampOnBounds: false,
            requestTurnLeft: false,
            requestTurnToDirection: null);

        /// <summary>
        /// 构造一个自动运动状态。
        /// </summary>
        public RobotAutoMotionState(
            bool enabled,
            EnumMoveDirection direction,
            double acc,
            bool suppressEdgeTurning,
            bool clampOnBounds,
            bool requestTurnLeft,
            EnumMoveDirection? requestTurnToDirection)
        {
            Enabled = enabled;
            Direction = direction;
            Acc = acc;
            SuppressEdgeTurning = suppressEdgeTurning;
            ClampOnBounds = clampOnBounds;
            RequestTurnLeft = requestTurnLeft;
            RequestTurnToDirection = requestTurnToDirection;
        }

        public bool Enabled { get; }                   // 是否启用自动模式
        public EnumMoveDirection Direction { get; }       // 自动模式下的移动方向
        public double Acc { get; }                      // 自动模式下的加速度
        public bool SuppressEdgeTurning { get; }             // 是否抑制边界转向
        public bool ClampOnBounds { get; }            // 是否夹紧在边界内
        public bool RequestTurnLeft { get; }              // 是否请求左转
        public EnumMoveDirection? RequestTurnToDirection { get; }                // 请求转向到指定方向
    }

    /// <summary>
    /// 机器人运动更新（离散方向 + 速度/加速度 + 边界约束 + 网格中心线吸附）。
    /// 该类不负责输入/寻路决策，仅消费外部提供的状态与指令。
    /// </summary>
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
        private readonly Func<double> _getForwardAcc;
        private readonly Func<RobotAutoMotionState> _getAutoMotionState;

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
            Func<double> getForwardAcc,
            Func<RobotAutoMotionState> getAutoMotionState)
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
            _getAutoMotionState = getAutoMotionState ?? throw new ArgumentNullException(nameof(getAutoMotionState));
            _turnController = new RobotTurn(_robotLock, _robot, this, _dt);
        }

        /// <summary>
        /// 对外暴露转向控制器（例如手动控制或自动导航触发转向）。
        /// </summary>
        public RobotTurn TurnController
        {
            get { return _turnController; }
        }

        /// <summary>
        /// 转向前的制动：将速度归零，并取消加速度，避免“边转边滑”。
        /// </summary>
        public void StopForTurn()
        {
            lock (_robotLock)
            {
                _setRobotSpeed(0.0);
                _robot.Acc = 0.0;
            }
        }

        /// <summary>
        /// 转向结束后恢复前进加速度。
        /// 注意：仅当用户仍保持“前进按键按下”时才恢复，以贴合手动控制手感。
        /// </summary>
        public void ResumeForwardAfterTurn()
        {
            lock (_robotLock)
            {
                if (!_robot.IsForwardKeyDown)
                {
                    return;
                }

                double forwardAcc = _getForwardAcc();
                _robot.Acc = forwardAcc;
            }
        }

        /// <summary>
        /// 每帧更新运动学：
        /// - 读取自动模式指令并应用（加速度、转向请求等）；
        /// - 积分更新速度与位置；
        /// - 根据世界边界进行夹紧/停止处理；
        /// - 最后更新转向控制器（转向插值等）。
        /// </summary>
        public void Update()
        {
            RobotAutoMotionState autoState = _getAutoMotionState();

            lock (_robotLock)
            {
                if (autoState.Enabled)
                {
                    // 自动模式：方向/加速度由 autoState 控制
                    _robot.Acc = autoState.Acc;

                    if (autoState.RequestTurnLeft && !_robot.IsTurning)
                    { // 自动模式可提出“左转”请求：仅在当前未处于转向动画时触发。
                        _turnController.StartTurnLeft();
                    }

                    if (autoState.RequestTurnToDirection.HasValue && !_robot.IsTurning)
                    { // 自动模式可提出“转到指定方向”的请求：同样仅在未转向时触发。
                        _turnController.StartTurnTo(autoState.RequestTurnToDirection.Value);
                    }
                }

                // 读取当前状态
                double v = _getRobotSpeed();
                double a = _robot.Acc;
                double vmax = _robot.MaxSpeed;
                double x = _getRobotX();
                double y = _getRobotY();
                EnumMoveDirection dir = _robot.Direction;

                if (!_robot.IsTurning)
                { // 转向动画期间不更新位移，原地转向
                    v += a * _dt;
                    if (v < 0) v = 0;
                    if (v > vmax) v = vmax;

                    switch (dir)
                    { // 根据离散方向推进（世界坐标）
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

                    if (autoState.Enabled && autoState.ClampOnBounds)
                    { // 边界夹紧
                        if (x < halfCell)
                        {
                            x = halfCell;
                        }
                        if (y < halfCell)
                        {
                            y = halfCell;
                        }
                        if (x > worldWidth - halfCell)
                        {
                            x = worldWidth - halfCell;
                        }
                        if (y > worldHeight - halfCell)
                        {
                            y = worldHeight - halfCell;
                        }
                    }

                    if (!(autoState.Enabled && autoState.SuppressEdgeTurning))
                    { // 夹紧,确保机器人在边界内运动
                        switch (dir)
                        {
                            case EnumMoveDirection.Right:
                                if (x >= worldWidth - halfCell)
                                {
                                    x = worldWidth - halfCell;
                                    v = 0.0;
                                }
                                break;

                            case EnumMoveDirection.Down:
                                if (y >= worldHeight - halfCell)
                                {
                                    y = worldHeight - halfCell;
                                    v = 0.0;
                                }
                                break;

                            case EnumMoveDirection.Left:
                                if (x <= 0.0 + halfCell)
                                {
                                    x = halfCell;
                                    v = 0.0;
                                }
                                break;

                            case EnumMoveDirection.Up:
                                if (y <= 0.0 + halfCell)
                                {
                                    y = halfCell;
                                    v = 0.0;
                                }
                                break;
                        }
                    }
                    _setRobotSpeed(v);
                    _setRobotX(x);
                    _setRobotY(y);
                    _robot.Direction = dir;
                }
            }

            _turnController.Update();
        }
    }
}