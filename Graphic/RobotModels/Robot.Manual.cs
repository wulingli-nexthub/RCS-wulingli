using GridDemo.RobotRuns;
using GridDemo.Robots;
using System;

namespace GridDemo.RobotModels
{
    /// <summary>
    /// 机器人手动控制器（键盘）：
    /// - 负责把键盘事件（W/A/D）翻译成“指令”（两种格式：位移/转向）；
    /// - 不直接修改 Robot 运动学状态（Acc/Speed/IsTurning 等），执行与监测由 Robot 统一调度。
    /// </summary>
    internal sealed class RobotManual
    {
        private readonly object _robotLock;
        private readonly RobotManager _robotManager;
        private bool _forwardKeyDown;
        private bool _turnLeftKeyDown;
        private bool _turnRightKeyDown;

        private const double ManualHugeTurnAngle = 1000.0; // 手动模式下的“持续转向”大角度

        public RobotManual(
            object robotLock,
            RobotManager robotManager)
        {
            _robotLock = robotLock ?? throw new ArgumentNullException(nameof(robotLock));
            _robotManager = robotManager ?? throw new ArgumentNullException(nameof(robotManager));
        }

        public bool IsEnabled { get; private set; }         // 是否启用手动控制

        public void Enable()
        {
            lock (_robotLock)
            {
                IsEnabled = true;
                _robotManager.SetMode(EnumRobotControlMode.Manual);

                // 切到手动时清空输入，避免残留
                _forwardKeyDown = false;
                _turnLeftKeyDown = false;
                _turnRightKeyDown = false;
            }
        }

        public void Disable()
        {
            lock (_robotLock)
            {
                IsEnabled = false;
                _forwardKeyDown = false;
                _turnLeftKeyDown = false;
                _turnRightKeyDown = false;
            }
        }

        public void InputForwardKey(bool isDown)
        {
            lock (_robotLock)
            {
                _forwardKeyDown = isDown;
            }
        }

        public void InputTurnLeftKey(bool isDown)
        {
            lock (_robotLock)
            {
                _turnLeftKeyDown = isDown;
            }
        }

        public void InputTurnRightKey(bool isDown)
        {
            lock (_robotLock)
            {
                _turnRightKeyDown = isDown;
            }
        }

        /// <summary>
        /// 手动模式产生命令（每次输入变化时最多产出 1 条）：
        /// - A/D 优先：持续转向；
        /// - 否则 W：无限前进；
        /// - 否则：返回 MoveDistance(0) 触发停车。
        /// </summary>
        public RobotCommand TryBuildNextCommand()
        {
            lock (_robotLock)
            {
                if (!IsEnabled)
                {
                    return null;
                }

                if (_turnLeftKeyDown && !_turnRightKeyDown)
                {
                    return RobotCommand.TurnAngle(-ManualHugeTurnAngle);
                }

                if (_turnRightKeyDown && !_turnLeftKeyDown)
                {
                    return RobotCommand.TurnAngle(ManualHugeTurnAngle);
                }

                if ((_turnLeftKeyDown && _turnRightKeyDown) || (!_turnLeftKeyDown && !_turnRightKeyDown))
                {
                    return RobotCommand.TurnAngle(0.0); // 同时按下 A/D 或松开则不转向
                }

                if (_forwardKeyDown)
                {
                    return RobotCommand.MoveDistance(double.MaxValue);
                }

                return RobotCommand.MoveDistance(0.0);
            }
        }
    }
}