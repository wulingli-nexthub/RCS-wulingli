using GridDemo.RobotRuns;
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
                _robotManager.InputManualForwardKey(false);
            }
        }

        public void Disable()
        {
            lock (_robotLock)
            {
                IsEnabled = false;
                _robotManager.InputManualForwardKey(false);
            }
        }
    }
}