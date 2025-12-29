using GridDemo.RobotRuns;
using System;
using System.Windows.Forms;

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
        private readonly Func<double> _getCellSizeM;
        private readonly RobotManager _robotManager;

        public RobotManual(
            object robotLock,
            Func<double> getCellSizeM,
            RobotManager robotManager)
        {
            _robotLock = robotLock ?? throw new ArgumentNullException(nameof(robotLock));
            _getCellSizeM = getCellSizeM ?? throw new ArgumentNullException(nameof(getCellSizeM));
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

        /// <summary>
        /// UI KeyDown -> 指令/输入
        /// </summary>
        public void OnKeyDown(KeyEventArgs e)
        {
            if (e == null) throw new ArgumentNullException(nameof(e));

            lock (_robotLock)
            {
                if (!IsEnabled)
                {
                    return;
                }

                switch (e.KeyCode)
                {
                    case Keys.W:
                        // 持续前进用“位移脉冲”实现：按下只标记，脉冲由 Robot.Tick 内部补发
                        _robotManager.InputManualForwardKey(true);
                        e.Handled = true;
                        break;

                    case Keys.A:
                        _robotManager.EnqueueCommand(RobotCommand.TurnLeft());
                        e.Handled = true;
                        break;

                    case Keys.D:
                        _robotManager.EnqueueCommand(RobotCommand.TurnRight());
                        e.Handled = true;
                        break;
                }
            }
        }

        /// <summary>
        /// UI KeyUp -> 停止输入
        /// </summary>
        public void OnKeyUp(KeyEventArgs e)
        {
            if (e == null) throw new ArgumentNullException(nameof(e));

            lock (_robotLock)
            {
                if (!IsEnabled)
                {
                    return;
                }

                if (e.KeyCode == Keys.W)
                {
                    _robotManager.InputManualForwardKey(false);
                    e.Handled = true;
                    return;
                }
            }
        }

        /// <summary>
        /// 可选：用于“单步前进一格”的离散指令（例如你将来做按钮/脚本控制）。
        /// </summary>
        public void StepForwardOneCell()
        {
            lock (_robotLock)
            {
                if (!IsEnabled)
                {
                    return;
                }

                double dist = _getCellSizeM();
                _robotManager.EnqueueCommand(RobotCommand.MoveDistance(dist));
            }
        }
    }
}