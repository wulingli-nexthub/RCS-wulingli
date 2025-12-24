using GridDemo.RobotRuns;
using System;

namespace GridDemo.RobotModels
{
    /// <summary>
    /// 机器人手动控制器（键盘）：
    /// - 负责把键盘事件（W/A/D）翻译成机器人状态的改变；
    /// - W：前进（设置 <see cref="Robot.IsForwardKeyDown"/>，并在非转向时设置加速度 <see cref="Robot.Acc"/>）；
    /// - A/D：触发 90° 左/右转（通过 <see cref="RobotTurn"/>）。
    /// 线程安全：
    /// - 通过 <see cref="_robotLock"/> 与仿真/绘制线程同步访问机器人状态。
    /// </summary>
    internal sealed class RobotManual
    {
        private readonly object _robotLock;
        private readonly Func<double> _getForwardAcc;
        private readonly Action<double> _setRobotSpeed;
        private readonly Robot _robot;
        private readonly RobotTurn _turn;

        public RobotManual(
            object robotLock,
            Func<double> getForwardAcc,
            Action<double> setRobotSpeed,
            Robot robot,
            RobotTurn turn)
        {
            _robotLock = robotLock ?? throw new ArgumentNullException(nameof(robotLock));
            _getForwardAcc = getForwardAcc ?? throw new ArgumentNullException(nameof(getForwardAcc));
            _setRobotSpeed = setRobotSpeed ?? throw new ArgumentNullException(nameof(setRobotSpeed));
            _robot = robot ?? throw new ArgumentNullException(nameof(robot));
            _turn = turn ?? throw new ArgumentNullException(nameof(turn));
        }

        public bool IsEnabled { get; private set; }         // 是否启用手动控制

        /// <summary>
        /// 启用手动模式：
        /// - 打开启用标志；
        /// - 清除“前进按键按下”状态；
        /// - 清零加速度并强制速度归零，确保进入手动模式时机器人处于可控且静止的初始状态。
        /// </summary>
        public void Enable()
        {
            lock (_robotLock)
            {
                IsEnabled = true;
                _robot.IsForwardKeyDown = false;
                _robot.Acc = 0.0;
                _setRobotSpeed(0.0);
            }
        }

        /// <summary>
        /// 禁用手动模式：
        /// - 关闭启用标志；
        /// - 同样清除按键状态并刹停，避免切换模式时机器人“带着速度/加速度”继续运动。
        /// </summary>
        public void Disable()
        {
            lock (_robotLock)
            {
                IsEnabled = false;
                _robot.IsForwardKeyDown = false;
                _robot.Acc = 0.0;
                _setRobotSpeed(0.0);
            }
        }
    }
}