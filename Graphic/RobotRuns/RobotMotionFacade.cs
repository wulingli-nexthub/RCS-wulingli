using System;

namespace GridDemo.RobotRuns
{
    /// <summary>
    /// 将 Robot 的“指令调度 Tick”与 Move 的“运动学 Update”组合成 Simulator 可调用的单一入口。
    /// </summary>
    internal sealed class RobotMotionFacade
    {
        private readonly object _robotLock;
        private readonly RobotMove _move;
        private readonly Robot _robot;
        private readonly double _dt;
        private readonly Func<double> _getForwardAcc;

        public RobotMotionFacade(object robotLock, RobotMove move, Robot robot, double dt, Func<double> getForwardAcc)
        {
            _robotLock = robotLock ?? throw new ArgumentNullException(nameof(robotLock));
            _move = move ?? throw new ArgumentNullException(nameof(move));
            _robot = robot ?? throw new ArgumentNullException(nameof(robot));
            _dt = dt;
            _getForwardAcc = getForwardAcc ?? throw new ArgumentNullException(nameof(getForwardAcc));
        }

        public void Update()
        {
            // Robot.Tick 内部会 lock
            _robot.Tick(_dt, _getForwardAcc);

            // Move.Update 内部会 lock
            _move.Update();
        }
    }
}