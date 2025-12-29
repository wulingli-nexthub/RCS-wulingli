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
        private readonly RobotManager _robotManager;
        private readonly double _dt;
        private readonly Func<double> _getForwardAcc;

        public RobotMotionFacade(object robotLock, RobotMove move, RobotManager robotManager, double dt, Func<double> getForwardAcc)
        {
            _robotLock = robotLock ?? throw new ArgumentNullException(nameof(robotLock));
            _move = move ?? throw new ArgumentNullException(nameof(move));
            _robotManager = robotManager ?? throw new ArgumentNullException(nameof(robotManager));
            _dt = dt;
            _getForwardAcc = getForwardAcc ?? throw new ArgumentNullException(nameof(getForwardAcc));
        }

        public void Update()
        {
            // Robot.Tick 内部会 lock
            _robotManager.Tick(_dt, _getForwardAcc);

            // Move.Update 内部会 lock
            _move.Update();
        }
    }
}