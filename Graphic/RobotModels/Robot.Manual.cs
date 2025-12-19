using Graphic.RobotRuns;
using System;
using System.Windows.Forms;

namespace Graphic.RobotModels
{
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

        public bool IsEnabled { get; private set; }

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

        public void OnKeyDown(KeyEventArgs e)
        {
            if (e == null) throw new ArgumentNullException(nameof(e));
            if (!IsEnabled) return;

            switch (e.KeyCode)
            {
                case Keys.W:
                    lock (_robotLock)
                    {
                        _robot.IsForwardKeyDown = true;
                        if (!_robot.IsTurning)
                        {
                            _robot.Acc = _getForwardAcc();
                        }
                    }
                    e.Handled = true;
                    break;

                case Keys.A:
                    _turn.StartTurnLeft();
                    e.Handled = true;
                    break;

                case Keys.D:
                    _turn.StartTurnRight();
                    e.Handled = true;
                    break;
            }
        }

        public void OnKeyUp(KeyEventArgs e)
        {
            if (e == null) throw new ArgumentNullException(nameof(e));
            if (!IsEnabled) return;

            if (e.KeyCode == Keys.W)
            {
                lock (_robotLock)
                {
                    _robot.IsForwardKeyDown = false;
                    _setRobotSpeed(0.0);
                    _robot.Acc = 0.0;
                }
                e.Handled = true;
            }
        }
    }
}