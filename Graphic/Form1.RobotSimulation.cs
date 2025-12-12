using System;
using System.Threading;

namespace Graphic
{
    /// <summary>
    ///  周期调用 Robot.Update，并通过回调通知 UI 重绘
    /// </summary>
    public class RobotSimulation : IDisposable
    {
        private readonly Robot _robot;
        private readonly double _dt;
        private readonly Action _invalidateAction;

        private Thread _workerThread;
        private volatile bool _isRunning;

        public RobotSimulation(Robot robot, double dt, Action invalidateAction)
        {
            _robot = robot;
            _dt = dt;
            _invalidateAction = invalidateAction;

            _Thread_Start();
        }

        private void _Thread_Start()
        {
            _isRunning = true;
            _workerThread = new Thread(_Thread_Loop)
            {
                IsBackground = true
            };
            _workerThread.Start();
        }

        private void _Thread_Loop()
        {
            while (_isRunning)
            {
                _robot.Update(_dt);

                try
                {
                    _invalidateAction?.Invoke();
                }
                catch
                {
                    // UI 已关闭可能抛异常，简单忽略
                }

                int sleepMs = (int)(_dt * 1000);
                if (sleepMs < 1) sleepMs = 1;
                Thread.Sleep(sleepMs);
            }
        }

        public void Stop()
        {
            _isRunning = false;
            if (_workerThread != null && _workerThread.IsAlive)
            {
                _workerThread.Join(200);
            }
        }

        public void Dispose()
        {
            Stop();
        }
    }
}