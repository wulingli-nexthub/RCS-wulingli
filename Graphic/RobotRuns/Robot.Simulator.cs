using System;
using System.Threading;
using System.Windows.Forms;

namespace GridDemo.RobotRuns
{
    internal class RobotSimulator
    {
        private readonly Control _host;
        private readonly RobotMove _move;
        private readonly double _dt;

        private Thread _workerThread;
        private volatile bool _isRunning;

        public RobotSimulator(Control host, RobotMove motion, double dt)
        {
            _host = host ?? throw new ArgumentNullException(nameof(host));
            _move = motion ?? throw new ArgumentNullException(nameof(motion));
            _dt = dt;
        }

        public void _Thead_Start()
        {
            if (_isRunning) return;

            _isRunning = true;
            _workerThread = new Thread(_Thread_Loop)
            {
                IsBackground = true
            };
            _workerThread.Start();
        }

        public void _Thead_Stop()
        {
            _isRunning = false;
            if (_workerThread != null && _workerThread.IsAlive)
            {
                _workerThread.Join(200);
            }
        }

        private void _Thread_Loop()
        {
            while (_isRunning)
            {
                // 运动一帧
                _move.Update();

                // 通知 UI 重绘
                try
                {
                    if (!_host.IsDisposed)
                    {
                        _host.BeginInvoke(new Action(() =>
                        {
                            _host.Invalidate();
                        }));
                    }
                }
                catch
                {
                    // 窗口关闭时可能抛异常，简单吞掉
                }

                int sleepMs = (int)(_dt * 1000);
                if (sleepMs < 1) sleepMs = 1;
                Thread.Sleep(sleepMs);
            }
        }
    }
}
