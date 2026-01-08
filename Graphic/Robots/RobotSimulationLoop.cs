using System;
using System.Threading;
using System.Windows.Forms;

namespace GridDemo.Robots
{
    internal sealed class RobotSimulationLoop
    {
        private readonly RobotEngine _engine;
        private readonly Control _host;
        private readonly double _dt;

        private Thread _thread;
        private bool _running;

        public RobotSimulationLoop(RobotEngine engine, Control host, double dt)
        {
            _engine = engine ?? throw new ArgumentNullException(nameof(engine));
            _host = host ?? throw new ArgumentNullException(nameof(host));
            _dt = dt;
        }

        public void Start()
        {
            if (_running) return;

            _running = true;
            _thread = new Thread(_Thread_Loop)
            {
                IsBackground = true,
                Name = "RobotSimulationLoop"
            };
            _thread.Start();
        }

        public void Stop()
        {
            _running = false;
            if (_thread != null && _thread.IsAlive)
            {
                _thread.Join(200);
            }
        }

        private void _Thread_Loop()
        {
            int sleepMs = (int)Math.Round(_dt * 1000);

            while (_running)
            {
                // 运动学更新在工作线程执行
                _engine.Tick();

                // 通知 UI 线程重绘
                if (!_host.IsDisposed)
                {
                    try
                    {
                        _host.BeginInvoke(new Action(() =>
                        {
                            if (!_host.IsDisposed)
                                _host.Invalidate();
                        }));
                    }
                    catch (ObjectDisposedException)
                    {
                        // 窗口可能正在关闭，忽略
                        _running = false;
                        break;
                    }
                }

                Thread.Sleep(sleepMs);
            }
        }
    }
}