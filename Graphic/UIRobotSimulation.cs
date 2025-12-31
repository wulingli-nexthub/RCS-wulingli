using GridDemo.Core;
using GridDemo.RobotRuns;
using System;
using System.Windows.Forms;

namespace GridDemo
{
    internal sealed class UiRobotSimulation
    {
        private readonly RobotSimulator _loop;
        private readonly Control _host;
        private readonly RobotEngine _engine;

        public UiRobotSimulation(RobotEngine engine, Control host, double dt)
        {
            _engine = engine ?? throw new ArgumentNullException(nameof(engine));
            _host = host ?? throw new ArgumentNullException(nameof(host));

            _loop = new RobotSimulator(() =>
            {
                _engine.Tick();
                if (!_host.IsDisposed)
                {
                    _host.BeginInvoke(new Action(() =>
                    {
                        if (!_host.IsDisposed)
                            _host.Invalidate();
                    }));
                }
            }, dt);
        }

        public void Start() => _loop.Start();
        public void Stop() => _loop.Stop();
    }
}