using SkiaSharp.Views.Desktop;
using System;
using System.Threading;
using System.Windows.Forms;

namespace GridDemo.RobotRuns
{
    /// <summary>
    /// 机器人仿真器：在后台线程中按固定时间步长推进运动学更新，并通知 UI 重绘。
    /// 说明：
    /// - 运动更新在工作线程执行（<see cref="RobotMove.Update"/>）；
    /// - UI 重绘通过 <see cref="Control.BeginInvoke(Delegate)"/> 切回 UI 线程，避免跨线程访问控件异常；
    /// - 使用 <see cref="Thread.Sleep(int)"/> 近似控制更新频率（非高精度定时）。
    /// </summary>
    internal class RobotSimulator
    {
        private readonly SKControl _host;
        private readonly Action _update;
        private readonly double _dt;
        private Thread _thread;
        private bool _running;

        public RobotSimulator(SKControl host, Action update, double dt)
        {
            _host = host ?? throw new ArgumentNullException(nameof(host));
            _update = update ?? throw new ArgumentNullException(nameof(update));
            _dt = dt;
        }

        public void _Thead_Start()
        {
            _running = true;
            _thread = new Thread(ThreadLoop) { IsBackground = true };
            _thread.Start();
        }

        public void _Thead_Stop()
        {
            _running = false;
            if (_thread != null && _thread.IsAlive)
            {
                _thread.Join(200);
            }
        }

        private void ThreadLoop()
        {
            while (_running)
            {
                _update();

                if (_host != null && !_host.IsDisposed)
                {
                    _host.Invalidate();
                }

                Thread.Sleep((int)Math.Round(_dt * 1000));
            }
        }
    }
}
