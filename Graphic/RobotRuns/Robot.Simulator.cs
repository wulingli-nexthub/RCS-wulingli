using System;
using System.Threading;

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
        private readonly Action _tick;
        private readonly double _dt;
        private Thread _thread;
        private bool _running;

        public RobotSimulator(Action tick, double dt)
        {
            _tick = tick ?? throw new ArgumentNullException(nameof(tick));
            _dt = dt;
        }

        public void Start()
        {
            _running = true;
            _thread = new Thread(ThreadLoop) { IsBackground = true };
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

        private void ThreadLoop()
        {
            while (_running)
            {
                _tick();
                Thread.Sleep((int)Math.Round(_dt * 1000));
            }
        }
    }
}
