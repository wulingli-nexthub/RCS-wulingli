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

        /// <summary>
        /// 启动仿真后台线程（重复调用会被忽略）。
        /// </summary>
        public void _Thead_Start()
        {
            if (_isRunning)
                return;
            _isRunning = true;
            _workerThread = new Thread(_Thread_Loop)       // 工作线程执行循环：推进运动并请求 UI 重绘
            {
                IsBackground = true
            };
            _workerThread.Start();
        }

        /// <summary>
        /// 停止仿真线程并尝试等待线程退出。
        /// </summary>
        public void _Thead_Stop()
        {
            _isRunning = false;
            if (_workerThread != null && _workerThread.IsAlive)         // Join 设定超时，避免关闭时卡死（例如 UI 正在退出或线程阻塞）
            {
                _workerThread.Join(200);
            }
        }

        /// <summary>
        /// 后台线程循环：
        /// 1) 推进一帧运动；
        /// 2) 通知 UI 重绘；
        /// 3) Sleep 近似控制帧率。
        /// </summary>
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
