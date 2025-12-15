using System;
using System.Threading;

namespace Graphic
{
    /// <summary>
    /// 机器人模拟器：
    /// - 在后台线程中以固定时间步长周期调用 <see cref="Robot.Update(double)"/>
    /// - 每次更新后通过回调通知 UI 发起重绘（通常是 BeginInvoke(Invalidate)）
    /// - 提供 Stop/Dispose 用于在窗体关闭时安全停止线程
    /// </summary>
    public class RobotSimulation : IDisposable
    {
        /// <summary>
        /// 被驱动的机器人模型，内部线程安全。
        /// </summary>
        private readonly Robot _robot;

        private readonly double _dt;

        /// <summary>
        /// 通知 UI 重绘的回调，一般由 Form1 传入（如：BeginInvoke(Invalidate)）。
        /// </summary>
        private readonly Action _invalidateAction;

        /// <summary>
        /// 后台工作线程，循环调用 Robot.Update。
        /// </summary>
        private Thread _workerThread;

        private volatile bool _isRunning;

        /// <summary>
        /// 构造并立即启动模拟：
        /// </summary>
        /// <param name="robot">要更新的机器人实例。</param>
        /// <param name="dt">每次更新的时间步长（秒）。</param>
        /// <param name="invalidateAction">
        /// 在每次更新后调用的重绘回调；调用方负责确保在 UI 线程安全执行（通常用 BeginInvoke）。
        /// </param>
        public RobotSimulation(Robot robot, double dt, Action invalidateAction)
        {
            _robot = robot;
            _dt = dt;
            _invalidateAction = invalidateAction;

            _Thread_Start();
        }

        /// <summary>
        /// 启动后台模拟线程。
        /// </summary>
        private void _Thread_Start()
        {
            _isRunning = true;
            _workerThread = new Thread(_Thread_Loop)
            {
                IsBackground = true
            };
            _workerThread.Start();
        }

        /// <summary>
        /// 线程主循环：
        /// - 按固定 dt 调用 Robot.Update
        /// - 尝试调用重绘回调
        /// - Sleep 对应 dt 的毫秒数
        /// </summary>
        private void _Thread_Loop()
        {
            var sw = System.Diagnostics.Stopwatch.StartNew();
            double lastTime = 0.0;

            while (_isRunning)
            {
                double now = sw.Elapsed.TotalSeconds;
                double realDt = now - lastTime;
                lastTime = now;

                // 防止某一帧卡太久导致 dt 过大，可以夹一下
                if (realDt > 0.1) realDt = 0.1;

                _robot.Update(realDt);

                try
                {
                    _invalidateAction?.Invoke();
                }
                catch
                {
                }

                int sleepMs = (int)(_dt * 1000); // _dt 只用来控制大致频率
                if (sleepMs < 1) sleepMs = 1;
                Thread.Sleep(sleepMs);
            }
        }

        /// <summary>
        /// 请求停止模拟线程，并在短时间内等待其退出。
        /// 通常在窗体关闭时调用。
        /// </summary>
        public void Stop()
        {
            _isRunning = false;
            if (_workerThread != null && _workerThread.IsAlive)
            {
                _workerThread.Join(200);
            }
        }

        /// <summary>
        /// 释放资源，实现 <see cref="IDisposable"/>，内部调用 <see cref="Stop"/>。
        /// </summary>
        public void Dispose()
        {
            Stop();
        }
    }
}