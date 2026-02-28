using System;
using System.Threading;

namespace GridDemo.Robots
{
    /// <summary>
    /// 机器人仿真循环（后台线程驱动）：
    /// - 在独立的后台线程中以固定时间步长（dt）循环调用 <see cref="RobotEngine.Tick"/>；
    /// - 与 UI 线程解耦：仿真推进在后台线程，UI 通过定时器读取快照并刷新画面；
    /// - 线程为 Background 模式，应用退出时自动终止，也可通过 <see cref="Stop"/> 主动停止。
    /// 
    /// 典型调用链（项目内）：
    /// - <see cref="Form1.Form1_Load"/> 中创建实例并调用 <see cref="Start"/>；
    /// - <see cref="Form1.Form1_FormClosing"/> 中调用 <see cref="Stop"/> 安全退出。
    /// </summary>
    internal sealed class RobotSimulationLoop
    {
        private readonly RobotEngine _engine;
        private readonly double _dt;

        private Thread _thread;
        private volatile bool _running;

        /// <summary>
        /// 构造仿真循环实例。
        /// </summary>
        /// <param name="engine">机器人引擎，每帧调用其 Tick 方法推进仿真。</param>
        /// <param name="dt">仿真时间步长（秒），决定每帧 Sleep 时长与物理更新精度。</param>
        public RobotSimulationLoop(RobotEngine engine, double dt)
        {
            _engine = engine ?? throw new ArgumentNullException(nameof(engine));
            _dt = dt;
        }

        /// <summary>
        /// 启动仿真循环：创建后台线程并开始周期性调用 Engine.Tick()。
        /// 重复调用安全（已启动则忽略）。
        /// </summary>
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

        /// <summary>
        /// 停止仿真循环：设置退出标志并等待线程结束（最多 200ms 超时）。
        /// 用于窗体关闭时安全终止后台线程，避免访问已释放的控件。
        /// </summary>
        public void Stop()
        {
            _running = false;
            if (_thread != null && _thread.IsAlive)
            {
                _thread.Join(200);
            }
        }

        /// <summary>
        /// 后台线程主体：按 dt 对应的毫秒间隔循环调用 Engine.Tick()。
        /// 注意：使用 Thread.Sleep 做简易节拍，不保证精确实时性，但对仿真演示足够。
        /// </summary>
        private void _Thread_Loop()
        {
            int sleepMs = (int)Math.Round(_dt * 1000);

            while (_running)
            {
                // 运动学更新在工作线程执行
                _engine.Tick();

                Thread.Sleep(sleepMs);
            }
        }
    }
}