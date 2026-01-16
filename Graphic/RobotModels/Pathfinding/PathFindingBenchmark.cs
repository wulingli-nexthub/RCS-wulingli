using System;
using System.Diagnostics;

namespace GridDemo.RobotModels.Pathfinding
{
    internal static class PathfindingBenchmark
    {
        internal sealed class Result
        {
            public long Operations { get; private set; }
            public TimeSpan Elapsed { get; private set; }
            public double OperationsPerSecond { get; private set; }

            public Result(long operations, TimeSpan elapsed, double operationsPerSecond)
            {
                Operations = operations;
                Elapsed = elapsed;
                OperationsPerSecond = operationsPerSecond;
            }

            public override string ToString()
            {
                return string.Format(
                    "Ops={0}, Elapsed={1:F3}s, Ops/sec={2:F0}",
                    Operations,
                    Elapsed.TotalSeconds,
                    OperationsPerSecond);
            }
        }

        /// <summary>
        /// 固定时间窗口压测：统计寻路 ops/sec。
        /// </summary>
        public static Result Run(TimeSpan duration, int warmupIterations, Func<int> runOnce)
        {
            if (duration <= TimeSpan.Zero)
            {
                throw new ArgumentOutOfRangeException(nameof(duration));
            }
            if (warmupIterations < 0)
            {
                throw new ArgumentOutOfRangeException(nameof(warmupIterations));
            }
            if (runOnce == null)
            {
                throw new ArgumentNullException(nameof(runOnce));
            }

            // 预热：触发 JIT，降低首次抖动
            for (int i = 0; i < warmupIterations; i++)
            {
                runOnce();
            }

            GC.Collect();
            GC.WaitForPendingFinalizers();
            GC.Collect();

            var sw = Stopwatch.StartNew();

            long ops = 0;
            long totalPathLen = 0;

            while (sw.Elapsed < duration)
            {
                totalPathLen += runOnce();
                ops++;
            }

            sw.Stop();

            double seconds = sw.Elapsed.TotalSeconds;
            double opsPerSec = seconds <= 0 ? 0 : ops / seconds;

            // 防止 JIT/优化把整个调用链“视为无副作用”而激进优化（顺便输出个参考值）
            if (totalPathLen == long.MinValue)
            {
                throw new InvalidOperationException("Unreachable.");
            }

            return new Result(ops, sw.Elapsed, opsPerSec);
        }
    }
}