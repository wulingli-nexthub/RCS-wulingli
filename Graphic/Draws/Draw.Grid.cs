using GridDemo.Robots;
using GridDemo.WorldView;
using SkiaSharp;
using System;
using System.Collections.Generic;

namespace GridDemo.Draws
{
    /// <summary>
    /// 负责绘制网格
    /// 网格在世界坐标系中，单位为米，每个网格单元大小为 0.55 米，共 30 x 30 个单元
    /// </summary>
    internal class DrawGrid
    {
        private const int GridCount = 30;           // 网格单元数量（行和列）
        private const double CellSizeM = 0.55;      // 每个网格单元的大小，单位为米

        private readonly WorldTransform _transform;
        private readonly double _worldWidthM;            // 世界宽度，单位为米
        private readonly double _worldHeightM;           // 世界高度，单位为米

        /// <summary>
        /// 创建网格绘制器
        /// </summary>
        /// <param name="transform"></param>
        /// <param name="worldWidthM"></param>
        /// <param name="worldHeightM"></param>
        internal DrawGrid(WorldTransform transform, double worldWidthM, double worldHeightM)
        {
            _transform = transform;
            _worldWidthM = worldWidthM;
            _worldHeightM = worldHeightM;
        }

        /// <summary>
        /// 绘制网格：先清屏，再绘制竖线与横线。
        /// 每第 5 条线使用更粗的画笔，形成“主网格”。
        /// </summary>
        internal void Draw(SKCanvas canvas, Func<IReadOnlyList<RobotStateSnapshot>> getRobotsSnapshot = null)
        {
            canvas.Clear(SKColors.White);               // 先清屏（背景色为白色）

            // 1) 先画路径“格子底色”，避免把线盖住
            if (getRobotsSnapshot != null)
            {
                DrawRobotPathCells(canvas, getRobotsSnapshot);
            }

            // 2) 再画网格线
            using (var thinPaint = new SKPaint
            {
                Color = new SKColor(211, 211, 211),
                StrokeWidth = 1f,
                IsAntialias = true,
                Style = SKPaintStyle.Stroke
            })
            using (var thickPaint = new SKPaint
            {
                Color = SKColors.Gray,
                StrokeWidth = 1.5f,
                IsAntialias = true,
                Style = SKPaintStyle.Stroke
            })
            {
                for (int i = 0; i <= GridCount; i++)
                {
                    double xWorld = i * CellSizeM;
                    var p1 = _transform.WorldToScreen(xWorld, 0);
                    var p2 = _transform.WorldToScreen(xWorld, _worldHeightM);

                    SKPaint paint = (i % 5 == 0) ? thickPaint : thinPaint;
                    canvas.DrawLine(p1.X, p1.Y, p2.X, p2.Y, paint);
                }

                for (int j = 0; j <= GridCount; j++)
                {
                    double yWorld = j * CellSizeM;
                    var p1 = _transform.WorldToScreen(0, yWorld);
                    var p2 = _transform.WorldToScreen(_worldWidthM, yWorld);

                    SKPaint paint = (j % 5 == 0) ? thickPaint : thinPaint;
                    canvas.DrawLine(p1.X, p1.Y, p2.X, p2.Y, paint);
                }
            }
        }

        private void DrawRobotPathCells(SKCanvas canvas, Func<IReadOnlyList<RobotStateSnapshot>> getRobotsSnapshot)
        {
            IReadOnlyList<RobotStateSnapshot> robots = getRobotsSnapshot();
            if (robots == null || robots.Count == 0)
            {
                return;
            }

            // Future3：绿色；History2：灰色
            using (var futureFill = new SKPaint
            {
                Color = new SKColor(0, 180, 0, 60),
                IsAntialias = true,
                Style = SKPaintStyle.Fill
            })
            using (var historyFill = new SKPaint
            {
                Color = new SKColor(120, 120, 120, 60),
                IsAntialias = true,
                Style = SKPaintStyle.Fill
            })
            {
                // 先画 history 再画 future，future 视觉上更“突出”
                for (int i = 0; i < robots.Count; i++)
                {
                    DrawCellsFromWorldPoints_NoLock(canvas, robots[i].History2, historyFill);
                }

                for (int i = 0; i < robots.Count; i++)
                {
                    DrawCellsFromWorldPoints_NoLock(canvas, robots[i].Future3, futureFill);
                }
            }
        }

        private void DrawCellsFromWorldPoints_NoLock(SKCanvas canvas, IReadOnlyList<(double X, double Y)> points, SKPaint fill)
        {
            if (points == null || points.Count == 0)
            {
                return;
            }

            // 做一个简易去重：避免同一格被重复刷多次
            var visited = new HashSet<long>();

            for (int i = 0; i < points.Count; i++)
            {
                var p = points[i];

                int gx = (int)Math.Floor(p.X / CellSizeM);
                int gy = (int)Math.Floor(p.Y / CellSizeM);

                if (gx < 0 || gy < 0 || gx >= GridCount || gy >= GridCount)
                {
                    continue;
                }

                long key = (((long)gx) << 32) ^ (uint)gy;
                if (!visited.Add(key))
                {
                    continue;
                }

                double left = gx * CellSizeM;
                double top = gy * CellSizeM;

                var s1 = _transform.WorldToScreen(left, top);
                var s2 = _transform.WorldToScreen(left + CellSizeM, top + CellSizeM);

                float x = Math.Min(s1.X, s2.X);
                float y = Math.Min(s1.Y, s2.Y);
                float w = Math.Abs(s2.X - s1.X);
                float h = Math.Abs(s2.Y - s1.Y);

                canvas.DrawRect(x, y, w, h, fill);
            }
        }
    }
}