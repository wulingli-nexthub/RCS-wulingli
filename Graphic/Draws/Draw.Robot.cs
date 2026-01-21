using GridDemo.MultiRobots;
using GridDemo.Robots;
using GridDemo.WorldView;
using SkiaSharp;
using System;

namespace GridDemo.Draws
{
    /// <summary>
    /// 绘制机器人当前位置和朝向
    /// 当前绘制效果：
    /// - 红色圆点：机器人本体（半径随缩放变化，保证不同缩放下视觉大小相对稳定）
    /// - 黑色描边：圆形轮廓
    /// - 黑色方向箭头：表示机器人朝向（由 <see cref="_getOrientationAngle"/> 提供的弧度角决定）
    /// </summary>
    internal class DrawRobot
    {
        private readonly WorldTransform _transform;
        private readonly object _robotLock;     // 机器人状态锁：用于与仿真/控制线程同步读取位置与角度，避免绘制线程读到不一致状态。

        private readonly Func<MultiRobotStateSnapshot> _getSnapshot;
        private readonly Func<double> _getScale;

        /// <summary>
        /// 创建机器人绘制器实例。
        /// </summary>
        internal DrawRobot(
            WorldTransform transform,
            object robotLock,
            Func<MultiRobotStateSnapshot> getSnapshot,
            Func<double> getScale)
        {
            _transform = transform;
            _robotLock = robotLock;
            _getSnapshot = getSnapshot;
            _getScale = getScale;
        }

        /// <summary>
        /// 绘制机器人：读取当前位置与朝向，转换到屏幕坐标后绘制圆形与方向箭头。
        /// </summary>
        internal void Draw(SKCanvas canvas)
        {
            MultiRobotStateSnapshot snapshot;

            lock (_robotLock)
            {
                snapshot = _getSnapshot();
            }

            double currentScale = _getScale();
            float radiusPx = (float)(currentScale / 6.0);

            // 通过目标点与机器人点的转换方式可以获得 cellSize，但这里用“相邻格在世界坐标差”来推不靠谱
            // 因此：直接用“画面上目标十字使用的 world 坐标”已知 cellSize 不在此类里
            // => 用屏幕上的 world->screen 的差值难算
            // 这里采用：从相邻格中心差固定 cellSizeM，因此要从 snapshot 里提供 cellSizeM 才最干净
            // 为最小改动：用半径推格大小不可控，所以改成“绘制与网格对齐的矩形边框”，需要 cellSizeM
            // 解决：用 world 坐标推断 cellSize：取 GoalWorldX - (GoalGridX*cellSize + cell/2) 不行
            // => 最稳：直接在 WorldTransform 外部传入 cellSizeM；但当前构造没这个参数。
            // 这里采用折中：用 snapshot 第一个机器人目标的 grid/world 反推出 cellSize。
            double cellSizeM = 0.0;
            if (snapshot.Robots.Count > 0)
            {
                var r0 = snapshot.Robots[0];
                if (r0.HasGoal)
                {
                    // goalWorldX = gx*cell + cell/2 => cell = 2*(goalWorldX - gx*cell) 不可直接解
                    // 退一步：用相邻格差：绘制层无法得知；所以这里用固定值 0.55（与 Form1 常量一致）
                    cellSizeM = 0.55;
                }
                else
                {
                    cellSizeM = 0.55;
                }
            }
            else
            {
                cellSizeM = 0.55;
            }

            using (var textPaint = new SKPaint
            {
                Color = SKColors.Black,
                IsAntialias = true
            })
            using (var font = new SKFont { Size = Math.Max(10, radiusPx) })
            using (var goalPaint = new SKPaint
            {
                Color = SKColors.DarkGreen,
                StrokeWidth = 2,
                IsAntialias = true,
                Style = SKPaintStyle.Stroke,
                StrokeCap = SKStrokeCap.Round
            })
            using (var forwardFill = new SKPaint
            {
                Color = new SKColor(255, 0, 0, 70), // 半透明红
                IsAntialias = true,
                Style = SKPaintStyle.Fill
            })
            using (var backFill = new SKPaint
            {
                Color = new SKColor(0, 120, 255, 70), // 半透明蓝
                IsAntialias = true,
                Style = SKPaintStyle.Fill
            })
            using (var cellStroke = new SKPaint
            {
                Color = new SKColor(0, 0, 0, 80),
                StrokeWidth = 1,
                IsAntialias = true,
                Style = SKPaintStyle.Stroke
            })
            {
                for (int i = 0; i < snapshot.Robots.Count; i++)
                {
                    var r = snapshot.Robots[i];

                    // 1) 画目标点（十字）
                    if (r.HasGoal)
                    {
                        var goalScreen = _transform.WorldToScreen(r.GoalWorldX, r.GoalWorldY);
                        float gx = goalScreen.X;
                        float gy = goalScreen.Y;
                        float s = Math.Max(6, radiusPx);

                        canvas.DrawLine(gx - s, gy, gx + s, gy, goalPaint);
                        canvas.DrawLine(gx, gy - s, gx, gy + s, goalPaint);
                    }

                    // 2) 画“前3后2格”预览
                    DrawPreviewCells(canvas, r, cellSizeM, forwardFill, backFill, cellStroke);

                    // 3) 画机器人
                    var pos = _transform.WorldToScreen(r.X, r.Y);
                    float cx = pos.X;
                    float cy = pos.Y;

                    SKColor fillColor = r.IsSelected ? SKColors.OrangeRed : SKColors.Red;
                    float strokeWidth = r.IsSelected ? 3.0f : 1.5f;

                    using (var fill = new SKPaint
                    {
                        Color = fillColor,
                        IsAntialias = true,
                        Style = SKPaintStyle.Fill
                    })
                    using (var stroke = new SKPaint
                    {
                        Color = SKColors.Black,
                        StrokeWidth = strokeWidth,
                        IsAntialias = true,
                        Style = SKPaintStyle.Stroke
                    })
                    {
                        canvas.DrawCircle(cx, cy, radiusPx, fill);
                        canvas.DrawCircle(cx, cy, radiusPx, stroke);
                    }

                    // 4) 方向箭头
                    float arrowTotalLen = radiusPx * 1.8f;
                    float arrowStartOffset = radiusPx * 0.3f;
                    float arrowLineWidth = Math.Max(1.0f, radiusPx * 0.12f);

                    float dirX = (float)Math.Cos(r.OrientationAngle);
                    float dirY = (float)Math.Sin(r.OrientationAngle);

                    float x1 = cx + dirX * arrowStartOffset;
                    float y1 = cy + dirY * arrowStartOffset;
                    float x2 = cx + dirX * arrowTotalLen;
                    float y2 = cy + dirY * arrowTotalLen;

                    using (var arrowPaint = new SKPaint
                    {
                        Color = SKColors.Black,
                        StrokeWidth = arrowLineWidth,
                        IsAntialias = true,
                        Style = SKPaintStyle.Stroke,
                        StrokeCap = SKStrokeCap.Round
                    })
                    {
                        canvas.DrawLine(x1, y1, x2, y2, arrowPaint);
                    }

                    // 5) 文本：ID + 目标格子
                    string goalText = r.HasGoal ? ("(" + r.GoalGridX + "," + r.GoalGridY + ")") : "(no goal)";
                    string label = "ID:" + r.Id + " G:" + goalText;

                    canvas.DrawText(label, cx + radiusPx + 4, cy - radiusPx - 4, SKTextAlign.Left, font, textPaint);
                }
            }
        }

        private void DrawPreviewCells(SKCanvas canvas, RobotItemSnapshot r, double cellSizeM, SKPaint forwardFill, SKPaint backFill, SKPaint stroke)
        {
            int cx = (int)Math.Floor(r.X / cellSizeM);
            int cy = (int)Math.Floor(r.Y / cellSizeM);

            int stepX;
            int stepY;

            GetDirStep(r.Direction, out stepX, out stepY);

            // 前 3 格
            for (int k = 1; k <= 3; k++)
            {
                DrawCellRect(canvas, cx + stepX * k, cy + stepY * k, cellSizeM, forwardFill, stroke);
            }

            // 后 2 格
            for (int k = 1; k <= 2; k++)
            {
                DrawCellRect(canvas, cx - stepX * k, cy - stepY * k, cellSizeM, backFill, stroke);
            }
        }

        private void DrawCellRect(SKCanvas canvas, int gx, int gy, double cellSizeM, SKPaint fill, SKPaint stroke)
        {
            double left = gx * cellSizeM;
            double top = gy * cellSizeM;
            double right = left + cellSizeM;
            double bottom = top + cellSizeM;

            var p1 = _transform.WorldToScreen(left, top);
            var p2 = _transform.WorldToScreen(right, bottom);

            float x = Math.Min(p1.X, p2.X);
            float y = Math.Min(p1.Y, p2.Y);
            float w = Math.Abs(p2.X - p1.X);
            float h = Math.Abs(p2.Y - p1.Y);

            var rect = new SKRect(x, y, x + w, y + h);

            canvas.DrawRect(rect, fill);
            canvas.DrawRect(rect, stroke);
        }

        private static void GetDirStep(EnumMoveDirection dir, out int dx, out int dy)
        {
            dx = 0;
            dy = 0;

            switch (dir)
            {
                case EnumMoveDirection.Right:
                    dx = 1;
                    dy = 0;
                    break;
                case EnumMoveDirection.Left:
                    dx = -1;
                    dy = 0;
                    break;
                case EnumMoveDirection.Down:
                    dx = 0;
                    dy = 1;
                    break;
                case EnumMoveDirection.Up:
                    dx = 0;
                    dy = -1;
                    break;
            }
        }
    }
}