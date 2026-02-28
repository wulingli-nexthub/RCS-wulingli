using GridDemo.WorldView;
using SkiaSharp;
using System;
using System.Collections.Generic;

namespace GridDemo.Draws
{
    /// <summary>
    /// 绘制机器人当前位置和朝向（多机器人版本）：
    /// - 红色圆点：机器人本体
    /// - 选中机器人：描边加粗/颜色强调
    /// - 黑色方向箭头：朝向
    /// - 数字标签：机器人编号（1..N）
    /// </summary>
    internal class DrawRobot
    {
        private readonly WorldTransform _transform;
        private readonly object _robotLock;

        private readonly Func<List<(int Id, double X, double Y, double Angle)>> _getRobotsSnapshot;
        private readonly Func<List<(int Id, double X, double Y)>> _getGoalsSnapshot;
        private readonly Func<int> _getSelectedId;
        private readonly Func<double> _getScale;

        internal DrawRobot(
            WorldTransform transform,
            object robotLock,
            Func<List<(int Id, double X, double Y, double Angle)>> getRobotsSnapshot,
            Func<List<(int Id, double X, double Y)>> getGoalsSnapshot,
            Func<int> getSelectedId,
            Func<double> getScale)
        {
            _transform = transform;
            _robotLock = robotLock;
            _getRobotsSnapshot = getRobotsSnapshot ?? throw new ArgumentNullException(nameof(getRobotsSnapshot));
            _getGoalsSnapshot = getGoalsSnapshot ?? throw new ArgumentNullException(nameof(getGoalsSnapshot));
            _getSelectedId = getSelectedId ?? throw new ArgumentNullException(nameof(getSelectedId));
            _getScale = getScale ?? throw new ArgumentNullException(nameof(getScale));
        }

        /// <summary>
        /// 在指定画布上绘制机器人位置和朝向
        /// </summary>
        internal void Draw(SKCanvas canvas)
        {
            List<(int Id, double X, double Y, double Angle)> robots; // 机器人快照列表（每个元素包含 Id、坐标、朝向角）
            List<(int Id, double X, double Y)> goals; // 目标点快照列表（每个元素包含 Id、坐标）
            int selectedId; // 当前选中的机器人 Id

            lock (_robotLock) // 对共享机器人状态加锁，防止绘制时被其他线程修改
            {
                robots = _getRobotsSnapshot();
                goals = _getGoalsSnapshot();
                selectedId = _getSelectedId();
            }

            if (robots == null || robots.Count == 0)
            { // 没有机器人可绘制则直接返回
                return;
            }

            double currentScale = _getScale(); // 读取当前缩放比例（用于决定显示尺寸）
            float radiusPx = (float)(currentScale / 6.0); // 将缩放换算为机器人圆点半径（像素）

            // 1) 绘制目标点：半透明圆 + 描边 + 编号标签
            if (goals != null && goals.Count > 0)
            {
                float goalRadiusPx = Math.Max(4.0f, radiusPx * 0.60f);

                using (var goalTextPaint = new SKPaint { Color = SKColors.Black, IsAntialias = true })
                using (var goalFont = new SKFont { Size = Math.Max(10.0f, goalRadiusPx * 1.0f) })
                {
                    for (int i = 0; i < goals.Count; i++)
                    {
                        var g = goals[i];
                        var sp = _transform.WorldToScreen(g.X, g.Y);

                        SKColor c = RobotPalette.GetRobotColor(g.Id);

                        using (var goalFill = new SKPaint
                        {
                            Color = new SKColor(c.Red, c.Green, c.Blue, 70),
                            IsAntialias = true,
                            Style = SKPaintStyle.Fill
                        })
                        using (var goalStroke = new SKPaint
                        {
                            Color = new SKColor(c.Red, c.Green, c.Blue, 200),
                            StrokeWidth = 2.0f,
                            IsAntialias = true,
                            Style = SKPaintStyle.Stroke
                        })
                        {
                            float gx = sp.X;
                            float gy = sp.Y;

                            canvas.DrawCircle(gx, gy, goalRadiusPx, goalFill);
                            canvas.DrawCircle(gx, gy, goalRadiusPx, goalStroke);

                            canvas.DrawText((g.Id + 1).ToString(), gx + goalRadiusPx, gy - goalRadiusPx, SKTextAlign.Left, goalFont, goalTextPaint);
                        }
                    }
                }
            }

            // 2) 绘制机器人本体：填充圆 + 描边（选中加粗）+ 方向箭头 + 编号标签
            for (int i = 0; i < robots.Count; i++)
            {
                var r = robots[i];
                var screenPos = _transform.WorldToScreen(r.X, r.Y);

                float cx = screenPos.X;
                float cy = screenPos.Y;

                bool isSelected = r.Id == selectedId;
                SKColor baseColor = RobotPalette.GetRobotColor(r.Id);

                // 本体圆：选中时 alpha=255（不透明），未选中时 alpha=220（微透明）
                using (var fill = new SKPaint
                {
                    Color = isSelected
                        ? new SKColor(baseColor.Red, baseColor.Green, baseColor.Blue, 255)
                        : new SKColor(baseColor.Red, baseColor.Green, baseColor.Blue, 220),
                    IsAntialias = true,
                    Style = SKPaintStyle.Fill
                })
                using (var stroke = new SKPaint
                {
                    Color = isSelected ? SKColors.Black : new SKColor(30, 30, 30, 200),
                    StrokeWidth = isSelected ? 3.0f : 1.5f,
                    IsAntialias = true,
                    Style = SKPaintStyle.Stroke
                })
                {
                    canvas.DrawCircle(cx, cy, radiusPx, fill);
                    canvas.DrawCircle(cx, cy, radiusPx, stroke);
                }

                // 方向箭头：从圆心略偏移处开始，沿朝向角延伸
                float arrowTotalLen = radiusPx * 1.8f;
                float arrowStartOffset = radiusPx * 0.3f;
                float arrowLineWidth = Math.Max(1.0f, radiusPx * 0.12f);

                float dirX = (float)Math.Cos(r.Angle);
                float dirY = (float)Math.Sin(r.Angle);

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

                // 编号标签：显示在右上角
                using (var textPaint = new SKPaint { Color = SKColors.Black, IsAntialias = true })
                using (var font = new SKFont { Size = Math.Max(10.0f, radiusPx * 0.9f) })
                {
                    canvas.DrawText((r.Id + 1).ToString(), cx + radiusPx, cy - radiusPx, SKTextAlign.Left, font, textPaint);
                }
            }
        }
    }
}