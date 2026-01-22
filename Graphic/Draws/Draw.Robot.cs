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
        private readonly Func<int> _getSelectedId;
        private readonly Func<double> _getScale;

        internal DrawRobot(
            WorldTransform transform,
            object robotLock,
            Func<List<(int Id, double X, double Y, double Angle)>> getRobotsSnapshot,
            Func<int> getSelectedId,
            Func<double> getScale)
        {
            _transform = transform;
            _robotLock = robotLock;
            _getRobotsSnapshot = getRobotsSnapshot ?? throw new ArgumentNullException(nameof(getRobotsSnapshot));
            _getSelectedId = getSelectedId ?? throw new ArgumentNullException(nameof(getSelectedId));
            _getScale = getScale ?? throw new ArgumentNullException(nameof(getScale));
        }

        internal void Draw(SKCanvas canvas)
        {
            List<(int Id, double X, double Y, double Angle)> robots;
            int selectedId;

            lock (_robotLock)
            {
                robots = _getRobotsSnapshot();
                selectedId = _getSelectedId();
            }

            if (robots == null || robots.Count == 0)
            {
                return;
            }

            double currentScale = _getScale();
            float radiusPx = (float)(currentScale / 6.0);

            for (int i = 0; i < robots.Count; i++)
            {
                var r = robots[i];
                var screenPos = _transform.WorldToScreen(r.X, r.Y);

                float cx = screenPos.X;
                float cy = screenPos.Y;

                bool isSelected = r.Id == selectedId;

                using (var fill = new SKPaint
                {
                    Color = isSelected ? SKColors.OrangeRed : SKColors.Red,
                    IsAntialias = true,
                    Style = SKPaintStyle.Fill
                })
                using (var stroke = new SKPaint
                {
                    Color = isSelected ? SKColors.Gold : SKColors.Black,
                    StrokeWidth = isSelected ? 3.0f : 1.5f,
                    IsAntialias = true,
                    Style = SKPaintStyle.Stroke
                })
                {
                    canvas.DrawCircle(cx, cy, radiusPx, fill);
                    canvas.DrawCircle(cx, cy, radiusPx, stroke);
                }

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

                using (var textPaint = new SKPaint { Color = SKColors.Black, IsAntialias = true })
                using (var font = new SKFont { Size = Math.Max(10.0f, radiusPx * 0.9f) })
                {
                    canvas.DrawText((r.Id + 1).ToString(), cx + radiusPx, cy - radiusPx, SKTextAlign.Left, font, textPaint);
                }
            }
        }
    }
}