using GridDemo.Robots;
using GridDemo.WorldView;
using SkiaSharp;
using System;
using System.Collections.Generic;

namespace GridDemo.Draws
{
    internal sealed class DrawRobots
    {
        private readonly WorldTransform _transform;
        private readonly object _robotLock;
        private readonly Func<IReadOnlyList<RobotStateSnapshot>> _getRobots;
        private readonly Func<double> _getScale;
        private readonly Func<int?> _getSelectedRobotId;

        public DrawRobots(
            WorldTransform transform,
            object robotLock,
            Func<IReadOnlyList<RobotStateSnapshot>> getRobots,
            Func<double> getScale,
            Func<int?> getSelectedRobotId)
        {
            _transform = transform;
            _robotLock = robotLock;
            _getRobots = getRobots;
            _getScale = getScale;
            _getSelectedRobotId = getSelectedRobotId;
        }

        public void Draw(SKCanvas canvas)
        {
            IReadOnlyList<RobotStateSnapshot> robots;
            int? selectedId;

            lock (_robotLock)
            {
                robots = _getRobots();
                selectedId = _getSelectedRobotId != null ? _getSelectedRobotId() : (int?)null;
            }

            double currentScale = _getScale();
            float radiusPx = (float)(currentScale / 7.0);

            using (var fill = new SKPaint
            {
                Color = SKColors.Red,
                IsAntialias = true,
                Style = SKPaintStyle.Fill
            })
            using (var stroke = new SKPaint
            {
                Color = SKColors.Black,
                StrokeWidth = 1.5f,
                IsAntialias = true,
                Style = SKPaintStyle.Stroke
            })
            using (var selectedStroke = new SKPaint
            {
                Color = SKColors.DeepSkyBlue,
                StrokeWidth = 3.0f,
                IsAntialias = true,
                Style = SKPaintStyle.Stroke
            })
            using (var arrowPaint = new SKPaint
            {
                Color = SKColors.Black,
                StrokeWidth = Math.Max(1.0f, radiusPx * 0.12f),
                IsAntialias = true,
                Style = SKPaintStyle.Stroke,
                StrokeCap = SKStrokeCap.Round
            })
            using (var idTextPaint = new SKPaint
            {
                Color = SKColors.Black,
                IsAntialias = true
            })
            using (var idFont = new SKFont { Size = Math.Max(10.0f, radiusPx * 0.9f) })
            {
                for (int i = 0; i < robots.Count; i++)
                {
                    var r = robots[i];
                    var screenPos = _transform.WorldToScreen(r.X, r.Y);

                    float cx = screenPos.X;
                    float cy = screenPos.Y;

                    // 机器人本体
                    canvas.DrawCircle(cx, cy, radiusPx, fill);
                    canvas.DrawCircle(cx, cy, radiusPx, stroke);

                    // 选中高亮
                    if (selectedId.HasValue && r.Id == selectedId.Value)
                    {
                        canvas.DrawCircle(cx, cy, radiusPx + 2.0f, selectedStroke);
                    }

                    // 朝向箭头
                    float arrowTotalLen = radiusPx * 1.8f;
                    float arrowStartOffset = radiusPx * 0.3f;

                    float dirX = (float)Math.Cos(r.OrientationAngle);
                    float dirY = (float)Math.Sin(r.OrientationAngle);

                    float x1 = cx + dirX * arrowStartOffset;
                    float y1 = cy + dirY * arrowStartOffset;
                    float x2 = cx + dirX * arrowTotalLen;
                    float y2 = cy + dirY * arrowTotalLen;

                    canvas.DrawLine(x1, y1, x2, y2, arrowPaint);

                    // 序号文本
                    canvas.DrawText(
                        r.Id.ToString(),
                        cx + radiusPx + 2.0f,
                        cy - radiusPx - 2.0f,
                        SKTextAlign.Left,
                        idFont,
                        idTextPaint);
                }
            }
        }
    }
}