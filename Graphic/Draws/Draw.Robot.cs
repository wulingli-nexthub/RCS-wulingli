using Graphic.WorldView;
using SkiaSharp;
using System;

namespace Graphic.Draws
{
    internal class DrawRobot
    {
        private readonly WorldTransform _transform;
        private readonly object _robotLock;

        private readonly Func<(double X, double Y)> _getRobotPosition;
        private readonly Func<double> _getScale;
        private readonly Func<double> _getOrientationAngle;

        internal DrawRobot(
            WorldTransform transform,
            object robotLock,
            Func<(double X, double Y)> getRobotPosition,
            Func<double> getScale,
            Func<double> getOrientationAngle)
        {
            _transform = transform;
            _robotLock = robotLock;
            _getRobotPosition = getRobotPosition;
            _getScale = getScale;
            _getOrientationAngle = getOrientationAngle;
        }

        internal void Draw(SKCanvas canvas)
        {
            double robotX;
            double robotY;
            double angle;

            lock (_robotLock)
            {
                var pos = _getRobotPosition();
                robotX = pos.X;
                robotY = pos.Y;
                angle = _getOrientationAngle();
            }

            var screenPos = _transform.WorldToScreen(robotX, robotY);
            double currentScale = _getScale();

            float radiusPx = (float)(currentScale / 6.0);
            float cx = screenPos.X;
            float cy = screenPos.Y;

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
            {
                canvas.DrawCircle(cx, cy, radiusPx, fill);
                canvas.DrawCircle(cx, cy, radiusPx, stroke);
            }

            float arrowTotalLen = radiusPx * 1.8f;
            float arrowStartOffset = radiusPx * 0.3f;
            float arrowLineWidth = Math.Max(1.0f, radiusPx * 0.12f);

            float dirX = (float)Math.Cos(angle);
            float dirY = (float)Math.Sin(angle);

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
        }
    }
}