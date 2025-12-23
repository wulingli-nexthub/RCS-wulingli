using Graphic.WorldView;
using SkiaSharp;
using System;
using System.Collections.Generic;

namespace Graphic.Draws
{
    internal sealed class DrawPath
    {
        private readonly WorldTransform _transform;
        private readonly object _robotLock;

        private readonly Func<List<(double X, double Y)>> _getPathPointsSnapshot;
        private readonly Func<int> _getPathIndexSnapshot;

        internal DrawPath(
            WorldTransform transform,
            object robotLock,
            Func<List<(double X, double Y)>> getPathPointsSnapshot,
            Func<int> getPathIndexSnapshot)
        {
            _transform = transform;
            _robotLock = robotLock;
            _getPathPointsSnapshot = getPathPointsSnapshot ?? throw new ArgumentNullException(nameof(getPathPointsSnapshot));
            _getPathIndexSnapshot = getPathIndexSnapshot ?? throw new ArgumentNullException(nameof(getPathIndexSnapshot));
        }

        internal void Draw(SKCanvas canvas)
        {
            List<(double X, double Y)> points = _getPathPointsSnapshot();
            int index = _getPathIndexSnapshot();

            if (points == null || points.Count < 2)
            {
                return;
            }

            using (var linePaint = new SKPaint
            {
                Color = new SKColor(30, 144, 255, 200),
                StrokeWidth = 3f,
                IsAntialias = true,
                Style = SKPaintStyle.Stroke,
                StrokeCap = SKStrokeCap.Round,
                StrokeJoin = SKStrokeJoin.Round,
            })
            using (var pointPaint = new SKPaint
            {
                Color = new SKColor(30, 144, 255, 220),
                IsAntialias = true,
                Style = SKPaintStyle.Fill
            })
            using (var startPaint = new SKPaint
            {
                Color = new SKColor(34, 139, 34, 220),
                IsAntialias = true,
                Style = SKPaintStyle.Fill
            })
            using (var goalPaint = new SKPaint
            {
                Color = new SKColor(220, 20, 60, 220),
                IsAntialias = true,
                Style = SKPaintStyle.Fill
            })
            using (var nextPaint = new SKPaint
            {
                Color = new SKColor(255, 165, 0, 230),
                IsAntialias = true,
                Style = SKPaintStyle.Fill
            })
            {
                var path = new SKPath();

                var p0 = _transform.WorldToScreen(points[0].X, points[0].Y);
                path.MoveTo(p0.X, p0.Y);

                for (int i = 1; i < points.Count; i++)
                {
                    var pi = _transform.WorldToScreen(points[i].X, points[i].Y);
                    path.LineTo(pi.X, pi.Y);
                }

                canvas.DrawPath(path, linePaint);

                // 小圆点标记（可选）
                const float r = 4f;
                for (int i = 0; i < points.Count; i++)
                {
                    var ps = _transform.WorldToScreen(points[i].X, points[i].Y);
                    canvas.DrawCircle(ps.X, ps.Y, r, pointPaint);
                }

                // 起点 / 终点
                canvas.DrawCircle(p0.X, p0.Y, 6f, startPaint);

                var pLast = _transform.WorldToScreen(points[points.Count - 1].X, points[points.Count - 1].Y);
                canvas.DrawCircle(pLast.X, pLast.Y, 6f, goalPaint);

                // 当前“下一个”目标点（_pathIndex）
                if (index >= 0 && index < points.Count)
                {
                    var pNext = _transform.WorldToScreen(points[index].X, points[index].Y);
                    canvas.DrawCircle(pNext.X, pNext.Y, 7f, nextPaint);
                }
            }
        }
    }
}