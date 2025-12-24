using GridDemo.WorldView;
using SkiaSharp;
using System;
using System.Collections.Generic;

namespace GridDemo.Draws
{
    /// <summary>
    /// 负责绘制机器人规划/行走路径的可视化层。
    /// 输入为“世界坐标系（米）”下的一系列路径点，通过 <see cref="WorldTransform"/> 转换为屏幕坐标后绘制：
    /// - 折线（路径主体）
    /// - 每个节点的小圆点
    /// - 起点/终点强调标记
    /// - 当前“下一个目标点”（由索引指定）
    /// </summary>
    internal sealed class DrawPath
    {
        private readonly WorldTransform _transform;
        private readonly Func<List<(double X, double Y)>> _getPathPointsSnapshot;           // 获取路径点快照的委托
        private readonly Func<int> _getPathIndexSnapshot;              // 获取路径索引快照的委托

        /// <summary>
        /// 创建路径绘制器实例。
        /// </summary>
        internal DrawPath(
            WorldTransform transform,
            Func<List<(double X, double Y)>> getPathPointsSnapshot,
            Func<int> getPathIndexSnapshot)
        {
            _transform = transform;
            _getPathPointsSnapshot = getPathPointsSnapshot ?? throw new ArgumentNullException(nameof(getPathPointsSnapshot));
            _getPathIndexSnapshot = getPathIndexSnapshot ?? throw new ArgumentNullException(nameof(getPathIndexSnapshot));
        }

        /// <summary>
        /// 在给定画布上绘制路径。
        /// 约定：当路径点少于 2 个时，不绘制（无法形成有效线段）。
        /// </summary>
        internal void Draw(SKCanvas canvas)
        {
            List<(double X, double Y)> points = _getPathPointsSnapshot();         // 获取路径点快照
            int index = _getPathIndexSnapshot();                     // 获取路径索引快照

            if (points == null || points.Count < 2)              // 少于 2 个点时不绘制
            {
                return;
            }

            using (var linePaint = new SKPaint                   // 路径折线样式
            {
                Color = new SKColor(30, 144, 255, 200),
                StrokeWidth = 3f,
                IsAntialias = true,
                Style = SKPaintStyle.Stroke,
                StrokeCap = SKStrokeCap.Round,
                StrokeJoin = SKStrokeJoin.Round,
            })
            using (var pointPaint = new SKPaint                  // 路径节点小圆点样式
            {
                Color = new SKColor(30, 144, 255, 220),
                IsAntialias = true,
                Style = SKPaintStyle.Fill
            })
            using (var startPaint = new SKPaint                   // 起点样式
            {
                Color = new SKColor(34, 139, 34, 220),
                IsAntialias = true,
                Style = SKPaintStyle.Fill
            })
            using (var goalPaint = new SKPaint                    // 终点样式
            {
                Color = new SKColor(220, 20, 60, 220),
                IsAntialias = true,
                Style = SKPaintStyle.Fill
            })
            using (var nextPaint = new SKPaint                     // 当前“下一个”目标点（路径索引）样式
            {
                Color = new SKColor(255, 165, 0, 230),
                IsAntialias = true,
                Style = SKPaintStyle.Fill
            })
            {
                var path = new SKPath();                     // 生成路径折线

                var p0 = _transform.WorldToScreen(points[0].X, points[0].Y);           // 起点
                path.MoveTo(p0.X, p0.Y);

                for (int i = 1; i < points.Count; i++)                    // 其余路径点依次连线
                {
                    var pi = _transform.WorldToScreen(points[i].X, points[i].Y);
                    path.LineTo(pi.X, pi.Y);
                }

                canvas.DrawPath(path, linePaint);               // 绘制路径折线

                const float r = 4f;                          // 路径节点小圆点半径
                for (int i = 0; i < points.Count; i++)              // 绘制路径节点小圆点
                {
                    var ps = _transform.WorldToScreen(points[i].X, points[i].Y);
                    canvas.DrawCircle(ps.X, ps.Y, r, pointPaint);
                }

                canvas.DrawCircle(p0.X, p0.Y, 6f, startPaint);                // 起点 / 终点
                var pLast = _transform.WorldToScreen(points[points.Count - 1].X, points[points.Count - 1].Y);
                canvas.DrawCircle(pLast.X, pLast.Y, 6f, goalPaint);

                if (index >= 0 && index < points.Count)                // 当前“下一个”目标点（_pathIndex）
                {
                    var pNext = _transform.WorldToScreen(points[index].X, points[index].Y);
                    canvas.DrawCircle(pNext.X, pNext.Y, 7f, nextPaint);
                }
            }
        }
    }
}