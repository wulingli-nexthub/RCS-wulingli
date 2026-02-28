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
        private readonly Func<List<(double X, double Y)>> _getPathPointsSnapshot;
        private readonly Func<(double X, double Y)> _getRobotWorldPos;

        /// <summary>
        /// 创建路径绘制器实例。
        /// </summary>
        internal DrawPath(
            WorldTransform transform,
            Func<List<(double X, double Y)>> getPathPointsSnapshot,
            Func<(double X, double Y)> getRobotWorldPos)
        {
            _transform = transform;
            _getPathPointsSnapshot = getPathPointsSnapshot ?? throw new ArgumentNullException(nameof(getPathPointsSnapshot));
            _getRobotWorldPos = getRobotWorldPos ?? throw new ArgumentNullException(nameof(getRobotWorldPos));
        }

        /// <summary>
        /// 在给定画布上绘制路径。
        /// 约定：当路径点少于 2 个时，不绘制（无法形成有效线段）。
        /// </summary>
        internal void Draw(SKCanvas canvas)
        {
            List<(double X, double Y)> points = _getPathPointsSnapshot();         // 获取路径点快照

            if (points == null || points.Count < 2)              // 少于 2 个点时不绘制
            {
                return;
            }

            // 关键：找到“当前位置最近的路径点索引”，认为该点之前已经走过，不再绘制
            var robot = _getRobotWorldPos();

            int startIndex = 0;
            double bestD2 = double.MaxValue;

            for (int i = 0; i < points.Count; i++)
            {
                double dx = points[i].X - robot.X;
                double dy = points[i].Y - robot.Y;
                double d2 = dx * dx + dy * dy;
                if (d2 < bestD2)
                {
                    bestD2 = d2;
                    startIndex = i;
                }
            }

            // 裁剪后不足 2 个点就不画
            if (startIndex >= points.Count - 1)
            {
                return;
            }

            using (var linePaint = new SKPaint              // 路径折线画笔
            {
                Color = new SKColor(30, 144, 255, 200),
                StrokeWidth = 3f,
                IsAntialias = true,
                Style = SKPaintStyle.Stroke,
                StrokeCap = SKStrokeCap.Round,
                StrokeJoin = SKStrokeJoin.Round,
            })
            using (var pointPaint = new SKPaint             // 路径节点画笔
            {
                Color = new SKColor(30, 144, 255, 220),
                IsAntialias = true,
                Style = SKPaintStyle.Fill
            })
            using (var goalPaint = new SKPaint              // 终点
            {
                Color = new SKColor(220, 20, 60, 220),
                IsAntialias = true,
                Style = SKPaintStyle.Fill
            })
            {
                var path = new SKPath();

                // 不再用 points[startIndex] 作为起点，起点直接用机器人当前点
                var robotScreen = _transform.WorldToScreen(robot.X, robot.Y);
                path.MoveTo(robotScreen.X, robotScreen.Y);

                for (int i = startIndex; i < points.Count; i++)
                {
                    var pi = _transform.WorldToScreen(points[i].X, points[i].Y);
                    path.LineTo(pi.X, pi.Y);
                }

                canvas.DrawPath(path, linePaint);

                const float r = 4f;
                for (int i = startIndex; i < points.Count; i++)
                {
                    var ps = _transform.WorldToScreen(points[i].X, points[i].Y);
                    canvas.DrawCircle(ps.X, ps.Y, r, pointPaint);
                }

                // 仅绘制终点红点
                var pLast = _transform.WorldToScreen(points[points.Count - 1].X, points[points.Count - 1].Y);
                canvas.DrawCircle(pLast.X, pLast.Y, 6f, goalPaint);
            }
        }
    }
}