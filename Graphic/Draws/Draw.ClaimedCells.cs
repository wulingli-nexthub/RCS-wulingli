using GridDemo.RobotModels.Pathfinding;
using GridDemo.WorldView;
using SkiaSharp;
using System;
using System.Collections.Generic;

namespace GridDemo.Draws
{
    /// <summary>
    /// 绘制每个机器人抢占的路径（按机器人颜色画线）。
    /// </summary>
    internal sealed class DrawClaimedCells
    {
        private readonly WorldTransform _transform;
        private readonly Func<Dictionary<int, List<GridPos>>> _getClaimedCellsSnapshot;
        private readonly Func<double> _getCellSizeM;

        internal DrawClaimedCells(
            WorldTransform transform,
            Func<Dictionary<int, List<GridPos>>> getClaimedCellsSnapshot,
            Func<double> getCellSizeM)
        {
            _transform = transform;
            _getClaimedCellsSnapshot = getClaimedCellsSnapshot ?? throw new ArgumentNullException(nameof(getClaimedCellsSnapshot));
            _getCellSizeM = getCellSizeM ?? throw new ArgumentNullException(nameof(getCellSizeM));
        }

        internal void Draw(SKCanvas canvas)
        {
            var dict = _getClaimedCellsSnapshot();
            if (dict == null || dict.Count == 0)
            {
                return;
            }

            double cellSizeM = _getCellSizeM();

            // 用 cell 的屏幕像素宽度决定线宽/点大小
            float cellSizePx = (float)Math.Abs(_transform.WorldToScreen(cellSizeM, 0).X - _transform.WorldToScreen(0, 0).X);
            float lineWidth = Math.Max(1.5f, cellSizePx * 0.18f);
            float pointRadius = Math.Max(2.0f, cellSizePx * 0.12f);

            foreach (var kv in dict)
            {
                int robotId = kv.Key;
                List<GridPos> cells = kv.Value;
                if (cells == null || cells.Count < 2)
                {
                    continue;
                }

                SKColor color = RobotPalette.GetRobotColor(robotId);

                using (var linePaint = new SKPaint
                {
                    Color = new SKColor(color.Red, color.Green, color.Blue, 180),
                    IsAntialias = true,
                    Style = SKPaintStyle.Stroke,
                    StrokeWidth = lineWidth,
                    StrokeCap = SKStrokeCap.Round,
                    StrokeJoin = SKStrokeJoin.Round
                })
                using (var pointPaint = new SKPaint
                {
                    Color = new SKColor(color.Red, color.Green, color.Blue, 200),
                    IsAntialias = true,
                    Style = SKPaintStyle.Fill
                })
                {
                    var path = new SKPath();

                    // 第一个点
                    var p0 = cells[0];
                    var s0 = _transform.WorldToScreen(
                        p0.X * cellSizeM + cellSizeM / 2.0,
                        p0.Y * cellSizeM + cellSizeM / 2.0);

                    path.MoveTo(s0.X, s0.Y);

                    // 后续点连线
                    for (int i = 1; i < cells.Count; i++)
                    {
                        var p = cells[i];
                        var sp = _transform.WorldToScreen(
                            p.X * cellSizeM + cellSizeM / 2.0,
                            p.Y * cellSizeM + cellSizeM / 2.0);

                        path.LineTo(sp.X, sp.Y);
                    }

                    canvas.DrawPath(path, linePaint);

                }
            }
        }
    }

    internal static class RobotPalette
    {
        private static readonly SKColor[] Colors = new[]
        {
            new SKColor(230, 57, 70),   // red
            new SKColor(29, 53, 87),    // dark blue
            new SKColor(69, 123, 157),  // steel blue
            new SKColor(46, 139, 87),   // sea green
            new SKColor(255, 140, 0),   // dark orange
            new SKColor(147, 112, 219), // medium purple
            new SKColor(255, 215, 0),   // gold
            new SKColor(0, 172, 193),   // cyan-ish
        };

        public static SKColor GetRobotColor(int robotId)
        {
            if (robotId < 0)
            {
                return SKColors.Red;
            }

            return Colors[robotId % Colors.Length];
        }
    }
}