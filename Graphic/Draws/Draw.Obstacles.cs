using GridDemo.WorldView;
using SkiaSharp;
using System;

namespace GridDemo.Draws
{
    /// <summary>
    /// 障碍物可视化：将为 true 的格子绘制为半透明填充。
    /// </summary>
    internal sealed class DrawObstacles
    {
        private readonly WorldTransform _transform;
        private readonly Func<bool[,]> _getObstacleSnapshot;
        private readonly Func<double> _getCellSizeM;

        internal DrawObstacles(
            WorldTransform transform,
            Func<bool[,]> getObstacleSnapshot,
            Func<double> getCellSizeM)
        {
            _transform = transform ?? throw new ArgumentNullException(nameof(transform));
            _getObstacleSnapshot = getObstacleSnapshot ?? throw new ArgumentNullException(nameof(getObstacleSnapshot));
            _getCellSizeM = getCellSizeM ?? throw new ArgumentNullException(nameof(getCellSizeM));
        }

        internal void Draw(SKCanvas canvas)
        {
            bool[,] obstacles = _getObstacleSnapshot();
            if (obstacles == null)
            {
                return;
            }

            int w = obstacles.GetLength(0);
            int h = obstacles.GetLength(1);
            double cell = _getCellSizeM();

            using (var fill = new SKPaint
            {
                Color = new SKColor(60, 60, 60, 160),
                IsAntialias = false,
                Style = SKPaintStyle.Fill
            })
            {
                for (int y = 0; y < h; y++)
                {
                    for (int x = 0; x < w; x++)
                    {
                        if (!obstacles[x, y])
                        {
                            continue;
                        }

                        double left = x * cell;
                        double top = y * cell;
                        double right = left + cell;
                        double bottom = top + cell;

                        var p1 = _transform.WorldToScreen(left, top);
                        var p2 = _transform.WorldToScreen(right, bottom);

                        float l = Math.Min(p1.X, p2.X);
                        float t = Math.Min(p1.Y, p2.Y);
                        float r = Math.Max(p1.X, p2.X);
                        float b = Math.Max(p1.Y, p2.Y);

                        canvas.DrawRect(l, t, r - l, b - t, fill);
                    }
                }
            }
        }
    }
}