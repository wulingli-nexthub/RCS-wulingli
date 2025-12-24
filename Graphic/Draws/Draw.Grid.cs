using GridDemo.WorldView;
using SkiaSharp;

namespace GridDemo.Draws
{
    internal class DrawGrid
    {
        private const int GridCount = 30;
        private const double CellSizeM = 0.55;

        private readonly WorldTransform _transform;
        private readonly double _worldWidthM;
        private readonly double _worldHeightM;

        internal DrawGrid(WorldTransform transform, double worldWidthM, double worldHeightM)
        {
            _transform = transform;
            _worldWidthM = worldWidthM;
            _worldHeightM = worldHeightM;
        }

        internal void Draw(SKCanvas canvas)
        {
            canvas.Clear(SKColors.White);

            using (var thinPaint = new SKPaint
            {
                Color = new SKColor(211, 211, 211),
                StrokeWidth = 1f,
                IsAntialias = true,
                Style = SKPaintStyle.Stroke
            })
            using (var thickPaint = new SKPaint
            {
                Color = SKColors.Gray,
                StrokeWidth = 1.5f,
                IsAntialias = true,
                Style = SKPaintStyle.Stroke
            })
            {
                for (int i = 0; i <= GridCount; i++)
                {
                    double xWorld = i * CellSizeM;
                    var p1 = _transform.WorldToScreen(xWorld, 0);
                    var p2 = _transform.WorldToScreen(xWorld, _worldHeightM);

                    SKPaint paint = (i % 5 == 0) ? thickPaint : thinPaint;
                    canvas.DrawLine(p1.X, p1.Y, p2.X, p2.Y, paint);
                }

                for (int j = 0; j <= GridCount; j++)
                {
                    double yWorld = j * CellSizeM;
                    var p1 = _transform.WorldToScreen(0, yWorld);
                    var p2 = _transform.WorldToScreen(_worldWidthM, yWorld);

                    SKPaint paint = (j % 5 == 0) ? thickPaint : thinPaint;
                    canvas.DrawLine(p1.X, p1.Y, p2.X, p2.Y, paint);
                }
            }
        }
    }
}