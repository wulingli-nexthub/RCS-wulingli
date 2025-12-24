using GridDemo.WorldView;
using SkiaSharp;

namespace GridDemo.Draws
{
    /// <summary>
    /// 负责绘制网格
    /// 网格在世界坐标系中，单位为米，每个网格单元大小为 0.55 米，共 30 x 30 个单元
    /// </summary>
    internal class DrawGrid
    {
        private const int GridCount = 30;           // 网格单元数量（行和列）
        private const double CellSizeM = 0.55;      // 每个网格单元的大小，单位为米

        private readonly WorldTransform _transform;
        private readonly double _worldWidthM;            // 世界宽度，单位为米
        private readonly double _worldHeightM;           // 世界高度，单位为米

        /// <summary>
        /// 创建网格绘制器
        /// </summary>
        /// <param name="transform"></param>
        /// <param name="worldWidthM"></param>
        /// <param name="worldHeightM"></param>
        internal DrawGrid(WorldTransform transform, double worldWidthM, double worldHeightM)
        {
            _transform = transform;
            _worldWidthM = worldWidthM;
            _worldHeightM = worldHeightM;
        }

        /// <summary>
        /// 绘制网格：先清屏，再绘制竖线与横线。
        /// 每第 5 条线使用更粗的画笔，形成“主网格”。
        /// </summary>
        internal void Draw(SKCanvas canvas)
        {
            canvas.Clear(SKColors.White);               // 先清屏（背景色为白色）

            using (var thinPaint = new SKPaint            // 细线画笔
            {
                Color = new SKColor(211, 211, 211),
                StrokeWidth = 1f,
                IsAntialias = true,
                Style = SKPaintStyle.Stroke
            })
            using (var thickPaint = new SKPaint          // 粗线画笔，每第 5 条线使用
            {
                Color = SKColors.Gray,
                StrokeWidth = 1.5f,
                IsAntialias = true,
                Style = SKPaintStyle.Stroke
            })
            {
                for (int i = 0; i <= GridCount; i++)                    // 绘制竖线
                {
                    double xWorld = i * CellSizeM;
                    var p1 = _transform.WorldToScreen(xWorld, 0);
                    var p2 = _transform.WorldToScreen(xWorld, _worldHeightM);

                    SKPaint paint = (i % 5 == 0) ? thickPaint : thinPaint;            // 每第 5 条线使用粗线。？为三元运算符，满足条件取前者，否则取后者
                    canvas.DrawLine(p1.X, p1.Y, p2.X, p2.Y, paint);
                }

                for (int j = 0; j <= GridCount; j++)                      // 绘制横线
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