using Graphic.WorldView;
using System.Drawing;

namespace Graphic.Draws
{
    internal class DrawGrid
    {
        private const int GridCount = 30;      // 网格数30
        private const double CellSizeM = 0.55; // 一格代表距离0.55米

        private readonly WorldTransform _transform;
        private readonly double _worldWidthM;
        private readonly double _worldHeightM;

        internal DrawGrid(WorldTransform transform, double worldWidthM, double worldHeightM)
        {
            _transform = transform;
            _worldWidthM = worldWidthM;
            _worldHeightM = worldHeightM;
        }

        internal void Grid(Graphics g)
        {
            // 抗锯齿
            g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;

            // 清屏
            g.Clear(Color.White);

            // 绘制网格（世界坐标 → 屏幕坐标）
            using (var thinPen = new Pen(Color.LightGray, 1f))
            using (var thickPen = new Pen(Color.Gray, 1.5f))
            {
                // 画竖线
                for (int i = 0; i <= GridCount; i++)
                {
                    double xWorld = i * CellSizeM;
                    PointF p1 = _transform.WorldToScreen(xWorld, 0);
                    PointF p2 = _transform.WorldToScreen(xWorld, _worldHeightM);

                    Pen pen = (i % 5 == 0) ? thickPen : thinPen; // 区分每五格线
                    g.DrawLine(pen, p1, p2);
                }

                // 画横线
                for (int j = 0; j <= GridCount; j++)
                {
                    double yWorld = j * CellSizeM;
                    PointF p1 = _transform.WorldToScreen(0, yWorld);
                    PointF p2 = _transform.WorldToScreen(_worldWidthM, yWorld);

                    Pen pen = (j % 5 == 0) ? thickPen : thinPen;
                    g.DrawLine(pen, p1, p2);
                }
            }
        }
    }
}
