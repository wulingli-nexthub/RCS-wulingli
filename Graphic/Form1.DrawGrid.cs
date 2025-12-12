using System.Drawing;

namespace Graphic
{
    public class DrawGrid
    {
        public int GridCount { get; } = 30;          // 网格数
        public double CellSizeM { get; } = 0.55;     // 每格0.55m

        public double WorldWidthM => GridCount * CellSizeM;         // 世界宽度，单位米
        public double WorldHeightM => GridCount * CellSizeM;       // 世界高度，单位米

        public double Scale { get; set; } = 30.0;    // 1m = 30px
        public double OffsetX { get; set; }          // 世界(0,0) 映射到屏幕的X
        public double OffsetY { get; set; }          // 世界(0,0) 映射到屏幕的Y

        // 这三个记录“初始”状态，方便重置
        public double InitialScale { get; set; }
        public double InitialOffsetX { get; set; }
        public double InitialOffsetY { get; set; }

        // 世界->屏幕
        public PointF WorldToScreen(double wx, double wy)
        {
            float sx = (float)(wx * Scale + OffsetX);
            float sy = (float)(wy * Scale + OffsetY);
            return new PointF(sx, sy);
        }

        // 屏幕->世界
        public PointF ScreenToWorld(float sx, float sy)
        {
            float wx = (float)((sx - OffsetX) / Scale);
            float wy = (float)((sy - OffsetY) / Scale);
            return new PointF(wx, wy);
        }

        /// <summary>
        /// 绘制网格
        /// </summary>
        /// <param name="g"></param>
        public void Draw(Graphics g)
        {
            using (var thinPen = new Pen(Color.LightGray, 1f))
            using (var thickPen = new Pen(Color.Gray, 1.5f))
            {
                // 竖线
                for (int i = 0; i <= GridCount; i++)
                {
                    double xWorld = i * CellSizeM;
                    PointF p1 = WorldToScreen(xWorld, 0);
                    PointF p2 = WorldToScreen(xWorld, WorldHeightM);

                    Pen pen = (i % 5 == 0) ? thickPen : thinPen;            // 每5格画粗线
                    g.DrawLine(pen, p1, p2);
                }

                // 横线
                for (int j = 0; j <= GridCount; j++)
                {
                    double yWorld = j * CellSizeM;
                    PointF p1 = WorldToScreen(0, yWorld);
                    PointF p2 = WorldToScreen(WorldWidthM, yWorld);

                    Pen pen = (j % 5 == 0) ? thickPen : thinPen;
                    g.DrawLine(pen, p1, p2);
                }
            }
        }
    }
}