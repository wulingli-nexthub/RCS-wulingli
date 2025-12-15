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

        /// <summary>
        /// 世界坐标->屏幕坐标
        /// </summary>
        /// <param name="wx">世界坐标X（米）</param>
        /// <param name="wy">世界坐标Y（米）</param>
        /// <returns>屏幕坐标点（像素）</returns>
        public PointF WorldToScreen(double wx, double wy)
        {
            float sx = (float)(wx * Scale + OffsetX);
            float sy = (float)(wy * Scale + OffsetY);
            return new PointF(sx, sy);
        }

        /// <summary>
        /// 屏幕坐标 -> 世界坐标。
        /// 和 WorldToScreen 互逆，通常用于拾取/缩放中心等操作。
        /// </summary>
        /// <param name="sx">屏幕 X（像素）</param>
        /// <param name="sy">屏幕 Y（像素）</param>
        /// <returns>世界坐标点（米）</returns>
        public PointF ScreenToWorld(float sx, float sy)
        {
            float wx = (float)((sx - OffsetX) / Scale);
            float wy = (float)((sy - OffsetY) / Scale);
            return new PointF(wx, wy);
        }

        /// <summary>
        /// 绘制网格：
        /// - 按当前 Scale / Offset 将世界坐标网格线映射到屏幕
        /// - 每 1 个 CellSizeM 画一条细线
        /// - 每 5 个格子画一条粗线（方便分辨大网格）
        /// </summary>
        /// <param name="g">GDI+ 画布对象</param>
        public void Draw(Graphics g)
        {
            using (var thinPen = new Pen(Color.LightGray, 1f))
            using (var thickPen = new Pen(Color.Gray, 1.5f))
            {
                for (int i = 0; i <= GridCount; i++)                // 竖线
                {
                    double xWorld = i * CellSizeM;
                    PointF p1 = WorldToScreen(xWorld, 0);
                    PointF p2 = WorldToScreen(xWorld, WorldHeightM);

                    Pen pen = (i % 5 == 0) ? thickPen : thinPen;            // 每5格画粗线
                    g.DrawLine(pen, p1, p2);
                }

                for (int j = 0; j <= GridCount; j++)                // 横线
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