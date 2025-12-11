using System.Drawing;
using System.Windows.Forms;

namespace Graphic
{
    public partial class Form1
    {
        // 世界坐标(米) -> 屏幕坐标(像素)
        private PointF WorldToScreen(double wx, double wy)
        {
            float sx = (float)(wx * _scale + _offsetX);
            float sy = (float)(wy * _scale + _offsetY);
            return new PointF(sx, sy);
        }

        // 屏幕 -> 世界
        private PointF ScreenToWorld(float sx, float sy)
        {
            float wx = (float)((sx - _offsetX) / _scale);
            float wy = (float)((sy - _offsetY) / _scale);
            return new PointF(wx, wy);
        }

        /// <summary>
        /// GDI+绘图核心：Paint事件
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Form1_Paint(object sender, PaintEventArgs e)
        {
            Graphics g = e.Graphics;
            //抗锯齿
            g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;

            //清屏
            g.Clear(Color.White);

            //绘制网格（世界坐标 → 屏幕坐标）
            using (var thinPen = new Pen(Color.LightGray, 1f))
            using (var thickPen = new Pen(Color.Gray, 1.5f))
            {
                // 画竖线
                for (int i = 0; i <= GridCount; i++)
                {
                    double xWorld = i * CellSizeM;
                    PointF p1 = WorldToScreen(xWorld, 0);
                    PointF p2 = WorldToScreen(xWorld, _worldHeightM);

                    Pen pen = (i % 5 == 0) ? thickPen : thinPen;            //区分每五格线
                    g.DrawLine(pen, p1, p2);
                }

                // 画横线
                for (int j = 0; j <= GridCount; j++)
                {
                    double yWorld = j * CellSizeM;
                    PointF p1 = WorldToScreen(0, yWorld);
                    PointF p2 = WorldToScreen(_worldWidthM, yWorld);

                    Pen pen = (j % 5 == 0) ? thickPen : thinPen;
                    g.DrawLine(pen, p1, p2);
                }
            }

            //画机器人
            DrawRobot(g);

            //在左上角显示当前数据信息
            using (var font = new Font("宋体", 10))
            using (var brush = new SolidBrush(Color.Black))
            {
                double robotX, robotY, robotV, robotA;
                lock (_robotLock)
                {
                    robotX = _robotX;
                    robotY = _robotY;
                    robotV = _robotSpeed;
                    robotA = _robotAcc;
                }

                string info = $"Scale: {_scale:F1} px/m   Offset: ({_offsetX:F0}, {_offsetY:F0})";
                string infoRobot = $"Robot: x={robotX:F2}m, y={robotY:F2}m, v={robotV:F2}m/s, a={robotA:F2}m/s2";
                g.DrawString(info, font, brush, new PointF(10, 10));
                g.DrawString(infoRobot, font, brush, new PointF(10, 25));
            }
        }

        /// <summary>
        /// 画机器人（世界坐标 → 屏幕坐标），一个实心的圆形
        /// </summary>
        /// <param name="g"></param>
        private void DrawRobot(Graphics g)
        {
            double robotX, robotY;
            lock (_robotLock)
            {
                robotX = _robotX;
                robotY = _robotY;
            }

            PointF screenPos = WorldToScreen(robotX, robotY);

            // 为保证机器人清晰，将像素设置为缩放大小的四分之一
            float radiusPx = (float)(_scale / 6);

            RectangleF rect = new RectangleF(
                screenPos.X - radiusPx,
                screenPos.Y - radiusPx,
                radiusPx * 2,
                radiusPx * 2);

            using (var brush = new SolidBrush(Color.Red))
            using (var pen = new Pen(Color.Black, 1.5f))
            {
                g.FillEllipse(brush, rect);
                g.DrawEllipse(pen, rect);
            }
        }
    }
}
