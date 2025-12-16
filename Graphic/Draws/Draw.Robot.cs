using Graphic.WorldView;
using System;
using System.Drawing;

namespace Graphic.Draws
{
    internal class DrawRobot
    {
        private readonly WorldTransform _transform;
        private readonly object _robotLock;

        // 动态获取机器人位置和当前缩放
        private readonly Func<(double X, double Y)> _getRobotPosition;
        private readonly Func<double> _getScale;

        internal DrawRobot(
            WorldTransform transform,
            object robotLock,
            Func<(double X, double Y)> getRobotPosition,
            Func<double> getScale)
        {
            _transform = transform;
            _robotLock = robotLock;
            _getRobotPosition = getRobotPosition;
            _getScale = getScale;
        }

        internal void Robot(Graphics g)
        {
            double robotX;
            double robotY;

            lock (_robotLock)
            {
                var pos = _getRobotPosition();
                robotX = pos.X;
                robotY = pos.Y;
            }

            PointF screenPos = _transform.WorldToScreen(robotX, robotY);

            // 每次绘制时获取“当前缩放”
            double currentScale = _getScale();

            // 机器人半径 = 当前缩放 / 6
            float radiusPx = (float)(currentScale / 6.0);

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