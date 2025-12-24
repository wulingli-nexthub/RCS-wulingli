using GridDemo.WorldView;
using SkiaSharp;
using System;

namespace GridDemo.Draws
{
    /// <summary>
    /// 绘制机器人当前位置和朝向
    /// 当前绘制效果：
    /// - 红色圆点：机器人本体（半径随缩放变化，保证不同缩放下视觉大小相对稳定）
    /// - 黑色描边：圆形轮廓
    /// - 黑色方向箭头：表示机器人朝向（由 <see cref="_getOrientationAngle"/> 提供的弧度角决定）
    /// </summary>
    internal class DrawRobot
    {
        private readonly WorldTransform _transform;
        private readonly object _robotLock;     // 机器人状态锁：用于与仿真/控制线程同步读取位置与角度，避免绘制线程读到不一致状态。

        private readonly Func<(double X, double Y)> _getRobotPosition;           // 获取机器人位置的委托
        private readonly Func<double> _getScale;                      // 获取当前缩放比例，根据当前缩放调整机器人绘制大小
        private readonly Func<double> _getOrientationAngle;            // 获取机器人朝向角度（弧度制）的委托

        /// <summary>
        /// 创建机器人绘制器实例。
        /// </summary>
        internal DrawRobot(
            WorldTransform transform,
            object robotLock,
            Func<(double X, double Y)> getRobotPosition,
            Func<double> getScale,
            Func<double> getOrientationAngle)
        {
            _transform = transform;
            _robotLock = robotLock;
            _getRobotPosition = getRobotPosition;
            _getScale = getScale;
            _getOrientationAngle = getOrientationAngle;
        }

        /// <summary>
        /// 绘制机器人：读取当前位置与朝向，转换到屏幕坐标后绘制圆形与方向箭头。
        /// </summary>
        internal void Draw(SKCanvas canvas)
        {
            double robotX;
            double robotY;
            double angle;

            lock (_robotLock)                  // 锁定机器人状态，确保位置与角度一致
            {
                var pos = _getRobotPosition();
                robotX = pos.X;
                robotY = pos.Y;
                angle = _getOrientationAngle();
            }

            var screenPos = _transform.WorldToScreen(robotX, robotY);             // 转换到屏幕坐标
            double currentScale = _getScale();             // 获取当前缩放比例
            float radiusPx = (float)(currentScale / 6.0);   // 根据缩放比例计算机器人半径（像素）

            float cx = screenPos.X;            //机器人屏幕坐标
            float cy = screenPos.Y;

            using (var fill = new SKPaint              // 机器人样式：红色填充 + 黑色描边
            {
                Color = SKColors.Red,
                IsAntialias = true,
                Style = SKPaintStyle.Fill
            })
            using (var stroke = new SKPaint
            {
                Color = SKColors.Black,
                StrokeWidth = 1.5f,
                IsAntialias = true,
                Style = SKPaintStyle.Stroke
            })
            {
                canvas.DrawCircle(cx, cy, radiusPx, fill);
                canvas.DrawCircle(cx, cy, radiusPx, stroke);
            }

            // ----------------------------------------------方向箭头参数-------------------------------------//
            float arrowTotalLen = radiusPx * 1.8f;               // 箭头总长度
            float arrowStartOffset = radiusPx * 0.3f;              // 箭头起始偏移（距离圆心）
            float arrowLineWidth = Math.Max(1.0f, radiusPx * 0.12f);                   // 箭头线宽

            float dirX = (float)Math.Cos(angle);            // 方向向量
            float dirY = (float)Math.Sin(angle);

            float x1 = cx + dirX * arrowStartOffset;             // 箭头起点
            float y1 = cy + dirY * arrowStartOffset;
            float x2 = cx + dirX * arrowTotalLen;                 // 箭头终点
            float y2 = cy + dirY * arrowTotalLen;

            using (var arrowPaint = new SKPaint              // 绘制方向箭头，线段表示朝向
            {
                Color = SKColors.Black,
                StrokeWidth = arrowLineWidth,
                IsAntialias = true,
                Style = SKPaintStyle.Stroke,
                StrokeCap = SKStrokeCap.Round
            })
            {
                canvas.DrawLine(x1, y1, x2, y2, arrowPaint);
            }
        }
    }
}