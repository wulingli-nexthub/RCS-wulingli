using GridDemo.WorldView;
using SkiaSharp;
using System;
using System.Collections.Generic;

namespace GridDemo.Draws
{
    /// <summary>
    /// 绘制机器人当前位置和朝向（多机器人版本）：
    /// - 红色圆点：机器人本体
    /// - 选中机器人：描边加粗/颜色强调
    /// - 黑色方向箭头：朝向
    /// - 数字标签：机器人编号（1..N）
    /// </summary>
    internal class DrawRobot
    {
        private readonly WorldTransform _transform;
        private readonly object _robotLock;

        private readonly Func<List<(int Id, double X, double Y, double Angle)>> _getRobotsSnapshot;
        private readonly Func<int> _getSelectedId;
        private readonly Func<double> _getScale;

        internal DrawRobot(
            WorldTransform transform,
            object robotLock,
            Func<List<(int Id, double X, double Y, double Angle)>> getRobotsSnapshot,
            Func<int> getSelectedId,
            Func<double> getScale)
        {
            _transform = transform;
            _robotLock = robotLock;
            _getRobotsSnapshot = getRobotsSnapshot ?? throw new ArgumentNullException(nameof(getRobotsSnapshot));
            _getSelectedId = getSelectedId ?? throw new ArgumentNullException(nameof(getSelectedId));
            _getScale = getScale ?? throw new ArgumentNullException(nameof(getScale));
        }

        /// <summary>
        /// 在指定画布上绘制机器人位置和朝向
        /// </summary>
        internal void Draw(SKCanvas canvas)
        {
            List<(int Id, double X, double Y, double Angle)> robots; // 机器人快照列表（每个元素包含 Id、坐标、朝向角）
            int selectedId; // 当前选中的机器人 Id

            lock (_robotLock) // 对共享机器人状态加锁，防止绘制时被其他线程修改
            {
                robots = _getRobotsSnapshot(); // 在锁内读取机器人状态快照
                selectedId = _getSelectedId(); // 在锁内读取选中机器人 Id
            }

            if (robots == null || robots.Count == 0)
            { // 没有机器人可绘制则直接返回
                return;
            }

            double currentScale = _getScale(); // 读取当前缩放比例（用于决定显示尺寸）
            float radiusPx = (float)(currentScale / 6.0); // 将缩放换算为机器人圆点半径（像素）

            for (int i = 0; i < robots.Count; i++)
            { // 遍历所有机器人逐个绘制
                var r = robots[i]; // 取出第 i 个机器人快照
                var screenPos = _transform.WorldToScreen(r.X, r.Y); // 将世界坐标转换为屏幕坐标

                float cx = screenPos.X; // 圆心 X（屏幕坐标）
                float cy = screenPos.Y; // 圆心 Y（屏幕坐标）

                bool isSelected = r.Id == selectedId; // 判断该机器人是否为当前选中机器人

                using (var fill = new SKPaint // 创建“填充”画笔（用于画实心圆）
                {
                    Color = isSelected ? SKColors.OrangeRed : SKColors.Red, // 选中时用更醒目的橙红，否则用红色
                    IsAntialias = true, // 开启抗锯齿以减少边缘锯齿
                    Style = SKPaintStyle.Fill // 填充样式
                })
                using (var stroke = new SKPaint // 创建“描边”画笔（用于画圆形边框）
                {
                    Color = isSelected ? SKColors.Gold : SKColors.Black, // 选中时边框金色，否则黑色
                    StrokeWidth = isSelected ? 3.0f : 1.5f, // 选中时线宽更粗
                    IsAntialias = true, // 开启抗锯齿
                    Style = SKPaintStyle.Stroke // 描边样式
                })
                {
                    canvas.DrawCircle(cx, cy, radiusPx, fill); // 绘制机器人本体（填充圆）
                    canvas.DrawCircle(cx, cy, radiusPx, stroke); // 绘制机器人外圈（描边圆）
                }

                float arrowTotalLen = radiusPx * 1.8f; // 方向箭头线段总长度（相对半径进行缩放）
                float arrowStartOffset = radiusPx * 0.3f; // 箭头起点离圆心的偏移（避免从圆心穿出）
                float arrowLineWidth = Math.Max(1.0f, radiusPx * 0.12f); // 箭头线宽（最小 1px，随半径变化）

                float dirX = (float)Math.Cos(r.Angle); // 朝向单位向量 X（Angle 为弧度）
                float dirY = (float)Math.Sin(r.Angle); // 朝向单位向量 Y（Angle 为弧度）

                float x1 = cx + dirX * arrowStartOffset; // 箭头线起点 X
                float y1 = cy + dirY * arrowStartOffset; // 箭头线起点 Y
                float x2 = cx + dirX * arrowTotalLen; // 箭头线终点 X
                float y2 = cy + dirY * arrowTotalLen; // 箭头线终点 Y

                using (var arrowPaint = new SKPaint // 创建方向箭头画笔
                {
                    Color = SKColors.Black, // 箭头颜色为黑色
                    StrokeWidth = arrowLineWidth, // 箭头线宽
                    IsAntialias = true, // 开启抗锯齿
                    Style = SKPaintStyle.Stroke, // 使用描边绘制线段
                    StrokeCap = SKStrokeCap.Round // 线帽圆角，使线段端点更平滑
                })
                {
                    canvas.DrawLine(x1, y1, x2, y2, arrowPaint); // 绘制朝向线段（方向箭头主体）
                }

                using (var textPaint = new SKPaint { Color = SKColors.Black, IsAntialias = true }) // 文本画笔（用于编号绘制）
                using (var font = new SKFont { Size = Math.Max(10.0f, radiusPx * 0.9f) }) // 字体对象：字号随半径缩放，最小 10px
                {
                    canvas.DrawText((r.Id + 1).ToString(), cx + radiusPx, cy - radiusPx, SKTextAlign.Left, font, textPaint); // 在机器人右上方绘制编号（显示为 1..N）
                }
            }
        }
    }
}