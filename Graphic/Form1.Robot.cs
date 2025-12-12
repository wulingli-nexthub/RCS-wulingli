using System.Drawing;

namespace Graphic
{
    /// <summary>
    /// 机器人移动方向
    /// </summary>
    public enum EnumMoveDirection
    {
        Right,
        Left,
        Down,
        Up
    }

    /// <summary>
    /// 机器人逻辑模型
    /// - 只负责维护自身的物理状态（位置、速度、加速度、方向）
    /// - 提供 Update(dt) 推进一小步运动
    /// - 提供 Draw(...) 根据当前世界->屏幕变换绘制自身
    /// - 内部加锁，支持被后台线程更新、UI 线程读取
    /// </summary>
    public class Robot
    {
        private readonly object _lock = new object();   // 内部锁对象，保护 X/Y/Speed/Acc/Direction 的并发访问。

        public double X { get; private set; }
        public double Y { get; private set; }
        public double Speed { get; private set; }  // m/s，标量
        public double Acc { get; private set; }    // m/s^2，标量

        public EnumMoveDirection Direction { get; private set; }   // 当前运动方向（右→下→左→上循环）。

        // 世界边界
        private readonly double _worldWidthM;
        private readonly double _worldHeightM;
        private readonly double _cellSizeM;

        public EnumMoveDirection TargetDirection { get; private set; }

        public double HeadingAngleDeg { get; private set; }

        /// <summary>
        /// 构造函数，读取世界尺寸和单元格大小，确定机器人初始位置/速度/方向
        /// </summary>
        /// <param name="cellSizeM"></param>
        /// <param name="worldWidthM"></param>
        /// <param name="worldHeightM"></param>
        public Robot(double cellSizeM, double worldWidthM, double worldHeightM)
        {
            _cellSizeM = cellSizeM;
            _worldWidthM = worldWidthM;
            _worldHeightM = worldHeightM;

            lock (_lock)
            {
                X = _cellSizeM / 2;
                Y = _cellSizeM / 2;
                Speed = 0.0;
                Acc = 0.0;
                Direction = EnumMoveDirection.Right;
                TargetDirection = EnumMoveDirection.Right;
                HeadingAngleDeg = 0.0;
            }
        }
        public void TurnLeft()
        {
            lock (_lock)
            {
                switch (Direction)
                {
                    case EnumMoveDirection.Right:
                        TargetDirection = EnumMoveDirection.Up;
                        break;
                    case EnumMoveDirection.Up:
                        TargetDirection = EnumMoveDirection.Left;
                        break;
                    case EnumMoveDirection.Left:
                        TargetDirection = EnumMoveDirection.Down;
                        break;
                    case EnumMoveDirection.Down:
                        TargetDirection = EnumMoveDirection.Right;
                        break;
                    default:
                        // 保持原有 TargetDirection
                        break;
                }
            }
        }

        public void TurnRight()
        {
            lock (_lock)
            {
                switch (Direction)
                {
                    case EnumMoveDirection.Right:
                        TargetDirection = EnumMoveDirection.Down;
                        break;
                    case EnumMoveDirection.Up:
                        TargetDirection = EnumMoveDirection.Right;
                        break;
                    case EnumMoveDirection.Left:
                        TargetDirection = EnumMoveDirection.Up;
                        break;
                    case EnumMoveDirection.Down:
                        TargetDirection = EnumMoveDirection.Left;
                        break;
                    default:
                        // 保持原有 TargetDirection
                        break;
                }
            }
        }

        public void StartMoving(double speed)
        {
            lock (_lock)
            {
                Speed = 1.5;
            }
        }

        public void StopMoving()
        {
            lock (_lock)
            {
                Speed = 0.0;
            }
        }

        public void SetAcc(double acc)
        {
            lock (_lock)
            {
                Acc = acc;
            }
        }

        public void SetSpeed(double speed)
        {
            lock (_lock)
            {
                Speed = speed;
            }
        }

        /// <summary>
        /// 获取当前状态快照
        /// </summary>
        /// <returns></returns>
        public (double X, double Y, double V, double A) GetState()
        {
            lock (_lock)
            {
                return (X, Y, Speed, Acc);
            }
        }

        private static double DirectionToAngle(EnumMoveDirection dir)
        {
            switch (dir)
            {
                case EnumMoveDirection.Right: return 0.0;
                case EnumMoveDirection.Down: return 90.0;
                case EnumMoveDirection.Left: return 180.0;
                case EnumMoveDirection.Up: return 270.0;
                default: return 0.0;
            }
        }

        /// <summary>
        /// 按固定时间步长更新机器人状态
        /// - 根据 Acc 更新 Speed
        /// - 根据 Direction+Speed 更新位置 X/Y
        /// - 到达边界时切换方向（右→下→左→上→右），并将位置夹在世界边缘处
        /// </summary>
        public void Update(double dt)
        {
            lock (_lock)
            {
                Speed += Acc * dt;                // 更新速度
                if (Speed < 0) Speed = 0;

                switch (Direction)                // 按当前方向移动
                {
                    case EnumMoveDirection.Right:
                        X += Speed * dt;
                        break;
                    case EnumMoveDirection.Left:
                        X -= Speed * dt;
                        break;
                    case EnumMoveDirection.Down:
                        Y += Speed * dt;
                        break;
                    case EnumMoveDirection.Up:
                        Y -= Speed * dt;
                        break;
                }

                switch (Direction)                // 到达边界时转向（右→下→左→上→右）
                {
                    case EnumMoveDirection.Right:
                        if (X >= _worldWidthM - _cellSizeM / 2)
                        {
                            X = _worldWidthM - _cellSizeM / 2;
                            Direction = EnumMoveDirection.Down;
                        }
                        break;

                    case EnumMoveDirection.Down:
                        if (Y >= _worldHeightM - _cellSizeM / 2)
                        {
                            Y = _worldHeightM - _cellSizeM / 2;
                            Direction = EnumMoveDirection.Left;
                        }
                        break;

                    case EnumMoveDirection.Left:
                        if (X <= _cellSizeM / 2)
                        {
                            X = _cellSizeM / 2;
                            Direction = EnumMoveDirection.Up;
                        }
                        break;

                    case EnumMoveDirection.Up:
                        if (Y <= _cellSizeM / 2)
                        {
                            Y = _cellSizeM / 2;
                            Direction = EnumMoveDirection.Right;
                        }
                        break;
                }

                // 4. 朝向动画：HeadingAngleDeg 向目标方向角度平滑逼近
                double targetAngle = DirectionToAngle(TargetDirection);
                // 归一化误差到 [-180, 180]
                double diff = targetAngle - HeadingAngleDeg;
                while (diff > 180.0) diff -= 360.0;
                while (diff < -180.0) diff += 360.0;

                // 每秒旋转 180°（半圈），你可以根据需要调整转向速度
                double turnSpeedDegPerSec = 180.0;
                double maxStep = turnSpeedDegPerSec * dt;

                if (System.Math.Abs(diff) <= maxStep)
                {
                    HeadingAngleDeg = targetAngle;
                    // 当朝向完成后，把真正的运动方向也切成目标方向
                    Direction = TargetDirection;
                }
                else
                {
                    HeadingAngleDeg += System.Math.Sign(diff) * maxStep;
                    // 保持在 0~360
                    if (HeadingAngleDeg < 0) HeadingAngleDeg += 360.0;
                    if (HeadingAngleDeg >= 360.0) HeadingAngleDeg -= 360.0;
                }
            }
        }

        /// <summary>
        /// 绘制机器人：
        /// - 使用 DrawGrid 提供的世界→屏幕转换
        /// - 半径与当前缩放成比例（Scale/6）
        /// </summary>
        public void Draw(Graphics g, DrawGrid grid)
        {
            double x, y, heading;
            {
                x = X;
                y = Y;
                heading = HeadingAngleDeg;
            }

            PointF screenPos = grid.WorldToScreen(x, y);

            float radiusPx = (float)(grid.Scale / 6);            // 让机器人半径与缩放成比例

            RectangleF rect = new RectangleF(
                screenPos.X - radiusPx,
                screenPos.Y - radiusPx,
                radiusPx * 2,
                radiusPx * 2);

            using (var brush = new SolidBrush(Color.Red))
            using (var pen = new Pen(Color.Black, 1.5f))
            {
                // 画圆形车身
                g.FillEllipse(brush, rect);
                g.DrawEllipse(pen, rect);

                // 画指示朝向的小箭头（在圆外稍微伸出一截）
                double rad = heading * System.Math.PI / 180.0;
                float arrowLen = radiusPx * 1.2f;

                var pHead = new PointF(
                    screenPos.X + (float)(System.Math.Cos(rad) * arrowLen),
                    screenPos.Y + (float)(System.Math.Sin(rad) * arrowLen));

                g.DrawLine(pen, screenPos, pHead);
            }
        }
    }
}
