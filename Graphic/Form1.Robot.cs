using System;
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
        public double MaxSpeed { get; private set; } // m/s，最大速度
        public double Acc { get; private set; }    // m/s^2，标量

        public EnumMoveDirection Direction { get; private set; }   // 当前运动方向（右→下→左→上循环）。
        public double _orientationAngle; // 方向角，度为单位，0度向右，顺时针增加。
        public double _targetAngle;

        // 世界边界
        private readonly double _worldWidthM;
        private readonly double _worldHeightM;
        private readonly double _cellSizeM;
        private readonly double _turnSpeed = Math.PI; // rad/s，转向速度

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
                MaxSpeed = 1.5;
                Acc = 0.0;
                Direction = EnumMoveDirection.Right;

                // 初始时面向右侧
                _orientationAngle = 0.0;
                _targetAngle = 0.0;
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
                if (speed < 0)
                {
                    speed = 0;
                }

                // 避免把速度设置到超过当前最大速度
                if (speed > MaxSpeed)
                {
                    speed = MaxSpeed;
                }

                Speed = speed;
            }
        }

        /// <summary>
        /// 设置最大速度。如果当前速度大于新的最大速度，会被立刻夹到该上限；
        /// 否则保持当前速度不变，由 Update 继续按加速度加速到新的上限。
        /// </summary>
        /// <param name="maxSpeed">新的最大速度（m/s），小于 0 会被当作 0 处理。</param>
        public void SetMaxSpeed(double maxSpeed)
        {
            if (maxSpeed < 0)
            {
                maxSpeed = 0;
            }

            lock (_lock)
            {
                MaxSpeed = maxSpeed;

                // 若当前速度超过新的上限，则立即夹到上限
                if (Speed > MaxSpeed)
                {
                    Speed = MaxSpeed;
                }
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

        /// <summary>
        /// 将离散方向枚举转换为目标朝向角度（弧度）
        /// </summary>
        private static double _DirectionToAngle(EnumMoveDirection direction)
        {
            switch (direction)
            {
                case EnumMoveDirection.Right:
                    return 0.0;
                case EnumMoveDirection.Down:
                    return Math.PI / 2.0;
                case EnumMoveDirection.Left:
                    return Math.PI;
                case EnumMoveDirection.Up:
                    return -Math.PI / 2.0;
                default:
                    return 0.0;
            }
        }

        /// <summary>
        /// 将角度归一化到 (-π, π] 区间，便于计算最短旋转路径
        /// </summary>
        private static double _NormalizeAngle(double angle)
        {
            while (angle <= -Math.PI)
            {
                angle += 2.0 * Math.PI;
            }

            while (angle > Math.PI)
            {
                angle -= 2.0 * Math.PI;
            }

            return angle;
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
                if (Speed > MaxSpeed) Speed = MaxSpeed;

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

                // 3. 边界检测 + 改变离散方向
                bool directionChanged = false;

                switch (Direction)
                {
                    case EnumMoveDirection.Right:
                        if (X >= _worldWidthM - _cellSizeM / 2.0)
                        {
                            X = _worldWidthM - _cellSizeM / 2.0;
                            Direction = EnumMoveDirection.Down;
                            directionChanged = true;
                        }

                        break;

                    case EnumMoveDirection.Down:
                        if (Y >= _worldHeightM - _cellSizeM / 2.0)
                        {
                            Y = _worldHeightM - _cellSizeM / 2.0;
                            Direction = EnumMoveDirection.Left;
                            directionChanged = true;
                        }

                        break;

                    case EnumMoveDirection.Left:
                        if (X <= _cellSizeM / 2.0)
                        {
                            X = _cellSizeM / 2.0;
                            Direction = EnumMoveDirection.Up;
                            directionChanged = true;
                        }

                        break;

                    case EnumMoveDirection.Up:
                        if (Y <= _cellSizeM / 2.0)
                        {
                            Y = _cellSizeM / 2.0;
                            Direction = EnumMoveDirection.Right;
                            directionChanged = true;
                        }

                        break;
                }

                // 4. 如果方向改变，更新目标角度
                if (directionChanged)
                {
                    _targetAngle = _DirectionToAngle(Direction);
                    _targetAngle = _NormalizeAngle(_targetAngle);
                }

                // 5. 将当前朝向角度朝目标角度平滑旋转（转向动画）
                // 计算当前与目标之间的最短角度差
                double delta = _NormalizeAngle(_targetAngle - _orientationAngle);

                // 本帧最大可旋转角度
                double maxStep = _turnSpeed * dt;

                if (Math.Abs(delta) <= maxStep)
                {
                    // 已经很接近，直接对齐
                    _orientationAngle = _targetAngle;
                }
                else
                {
                    // 按固定角速度向目标旋转
                    if (delta > 0)
                    {
                        _orientationAngle += maxStep;
                    }
                    else
                    {
                        _orientationAngle -= maxStep;
                    }

                    _orientationAngle = _NormalizeAngle(_orientationAngle);
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
            double x, y, orientationAngle;
            lock (_lock)
            {
                x = X;
                y = Y;
                orientationAngle = _orientationAngle;
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
                g.FillEllipse(brush, rect);
                g.DrawEllipse(pen, rect);
            }

            // ===== 画方向箭头：从圆心伸出一条细黑线，并在末端画小箭头头部 =====

            // 箭头总长度：略长于圆半径
            float arrowTotalLen = radiusPx * 1.8f;
            // 箭头从圆内起始的位置（略偏外一点，看起来像从圆中伸出）
            float arrowStartOffset = radiusPx * 0.3f;
            float arrowLineWidth = Math.Max(1.0f, radiusPx * 0.12f);

            // 面向方向单位向量
            float dirX = (float)Math.Cos(orientationAngle);
            float dirY = (float)Math.Sin(orientationAngle);

            // 箭头起点：圆心沿方向正向偏移一段距离
            PointF pStart = new PointF(
                screenPos.X + dirX * arrowStartOffset,
                screenPos.Y + dirY * arrowStartOffset);

            // 箭头终点：比圆半径略长
            PointF pEnd = new PointF(
                screenPos.X + dirX * arrowTotalLen,
                screenPos.Y + dirY * arrowTotalLen);

            using (var arrowPen = new Pen(Color.Black, arrowLineWidth))
            {
                arrowPen.EndCap = System.Drawing.Drawing2D.LineCap.Round;
                arrowPen.StartCap = System.Drawing.Drawing2D.LineCap.Round;

                // 主体箭头线
                g.DrawLine(arrowPen, pStart, pEnd);
            }
        }
    }
}
