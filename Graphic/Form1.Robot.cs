using System;
using System.Drawing;

namespace Graphic
{
    /// <summary>
    /// 机器人逻辑模型（键盘控制版）
    /// - 保留 EnumMoveDirection，表示大致朝向（用于显示/逻辑）
    /// - 实际运动使用连续角 _orientationAngle
    /// - 键盘控制三种动作：前进 / 左转 / 右转
    /// </summary>
    public enum EnumMoveDirection
    {
        Right,
        Left,
        Down,
        Up
    }

    public class Robot
    {
        private readonly object _lock = new object();

        public double X { get; private set; }
        public double Y { get; private set; }
        public double Speed { get; private set; }      // 当前线速度（m/s）
        public double MaxSpeed { get; private set; }   // 最大线速度（m/s）
        public double Acc { get; private set; }        // 线加速度（m/s²）

        /// <summary>当前朝向对应的“离散大方向”</summary>
        public EnumMoveDirection Direction { get; private set; }

        /// <summary>连续朝向角（弧度）：0 向右，顺时针为正</summary>
        private double _orientationAngle;

        // 世界边界
        private readonly double _worldWidthM;
        private readonly double _worldHeightM;
        private readonly double _cellSizeM;

        // 角速度（转向速度，rad/s）
        private readonly double _turnSpeed = Math.PI;  // 180°/s，可自行调整

        // ===== 键盘控制状态 =====
        private bool _isMovingForward;  // 是否正在前进

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
                Acc = 1.5;             // 默认加速度，你也可以用 UI 控制
                Direction = EnumMoveDirection.Right;

                _orientationAngle = 0.0;   // 初始朝右
            }
        }

        #region 对外接口（Form 调用）

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
                if (speed < 0) speed = 0;
                if (speed > MaxSpeed) speed = MaxSpeed;
                Speed = speed;
            }
        }

        public void SetMaxSpeed(double maxSpeed)
        {
            if (maxSpeed < 0) maxSpeed = 0;

            lock (_lock)
            {
                MaxSpeed = maxSpeed;
                if (Speed > MaxSpeed)
                    Speed = MaxSpeed;
            }
        }

        /// <summary>获取状态快照（多了一个朝向角度 thetaDeg）</summary>
        public (double X, double Y, double V, double A, double ThetaDeg, EnumMoveDirection Dir) GetState()
        {
            lock (_lock)
            {
                double thetaDeg = _orientationAngle * 180.0 / Math.PI;
                return (X, Y, Speed, Acc, thetaDeg, Direction);
            }
        }

        // === 键盘控制：三种动作 ===
        public void StartMoveForward()
        {
            lock (_lock) { _isMovingForward = true; }
        }

        public void StopMoveForward()
        {
            lock (_lock) { _isMovingForward = false; }
        }

        public void TurnLeft()
        {
            lock (_lock)
            {
                _orientationAngle -= Math.PI / 2.0;   // 逆时针 90°
                _orientationAngle = NormalizeAngle(_orientationAngle);
                UpdateDiscreteDirection();
            }
        }

        public void TurnRight()
        {
            lock (_lock)
            {
                _orientationAngle += Math.PI / 2.0;   // 顺时针 90°
                _orientationAngle = NormalizeAngle(_orientationAngle);
                UpdateDiscreteDirection();
            }
        }

        #endregion

        private static double NormalizeAngle(double angle)
        {
            while (angle <= -Math.PI) angle += 2.0 * Math.PI;
            while (angle > Math.PI) angle -= 2.0 * Math.PI;
            return angle;
        }

        /// <summary>
        /// 根据连续角度更新离散方向（用于保留 EnumMoveDirection）
        /// </summary>
        private void UpdateDiscreteDirection()
        {
            // 把角度映射到 [-PI, PI)
            double a = NormalizeAngle(_orientationAngle);

            // 按象限划分（简单实现，你也可以更精细）
            // -45°~45° => Right
            // 45°~135° => Down
            // -135°~-45° => Up
            // 其他 => Left
            double deg = a * 180.0 / Math.PI;

            if (deg >= -45 && deg < 45)
                Direction = EnumMoveDirection.Right;
            else if (deg >= 45 && deg < 135)
                Direction = EnumMoveDirection.Down;
            else if (deg >= -135 && deg < -45)
                Direction = EnumMoveDirection.Up;
            else
                Direction = EnumMoveDirection.Left;
        }

        /// <summary>
        /// Update：根据键盘状态更新位置和角度
        /// </summary>
        public void Update(double dt)
        {
            lock (_lock)
            {
                // 2. 前进/速度
                if (_isMovingForward)
                {
                    Speed += Acc * dt;
                    if (Speed < 0) Speed = 0;
                    if (Speed > MaxSpeed) Speed = MaxSpeed;
                }
                else
                {
                    // 松开前进键：立即停（也可做缓慢减速）
                    Speed = 0;
                }

                // 3. 按朝向和速度更新位置
                if (Speed > 0)
                {
                    double dx = Speed * Math.Cos(_orientationAngle) * dt;
                    double dy = Speed * Math.Sin(_orientationAngle) * dt;

                    X += dx;
                    Y += dy;

                    // 4. 边界夹住，避免走出世界
                    double half = _cellSizeM / 2.0;
                    if (X < half) X = half;
                    if (Y < half) Y = half;
                    if (X > _worldWidthM - half) X = _worldWidthM - half;
                    if (Y > _worldHeightM - half) Y = _worldHeightM - half;
                }

                // 5. 更新离散方向枚举（保持 EnumMoveDirection 有意义）
                UpdateDiscreteDirection();
            }
        }

        /// <summary>
        /// 绘制机器人（与原代码基本一致，只是 orientationAngle 来源不同）
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
            float radiusPx = (float)(grid.Scale / 6);

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

            // 画方向箭头
            float arrowTotalLen = radiusPx * 1.8f;
            float arrowStartOffset = radiusPx * 0.3f;
            float arrowLineWidth = Math.Max(1.0f, radiusPx * 0.12f);

            float dirX = (float)Math.Cos(orientationAngle);
            float dirY = (float)Math.Sin(orientationAngle);

            PointF pStart = new PointF(
                screenPos.X + dirX * arrowStartOffset,
                screenPos.Y + dirY * arrowStartOffset);

            PointF pEnd = new PointF(
                screenPos.X + dirX * arrowTotalLen,
                screenPos.Y + dirY * arrowTotalLen);

            using (var arrowPen = new Pen(Color.Black, arrowLineWidth))
            {
                arrowPen.StartCap = System.Drawing.Drawing2D.LineCap.Round;
                arrowPen.EndCap = System.Drawing.Drawing2D.LineCap.Round;
                g.DrawLine(arrowPen, pStart, pEnd);
            }
        }
    }
}