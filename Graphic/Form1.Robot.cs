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

        private double _orientationAngle;  // 当前朝向角度（弧度制，连续）
        private double _targetAngle;  // 目标朝向角度（弧度制，连续）

        // 世界边界
        private readonly double _worldWidthM;
        private readonly double _worldHeightM;
        private readonly double _cellSizeM;

        // 角速度（转向速度，rad/s）
        private readonly double _turnSpeed = Math.PI;  // 180°/s，可自行调整

        // ===== 键盘控制状态 =====
        private bool _isMovingForward;  // 是否正在前进

        /// <summary>
        /// 构造函数，初始化机器人在世界中的位置与运动参数
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
                X = _cellSizeM / 2;      // 初始位置在左上角第一个格子中心
                Y = _cellSizeM / 2;
                Speed = 0.0;
                MaxSpeed = 1.5;
                Acc = 0.0;
                Direction = EnumMoveDirection.Right;

                _orientationAngle = 0.0;   // 初始朝右
                _targetAngle = 0.0;
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
                // 目标角度在当前目标的基础上左转 90°
                _targetAngle -= Math.PI / 2.0;
                _targetAngle = NormalizeAngle(_targetAngle);
            }
        }

        public void TurnRight()
        {
            lock (_lock)
            {
                // 目标角度在当前目标的基础上右转 90°
                _targetAngle += Math.PI / 2.0;
                _targetAngle = NormalizeAngle(_targetAngle);
            }
        }

        #endregion

        /// <summary>
        /// 角度归一化到 (-π, π] 区间，避免数值越来越大或比较困难
        /// </summary>
        /// <param name="angle">任意弧度制角度</param>
        /// <returns>等价的、位于 (-π, π] 的角度</returns>
        private static double NormalizeAngle(double angle)
        {
            while (angle <= -Math.PI) angle += 2.0 * Math.PI;
            while (angle > Math.PI) angle -= 2.0 * Math.PI;
            return angle;
        }

        /// <summary>
        /// 根据连续角度 _orientationAngle 更新离散方向枚举 Direction
        /// 方便在 UI 或逻辑中按大方向使用 EnumMoveDirection
        /// </summary>
        private void UpdateDiscreteDirection()
        {
            // 把角度映射到 [-PI, PI)
            double a = NormalizeAngle(_orientationAngle);

            // 按象限划分
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
        /// 更新机器人状态（由定时器/游戏循环每帧调用）
        /// - 平滑转向到目标角度
        /// - 处理前进/停止逻辑与速度变化
        /// - 按当前朝向与速度更新位置
        /// - 做世界边界裁剪
        /// - 更新离散方向枚举
        /// </summary>
        /// <param name="dt">时间步长（秒），即距离上次 Update 的时间</param>
        public void Update(double dt)
        {
            lock (_lock)
            {
                double delta = NormalizeAngle(_targetAngle - _orientationAngle);                // 1. 朝目标角度平滑旋转

                double maxStep = _turnSpeed * dt;   // 本帧最大可转角度

                if (Math.Abs(delta) <= maxStep)
                {
                    _orientationAngle = _targetAngle;                    // 已经接近目标，直接对齐

                }
                else
                {
                    if (delta > 0)                    // 按固定角速度向目标旋转

                        _orientationAngle += maxStep;
                    else
                        _orientationAngle -= maxStep;

                    _orientationAngle = NormalizeAngle(_orientationAngle);
                }

                if (_isMovingForward)   // 2. 前进/速度
                {
                    Speed += Acc * dt;
                    if (Speed < 0) Speed = 0;
                    if (Speed > MaxSpeed) Speed = MaxSpeed;
                }
                else
                {
                    Speed = 0;                    // 松开前进键：立即停
                }

                if (Speed > 0)                // 3. 按朝向和速度更新位置
                {
                    double dx = Speed * Math.Cos(_orientationAngle) * dt;
                    double dy = Speed * Math.Sin(_orientationAngle) * dt;

                    X += dx;
                    Y += dy;

                    double half = _cellSizeM / 2.0;                    // 4. 边界夹住，避免走出世界
                    if (X < half) X = half;
                    if (Y < half) Y = half;
                    if (X > _worldWidthM - half) X = _worldWidthM - half;
                    if (Y > _worldHeightM - half) Y = _worldHeightM - half;
                }

                UpdateDiscreteDirection();                // 5. 更新离散方向枚举

            }
        }

        /// <summary>
        /// 绘制机器人：
        /// - 在网格坐标系下将世界坐标转换为屏幕坐标
        /// - 画一个圆表示机器人本体
        /// - 画一条线表示当前朝向箭头
        /// </summary>
        /// <param name="g">绘图对象（来自 OnPaint 或 Paint 事件）</param>
        /// <param name="grid">网格对象，用于世界坐标到屏幕坐标转换</param>
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
            float radiusPx = (float)(grid.Scale / 6);           // 机器人半径，约为格子大小的1/3

            RectangleF rect = new RectangleF(         // 机器人本体为圆形
                screenPos.X - radiusPx,
                screenPos.Y - radiusPx,
                radiusPx * 2,
                radiusPx * 2);

            using (var brush = new SolidBrush(Color.Red))              // 画出机器人主体
            using (var pen = new Pen(Color.Black, 1.5f))
            {
                g.FillEllipse(brush, rect);
                g.DrawEllipse(pen, rect);
            }

            // ===== 画方向箭头 =====
            float arrowTotalLen = radiusPx * 1.8f;      // 箭头总长度
            float arrowStartOffset = radiusPx * 0.3f;         // 箭头起点距离圆心的偏移
            float arrowLineWidth = Math.Max(1.0f, radiusPx * 0.12f);      // 箭头线宽

            float dirX = (float)Math.Cos(orientationAngle);            // 根据朝向角计算单位方向向量
            float dirY = (float)Math.Sin(orientationAngle);

            PointF pStart = new PointF(          //箭头起点
                screenPos.X + dirX * arrowStartOffset,
                screenPos.Y + dirY * arrowStartOffset);

            PointF pEnd = new PointF(         // 箭头终点
                screenPos.X + dirX * arrowTotalLen,
                screenPos.Y + dirY * arrowTotalLen);

            using (var arrowPen = new Pen(Color.Black, arrowLineWidth))           // 画箭头线
            {
                arrowPen.StartCap = System.Drawing.Drawing2D.LineCap.Round;
                arrowPen.EndCap = System.Drawing.Drawing2D.LineCap.Round;
                g.DrawLine(arrowPen, pStart, pEnd);
            }
        }
    }
}