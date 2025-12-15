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

        // 世界边界
        private readonly double _worldWidthM;
        private readonly double _worldHeightM;
        private readonly double _cellSizeM;

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
            }
        }

        /// <summary>
        /// 绘制机器人：
        /// - 使用 DrawGrid 提供的世界→屏幕转换
        /// - 半径与当前缩放成比例（Scale/6）
        /// </summary>
        public void Draw(Graphics g, DrawGrid grid)
        {
            double x, y;
            lock (_lock)
            {
                x = X;
                y = Y;
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
        }
    }
}
