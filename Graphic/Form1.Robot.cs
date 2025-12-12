using System.Drawing;

namespace Graphic
{
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

        // 位置、速度、加速度
        public double X { get; private set; }
        public double Y { get; private set; }
        public double Speed { get; private set; }  // m/s
        public double Acc { get; private set; }    // m/s^2

        public EnumMoveDirection Direction { get; private set; }

        // 世界边界
        private readonly double _worldWidthM;
        private readonly double _worldHeightM;
        private readonly double _cellSizeM;

        public Robot(double cellSizeM, double worldWidthM, double worldHeightM)
        {
            _cellSizeM = cellSizeM;
            _worldWidthM = worldWidthM;
            _worldHeightM = worldHeightM;

            Reset();
        }

        public void Reset()
        {
            lock (_lock)
            {
                X = _cellSizeM / 2;
                Y = _cellSizeM / 2;
                Speed = 1.5;
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

        public (double X, double Y, double V, double A) GetState()
        {
            lock (_lock)
            {
                return (X, Y, Speed, Acc);
            }
        }

        /// <summary>
        /// 按固定时间步长更新机器人状态
        /// </summary>
        public void Update(double dt)
        {
            lock (_lock)
            {
                // 更新速度
                Speed += Acc * dt;
                if (Speed < 0) Speed = 0;

                // 按当前方向移动
                switch (Direction)
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

                // 到达边界时转向（右→下→左→上→右）
                switch (Direction)
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
        /// 绘制机器人（使用 DrawGrid 里的坐标转换）
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

            // 让机器人半径与缩放成比例
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
        }
    }
}
