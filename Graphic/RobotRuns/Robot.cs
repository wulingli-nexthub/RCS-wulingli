using System;

namespace GridDemo.RobotRuns
{
    public enum EnumMoveDirection
    {
        Right,
        Left,
        Down,
        Up
    }

    public enum EnumRobotStatus
    {
        Turning,
        Moving
    }

    /// <summary>
    /// 机器人基础运动模型（加速度/速度上限/方向/朝向）。
    /// 与渲染/控制分离：该类保存运动相关的状态与约定。
    /// </summary>
    internal class Robot
    {
        public double Acc { get; set; }

        public double MaxSpeed { get; set; }

        public EnumMoveDirection Direction { get; set; }

        /// <summary>
        /// 当前朝向角度（弧度，0 向右，顺时针为正）
        /// </summary>
        public double OrientationAngle { get; set; }

        /// <summary>
        /// 目标朝向角度（弧度），用于转向动画插值
        /// </summary>
        public double TargetOrientationAngle { get; set; }

        /// <summary>
        /// 是否正在做转向动画
        /// </summary>
        public bool IsTurning { get; set; }

        /// <summary>
        /// 转向速度（弧度/秒）
        /// </summary>
        public double TurnAngularSpeed { get; set; } = System.Math.PI; // 默认 180°/s
        // 当前是否在“试图前进”（W 是否按着）
        public bool IsForwardKeyDown { get; set; }

        /// <summary>
        /// 创建机器人并根据初始离散方向初始化朝向角度。
        /// </summary>
        public Robot(double acc, double maxSpeed, EnumMoveDirection direction)
        {
            Acc = acc;
            MaxSpeed = maxSpeed;
            Direction = direction;

            // 根据初始方向初始化角度
            OrientationAngle = DirectionToAngle(direction);
            TargetOrientationAngle = OrientationAngle;
        }

        /// <summary>
        /// 将离散方向转换为弧度角度（用于 <see cref="OrientationAngle"/> 的约定）。
        /// </summary>
        public static double DirectionToAngle(EnumMoveDirection dir)
        {
            switch (dir)
            {
                case EnumMoveDirection.Right: return 0;
                case EnumMoveDirection.Down: return Math.PI / 2;
                case EnumMoveDirection.Left: return Math.PI;
                case EnumMoveDirection.Up: return 3 * Math.PI / 2;
                default: return 0;
            }
        }
    }
}