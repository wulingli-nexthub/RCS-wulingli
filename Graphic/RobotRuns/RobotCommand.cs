using System;

namespace GridDemo.RobotRuns
{
    internal enum EnumRobotCommandType
    {
        MoveDistance,   // 前进多少位移（米）
        Turn            // 转向（左/右/或转到指定方向）
    }

    /// <summary>
    /// 机器人指令（格式固定两种）：
    /// 1) 前进位移（米）
    /// 2) 转向（左/右/或转到指定方向）
    /// </summary>
    internal sealed class RobotCommand
    {
        private RobotCommand(
            EnumRobotCommandType type,
            double? distanceM,
            double? turnAngleRad)
        {
            Type = type;
            DistanceM = distanceM;
            TurnAngleRad = turnAngleRad;
        }

        public EnumRobotCommandType Type { get; }
        public double? DistanceM { get; }
        public double? TurnAngleRad { get; }
        //public EnumTurnCommand? Turn { get; }
        //public EnumMoveDirection? TargetDirection { get; }

        public static RobotCommand MoveDistance(double distanceM)
        {
            if (distanceM < 0) throw new ArgumentOutOfRangeException(nameof(distanceM));
            return new RobotCommand(EnumRobotCommandType.MoveDistance, distanceM, null);
        }

        // 统一转向指令：相对角度
        public static RobotCommand TurnAngle(double angleRad)
        {
            return new RobotCommand(EnumRobotCommandType.Turn, null, angleRad);
        }
    }
}