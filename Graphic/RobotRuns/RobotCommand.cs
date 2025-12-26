using System;

namespace GridDemo.RobotRuns
{
    internal enum EnumRobotCommandType
    {
        MoveDistance,   // 前进多少位移（米）
        Turn            // 转向（左/右/或转到指定方向）
    }

    internal enum EnumTurnCommand
    {
        Left,
        Right,
        ToDirection
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
            EnumTurnCommand? turn,
            EnumMoveDirection? targetDirection)
        {
            Type = type;
            DistanceM = distanceM;
            Turn = turn;
            TargetDirection = targetDirection;
        }

        public EnumRobotCommandType Type { get; }
        public double? DistanceM { get; }
        public EnumTurnCommand? Turn { get; }
        public EnumMoveDirection? TargetDirection { get; }

        public static RobotCommand MoveDistance(double distanceM)
        {
            if (distanceM < 0) throw new ArgumentOutOfRangeException(nameof(distanceM));
            return new RobotCommand(EnumRobotCommandType.MoveDistance, distanceM, null, null);
        }

        public static RobotCommand TurnLeft()
        {
            return new RobotCommand(EnumRobotCommandType.Turn, null, EnumTurnCommand.Left, null);
        }

        public static RobotCommand TurnRight()
        {
            return new RobotCommand(EnumRobotCommandType.Turn, null, EnumTurnCommand.Right, null);
        }

        public static RobotCommand TurnTo(EnumMoveDirection direction)
        {
            return new RobotCommand(EnumRobotCommandType.Turn, null, EnumTurnCommand.ToDirection, direction);
        }
    }
}