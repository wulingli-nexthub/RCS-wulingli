using GridDemo.Robots;
using System.Collections.Generic;

namespace GridDemo.MultiRobots
{
    internal readonly struct MultiRobotStateSnapshot
    {
        public MultiRobotStateSnapshot(IReadOnlyList<RobotItemSnapshot> robots)
        {
            Robots = robots;
        }

        public IReadOnlyList<RobotItemSnapshot> Robots { get; }
    }

    internal readonly struct RobotItemSnapshot
    {
        public RobotItemSnapshot(
            int id,
            double x,
            double y,
            double speed,
            double acc,
            double orientationAngle,
            EnumMoveDirection direction,
            bool isSelected,
            bool hasGoal,
            int goalGridX,
            int goalGridY,
            double goalWorldX,
            double goalWorldY)
        {
            Id = id;
            X = x;
            Y = y;
            Speed = speed;
            Acc = acc;
            OrientationAngle = orientationAngle;
            Direction = direction;
            IsSelected = isSelected;
            HasGoal = hasGoal;
            GoalGridX = goalGridX;
            GoalGridY = goalGridY;
            GoalWorldX = goalWorldX;
            GoalWorldY = goalWorldY;
        }

        public int Id { get; }
        public double X { get; }
        public double Y { get; }
        public double Speed { get; }
        public double Acc { get; }
        public double OrientationAngle { get; }
        public EnumMoveDirection Direction { get; }

        public bool IsSelected { get; }
        public bool HasGoal { get; }
        public int GoalGridX { get; }
        public int GoalGridY { get; }
        public double GoalWorldX { get; }
        public double GoalWorldY { get; }
    }
}