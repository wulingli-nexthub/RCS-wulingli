namespace Graphic.RobotRuns
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
    internal class Robot
    {
        public double Acc { get; set; }

        public double MaxSpeed { get; set; }

        public EnumMoveDirection Direction { get; set; }

        public Robot(double acc, double maxSpeed, EnumMoveDirection direction)
        {
            Acc = acc;
            MaxSpeed = maxSpeed;
            Direction = direction;
        }
    }
}
