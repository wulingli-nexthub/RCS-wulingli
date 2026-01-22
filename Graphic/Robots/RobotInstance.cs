using Graphic.Maps;
using GridDemo.RobotModels;
using GridDemo.RobotModels.Pathfinding;
using GridDemo.RobotRuns;
using System;

namespace GridDemo.Robots
{
    /// <summary>
    /// 单个机器人运行实例（多机器人引擎内部使用）。
    /// 将单机器人时期的 Manager/Move/Auto/Manual/状态聚合到一起，便于批量管理。
    /// </summary>
    internal sealed class RobotInstance
    {
        private readonly object _robotLock;
        private readonly ObstacleMap _obstacleMap;
        private readonly int _gridCount;
        private readonly double _cellSizeM;

        public RobotInstance(
            int id,
            object robotLock,
            ObstacleMap obstacleMap,
            int gridCount,
            double cellSizeM,
            double dt,
            double worldWidthM,
            double worldHeightM,
            double initialMaxSpeed,
            EnumMoveDirection initialDirection,
            double initialX,
            double initialY)
        {
            Id = id;
            _robotLock = robotLock ?? throw new ArgumentNullException(nameof(robotLock));
            _obstacleMap = obstacleMap ?? throw new ArgumentNullException(nameof(obstacleMap));
            _gridCount = gridCount;
            _cellSizeM = cellSizeM;

            X = initialX;
            Y = initialY;
            Speed = 0.0;
            Acc = 0.0;

            Manager = new RobotManager(
                acc: Acc,
                maxSpeed: initialMaxSpeed,
                direction: initialDirection);

            Move = new RobotMove(
                robotLock: _robotLock,
                getRobotX: () => X,
                setRobotX: v => X = v,
                getRobotY: () => Y,
                setRobotY: v => Y = v,
                getRobotSpeed: () => Speed,
                setRobotSpeed: v => Speed = v,
                robotManager: Manager,
                getWorldWidthM: () => worldWidthM,
                getWorldHeightM: () => worldHeightM,
                cellSizeM: cellSizeM,
                dt: dt,
                isWorldWalkable: (wx, wy) =>
                {
                    int gx = (int)Math.Floor(wx / _cellSizeM);
                    int gy = (int)Math.Floor(wy / _cellSizeM);

                    if (gx < 0 || gy < 0 || gx >= _gridCount || gy >= _gridCount)
                    {
                        return false;
                    }

                    return !_obstacleMap.IsObstacle(new GridPos(gx, gy));
                });

            AutoNavigator = new RobotAutoNavigator(
                robotLock: _robotLock,
                getRobotX: () => X,
                getRobotY: () => Y,
                setRobotSpeed: v => Speed = v,
                getWorldWidthM: () => worldWidthM,
                getWorldHeightM: () => worldHeightM,
                cellSizeM: cellSizeM,
                robotManager: Manager);

            Manual = new RobotManual(
                robotLock: _robotLock,
                robotManager: Manager);

            Manager.BindRuntime(
                robotLock: _robotLock,
                move: Move,
                turn: Move.TurnController,
                autoCommandProvider: () => AutoNavigator.TryBuildNextCommand(),
                getForwardAcc: () => Acc);

            Manager.BindManualCommandProvider(() => Manual.TryBuildNextCommand());
        }

        public int Id { get; }

        public double X { get; set; }
        public double Y { get; set; }
        public double Speed { get; set; }
        public double Acc { get; set; }

        public RobotManager Manager { get; }
        public RobotMove Move { get; }
        public RobotAutoNavigator AutoNavigator { get; }
        public RobotManual Manual { get; }

        public RobotStateSnapshot GetSnapshot()
        {
            return new RobotStateSnapshot(
                X: X,
                Y: Y,
                Speed: Speed,
                Acc: Acc,
                OrientationAngle: Manager.OrientationAngle);
        }

        public GridPos GetGridPos_NoLock()
        {
            int gx = (int)Math.Floor(X / _cellSizeM);
            int gy = (int)Math.Floor(Y / _cellSizeM);

            if (gx < 0) gx = 0;
            if (gy < 0) gy = 0;
            if (gx >= _gridCount) gx = _gridCount - 1;
            if (gy >= _gridCount) gy = _gridCount - 1;

            return new GridPos(gx, gy);
        }
    }
}