using Graphic.RobotModels.Pathfinding;
using Graphic.RobotRuns;
using System;
using System.Collections.Generic;

namespace Graphic.RobotModels
{
    internal sealed class RobotAutoNavigator
    {
        private readonly object _robotLock;
        private readonly Func<double> _getRobotX;
        private readonly Func<double> _getRobotY;
        private readonly Action<double> _setRobotSpeed;

        private readonly Func<double> _getWorldWidthM;
        private readonly Func<double> _getWorldHeightM;

        private readonly double _cellSizeM;
        private readonly Robot _robot;

        private const double ArriveEpsilonM = 0.02;

        private readonly List<GridPos> _path = new List<GridPos>();
        private int _pathIndex;

        private EnumPathfindingAlgorithm _algorithm = EnumPathfindingAlgorithm.AStar;

        public RobotAutoNavigator(
            object robotLock,
            Func<double> getRobotX,
            Func<double> getRobotY,
            Action<double> setRobotSpeed,
            Func<double> getWorldWidthM,
            Func<double> getWorldHeightM,
            double cellSizeM,
            Robot robot)
        {
            _robotLock = robotLock ?? throw new ArgumentNullException(nameof(robotLock));
            _getRobotX = getRobotX ?? throw new ArgumentNullException(nameof(getRobotX));
            _getRobotY = getRobotY ?? throw new ArgumentNullException(nameof(getRobotY));
            _setRobotSpeed = setRobotSpeed ?? throw new ArgumentNullException(nameof(setRobotSpeed));
            _getWorldWidthM = getWorldWidthM ?? throw new ArgumentNullException(nameof(getWorldWidthM));
            _getWorldHeightM = getWorldHeightM ?? throw new ArgumentNullException(nameof(getWorldHeightM));
            _cellSizeM = cellSizeM;
            _robot = robot ?? throw new ArgumentNullException(nameof(robot));
        }

        public bool IsEnabled { get; private set; }

        public EnumPathfindingAlgorithm Algorithm
        {
            get { return _algorithm; }
            set
            {
                lock (_robotLock)
                {
                    if (_algorithm == value)
                    {
                        return;
                    }

                    _algorithm = value;

                    // 算法变化后，清空路径，等待下一帧/或外部调用 RebuildPath 立即重算
                    _path.Clear();
                    _pathIndex = 0;
                }
            }
        }

        public void RebuildPath()
        {
            lock (_robotLock)
            {
                RebuildPath_NoLock();
            }
        }

        public void Enable()
        {
            lock (_robotLock)
            {
                IsEnabled = true;

                _robot.IsForwardKeyDown = true;

                RebuildPath_NoLock();

                double angle = Robot.DirectionToAngle(_robot.Direction);
                _robot.OrientationAngle = angle;
                _robot.TargetOrientationAngle = angle;
            }
        }

        public void Disable()
        {
            lock (_robotLock)
            {
                IsEnabled = false;

                _path.Clear();
                _pathIndex = 0;

                _robot.IsForwardKeyDown = false;
                _robot.Acc = 0.0;
                _setRobotSpeed(0.0);
            }
        }

        public RobotAutoMotionState GetAutoMotionState(Func<double> getForwardAcc)
        {
            if (getForwardAcc == null) throw new ArgumentNullException(nameof(getForwardAcc));

            lock (_robotLock)
            {
                if (!IsEnabled)
                {
                    return RobotAutoMotionState.Disabled;
                }

                if (_path.Count == 0)
                {
                    RebuildPath_NoLock();
                }

                if (_path.Count == 0)
                {
                    _robot.IsForwardKeyDown = false;
                    _robot.Acc = 0.0;
                    _setRobotSpeed(0.0);

                    return new RobotAutoMotionState(
                        enabled: true,
                        direction: _robot.Direction,
                        acc: 0.0,
                        suppressEdgeTurning: true,
                        clampOnBounds: true,
                        requestTurnLeft: false);
                }

                double x = _getRobotX();
                double y = _getRobotY();

                AdvanceWaypointIfArrived_NoLock(x, y);

                if (_pathIndex >= _path.Count)
                {
                    _robot.IsForwardKeyDown = false;
                    _robot.Acc = 0.0;
                    _setRobotSpeed(0.0);

                    return new RobotAutoMotionState(
                        enabled: true,
                        direction: _robot.Direction,
                        acc: 0.0,
                        suppressEdgeTurning: true,
                        clampOnBounds: true,
                        requestTurnLeft: false);
                }

                GridPos next = _path[_pathIndex];
                double targetX = GridToCenterWorldX(next.X);
                double targetY = GridToCenterWorldY(next.Y);

                EnumMoveDirection dir = ChooseDirectionToTarget(x, y, targetX, targetY);

                double acc = getForwardAcc();

                _robot.IsForwardKeyDown = true;
                _robot.Acc = acc;
                _robot.Direction = dir;

                double angle = Robot.DirectionToAngle(dir);
                _robot.OrientationAngle = angle;
                _robot.TargetOrientationAngle = angle;

                return new RobotAutoMotionState(
                    enabled: true,
                    direction: dir,
                    acc: acc,
                    suppressEdgeTurning: true,
                    clampOnBounds: true,
                    requestTurnLeft: false);
            }
        }

        private void RebuildPath_NoLock()
        {
            double worldWidth = _getWorldWidthM();
            double worldHeight = _getWorldHeightM();

            int gridW = Math.Max(1, (int)Math.Round(worldWidth / _cellSizeM));
            int gridH = Math.Max(1, (int)Math.Round(worldHeight / _cellSizeM));

            GridPos start = WorldToGrid(_getRobotX(), _getRobotY(), gridW, gridH);
            GridPos goal = new GridPos(gridW - 1, gridH - 1);

            Func<GridPos, bool> isWalkable = p => true;

            List<GridPos> path = GridPathfinder.FindPath(
                width: gridW,
                height: gridH,
                start: start,
                goal: goal,
                isWalkable: isWalkable,
                algorithm: _algorithm);

            _path.Clear();
            _path.AddRange(path);

            _pathIndex = 0;
            if (_path.Count > 0 && _path[0].Equals(start))
            {
                _pathIndex = Math.Min(1, _path.Count);
            }
        }

        private void AdvanceWaypointIfArrived_NoLock(double robotX, double robotY)
        {
            while (_pathIndex < _path.Count)
            {
                GridPos p = _path[_pathIndex];
                double tx = GridToCenterWorldX(p.X);
                double ty = GridToCenterWorldY(p.Y);

                bool arriveX = Math.Abs(robotX - tx) <= ArriveEpsilonM;
                bool arriveY = Math.Abs(robotY - ty) <= ArriveEpsilonM;

                if (!(arriveX && arriveY))
                {
                    break;
                }

                _pathIndex++;
            }
        }

        private GridPos WorldToGrid(double wx, double wy, int gridW, int gridH)
        {
            int gx = (int)Math.Floor(wx / _cellSizeM);
            int gy = (int)Math.Floor(wy / _cellSizeM);

            if (gx < 0) gx = 0;
            if (gy < 0) gy = 0;
            if (gx >= gridW) gx = gridW - 1;
            if (gy >= gridH) gy = gridH - 1;

            return new GridPos(gx, gy);
        }

        private double GridToCenterWorldX(int gx) => gx * _cellSizeM + _cellSizeM / 2.0;
        private double GridToCenterWorldY(int gy) => gy * _cellSizeM + _cellSizeM / 2.0;

        private static EnumMoveDirection ChooseDirectionToTarget(double x, double y, double tx, double ty)
        {
            double dx = tx - x;
            double dy = ty - y;

            if (Math.Abs(dx) >= Math.Abs(dy))
            {
                return dx >= 0 ? EnumMoveDirection.Right : EnumMoveDirection.Left;
            }

            return dy >= 0 ? EnumMoveDirection.Down : EnumMoveDirection.Up;
        }
    }
}