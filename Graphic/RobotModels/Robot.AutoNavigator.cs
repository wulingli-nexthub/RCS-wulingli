using GridDemo.RobotModels.Pathfinding;
using GridDemo.RobotRuns;
using System;
using System.Collections.Generic;

namespace GridDemo.RobotModels
{
    /// <summary>
    /// 机器人自动导航器（仅负责“规划路径 + 生成下一条离散指令”）：
    /// - 规划：世界坐标离散为网格，使用寻路算法生成从当前位置到目标格子的路径；
    /// - 输出：每次最多生成一条 <see cref="RobotCommand"/>（两种格式：位移/转向）；
    /// - 不直接驱动加速度/速度/按键状态，不直接调用 Move/Turn（业务执行由 Robot 负责）。
    /// </summary>
    internal sealed class RobotAutoNavigator
    {
        private readonly object _robotLock;
        private readonly Func<double> _getRobotX;
        private readonly Func<double> _getRobotY;

        private readonly Func<double> _getWorldWidthM;
        private readonly Func<double> _getWorldHeightM;

        private readonly double _cellSizeM;
        private readonly Robot _robot;

        private const double ArriveEpsilonM = 0.05;

        private readonly List<GridPos> _path = new List<GridPos>();
        private int _pathIndex;

        private EnumPathfindingAlgorithm _algorithm = EnumPathfindingAlgorithm.AStar;
        private GridPos? _goal;

        public RobotAutoNavigator(
            object robotLock,
            Func<double> getRobotX,
            Func<double> getRobotY,
            Action<double> setRobotSpeed, // 兼容旧构造参数：目前不再使用（可后续删除）
            Func<double> getWorldWidthM,
            Func<double> getWorldHeightM,
            double cellSizeM,
            Robot robot)
        {
            _robotLock = robotLock ?? throw new ArgumentNullException(nameof(robotLock));
            _getRobotX = getRobotX ?? throw new ArgumentNullException(nameof(getRobotX));
            _getRobotY = getRobotY ?? throw new ArgumentNullException(nameof(getRobotY));
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
                    _path.Clear();
                    _pathIndex = 0;
                }
            }
        }

        public void SetGoal(GridPos goal, bool rebuildIfEnabled)
        {
            lock (_robotLock)
            {
                _goal = goal;

                _path.Clear();
                _pathIndex = 0;

                if (rebuildIfEnabled && IsEnabled)
                {
                    RebuildPath_NoLock();
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

                _path.Clear();
                _pathIndex = 0;

                RebuildPath_NoLock();

                // 仅做内部状态同步（不驱动运动）
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
            }
        }

        public List<(double X, double Y)> GetPathWorldPointsSnapshot()
        {
            lock (_robotLock)
            {
                var points = new List<(double X, double Y)>(_path.Count);
                for (int i = 0; i < _path.Count; i++)
                {
                    GridPos p = _path[i];
                    points.Add((GridToCenterWorldX(p.X), GridToCenterWorldY(p.Y)));
                }

                return points;
            }
        }

        public int GetPathIndexSnapshot()
        {
            lock (_robotLock)
            {
                return _pathIndex;
            }
        }

        /// <summary>
        /// 生成下一条要执行的指令（若暂时没有则返回 null）。
        /// 规则：
        /// - 未启用/未设目标/无路径：返回 null；
        /// - 已到达当前路径点：推进索引后再判断；
        /// - 需要调整方向：返回 TurnTo(...)；
        /// - 方向已对齐：返回 MoveDistance(到“下一格中心”的轴向剩余距离)。
        /// </summary>
        public RobotCommand TryBuildNextCommand()
        {
            lock (_robotLock)
            {
                if (!IsEnabled)
                {
                    return null;
                }

                if (!_goal.HasValue)
                {
                    return null;
                }

                if (_path.Count == 0)
                {
                    RebuildPath_NoLock();
                }

                if (_path.Count == 0)
                {
                    return null;
                }

                double x = _getRobotX();
                double y = _getRobotY();

                AdvanceWaypointIfArrived_NoLock(x, y);

                if (_pathIndex >= _path.Count)
                {
                    _path.Clear();
                    _pathIndex = 0;
                    return null;
                }

                // 当前目标点
                GridPos first = _path[_pathIndex];
                double firstX = GridToCenterWorldX(first.X);
                double firstY = GridToCenterWorldY(first.Y);

                EnumMoveDirection desiredDir = ChooseDirectionToTarget(x, y, firstX, firstY);

                // 需要转向：先转向（只输出 Turn 指令）
                if (!_robot.IsTurning && desiredDir != _robot.Direction)
                {
                    return RobotCommand.TurnTo(desiredDir);
                }

                // 已对齐方向：把同方向连续段合并成一个 MoveDistance 指令
                // 1) 先计算从当前位置到第一个点的轴向剩余距离
                double total = 0.0;

                if (desiredDir == EnumMoveDirection.Right || desiredDir == EnumMoveDirection.Left)
                {
                    total += Math.Abs(firstX - x);
                }
                else
                {
                    total += Math.Abs(firstY - y);
                }

                // 2) 向后看，把“下一段仍为同方向”的格子中心距离叠加
                int i = _pathIndex;
                while (i + 1 < _path.Count)
                {
                    GridPos cur = _path[i];
                    GridPos next = _path[i + 1];

                    // 判断 cur->next 的离散方向
                    int dx = next.X - cur.X;
                    int dy = next.Y - cur.Y;

                    EnumMoveDirection stepDir;
                    if (Math.Abs(dx) >= Math.Abs(dy))
                    {
                        stepDir = dx >= 0 ? EnumMoveDirection.Right : EnumMoveDirection.Left;
                    }
                    else
                    {
                        stepDir = dy >= 0 ? EnumMoveDirection.Down : EnumMoveDirection.Up;
                    }

                    if (stepDir != desiredDir)
                    {
                        break;
                    }

                    // 每个格中心距离就是 cellSizeM（严格来说如果 path 有斜跳，这里要用 dx/dy*cellSizeM，但你的 path 是 4-neighbor）
                    total += _cellSizeM;
                    i++;
                }

                if (total <= 0.000001)
                {
                    return null;
                }

                return RobotCommand.MoveDistance(total);
            }
        }

        private void RebuildPath_NoLock()
        {
            double worldWidth = _getWorldWidthM();
            double worldHeight = _getWorldHeightM();

            int gridW = Math.Max(1, (int)Math.Round(worldWidth / _cellSizeM));
            int gridH = Math.Max(1, (int)Math.Round(worldHeight / _cellSizeM));

            GridPos start = WorldToGrid(_getRobotX(), _getRobotY(), gridW, gridH);

            if (!_goal.HasValue)
            {
                _path.Clear();
                _pathIndex = 0;
                return;
            }

            GridPos goal = _goal.Value;

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