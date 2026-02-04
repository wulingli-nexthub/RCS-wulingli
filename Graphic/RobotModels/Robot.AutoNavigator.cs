using GridDemo.RobotModels.Pathfinding;
using GridDemo.RobotRuns;
using GridDemo.Robots;
using System;
using System.Collections.Generic;

namespace GridDemo.RobotModels
{
    internal sealed class RobotAutoNavigator
    {
        private readonly object _robotLock;
        private readonly Func<double> _getRobotX;
        private readonly Func<double> _getRobotY;

        private readonly Func<double> _getWorldWidthM;
        private readonly Func<double> _getWorldHeightM;

        private readonly double _cellSizeM;
        private readonly RobotManager _robotManager;

        private const double ArriveEpsilonM = 0.05;

        private readonly List<GridPos> _path = new List<GridPos>();
        private int _pathIndex;

        // 新增：由引擎“格子锁”抢占得到的路径前缀（只允许沿该前缀出指令）
        private readonly List<GridPos> _claimedPathPrefix = new List<GridPos>();

        private EnumPathfindingAlgorithm _algorithm = EnumPathfindingAlgorithm.AStar;
        private GridPos? _goal;
        private Func<GridPos, bool> _isWalkableProvider = p => true;

        private readonly int _robotId;
        private readonly int _gridW;
        private readonly int _gridH;
        private readonly Func<Dictionary<int, int>> _getGoalOwnerMap;

        public RobotAutoNavigator(
            object robotLock,
            Func<double> getRobotX,
            Func<double> getRobotY,
            Action<double> setRobotSpeed,
            Func<double> getWorldWidthM,
            Func<double> getWorldHeightM,
            double cellSizeM,
            RobotManager robotManager,
            int robotId,
            int gridCount,
            Func<Dictionary<int, int>> getGoalOwnerMap)
        {
            _robotLock = robotLock ?? throw new ArgumentNullException(nameof(robotLock));
            _getRobotX = getRobotX ?? throw new ArgumentNullException(nameof(getRobotX));
            _getRobotY = getRobotY ?? throw new ArgumentNullException(nameof(getRobotY));
            _getWorldWidthM = getWorldWidthM ?? throw new ArgumentNullException(nameof(getWorldWidthM));
            _getWorldHeightM = getWorldHeightM ?? throw new ArgumentNullException(nameof(getWorldHeightM));
            _cellSizeM = cellSizeM;
            _robotManager = robotManager ?? throw new ArgumentNullException(nameof(robotManager));

            _robotId = robotId;
            _gridW = gridCount;
            _gridH = gridCount;
            _getGoalOwnerMap = getGoalOwnerMap ?? throw new ArgumentNullException(nameof(getGoalOwnerMap));
        }

        public void SetIsWalkableProvider(Func<GridPos, bool> isWalkableProvider)
        {
            lock (_robotLock)
            {
                _isWalkableProvider = isWalkableProvider ?? throw new ArgumentNullException(nameof(isWalkableProvider));
            }
        }

        public bool IsEnabled { get; private set; }

        public void Enable()
        {
            lock (_robotLock)
            {
                IsEnabled = true;

                _path.Clear();
                _claimedPathPrefix.Clear();
                _pathIndex = 0;

                RebuildPath_NoLock();
            }
        }

        public void Disable()
        {
            lock (_robotLock)
            {
                IsEnabled = false;

                _path.Clear();
                _claimedPathPrefix.Clear();
                _pathIndex = 0;
            }
        }

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
                    _claimedPathPrefix.Clear();
                    _pathIndex = 0;

                    _robotManager.ResetAutoCommands();
                }
            }
        }

        public void SetGoal(GridPos goal, bool rebuildIfEnabled)
        {
            lock (_robotLock)
            {
                _goal = goal;

                _path.Clear();
                _claimedPathPrefix.Clear();
                _pathIndex = 0;

                if (rebuildIfEnabled && IsEnabled)
                {
                    RebuildPath_NoLock();
                    _robotManager.ResetAutoCommands();
                }
            }
        }

        public (double X, double Y)? GetGoalWorldSnapshot()
        {
            lock (_robotLock)
            {
                if (!_goal.HasValue)
                {
                    return null;
                }

                GridPos g = _goal.Value;
                return (GridToCenterWorldX(g.X), GridToCenterWorldY(g.Y));
            }
        }

        public void ClearGoal()
        {
            lock (_robotLock)
            {
                _goal = null;
                _path.Clear();
                _claimedPathPrefix.Clear();
                _pathIndex = 0;
            }
        }

        public void RebuildPath()
        {
            lock (_robotLock)
            {
                RebuildPath_NoLock();

                // 重新寻路后：抢占前缀由引擎下一帧重算并回灌，此处仅清空
                _claimedPathPrefix.Clear();

                if (IsEnabled)
                {
                    _robotManager.ResetAutoCommands();
                }
            }
        }

        /// <summary>
        /// 引擎层“格子锁”抢占后，将“我拥有的路径前缀”回灌到导航器。
        /// 注意：这里不负责抢占逻辑，只存储结果，用于后续出指令。
        /// </summary>
        public void SetClaimedPathPrefix(List<GridPos> claimedPrefix)
        {
            lock (_robotLock)
            {
                _claimedPathPrefix.Clear();

                if (claimedPrefix == null || claimedPrefix.Count == 0)
                {
                    return;
                }

                _claimedPathPrefix.AddRange(claimedPrefix);
            }
        }

        /// <summary>
        /// 自动指令生成（基于“已抢占的路径前缀”）：抢不到下一步就返回 null（上层选择等待/停车）。
        /// </summary>
        public RobotCommand TryBuildNextCommandFromClaimedPath()
        {
            lock (_robotLock)
            {
                if (!IsEnabled || !_goal.HasValue)
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

                if (_claimedPathPrefix.Count == 0)
                {
                    return null; // 没抢到任何路径前缀：必须等待
                }

                double x = _getRobotX();
                double y = _getRobotY();

                AdvanceWaypointIfArrived_Claimed_NoLock(x, y);

                // 已经走完已抢占前缀：等待下一帧抢占更多
                if (_pathIndex >= _claimedPathPrefix.Count)
                {
                    return null;
                }

                GridPos first = _claimedPathPrefix[_pathIndex];
                double firstX = GridToCenterWorldX(first.X);
                double firstY = GridToCenterWorldY(first.Y);
                EnumMoveDirection desiredDir = ChooseDirectionToTarget(x, y, firstX, firstY);

                if (!_robotManager.IsTurning && desiredDir != _robotManager.Direction)
                {
                    double? turnAngle = TryGetTurnAngleRad(_robotManager.Direction, desiredDir);
                    if (turnAngle.HasValue)
                    {
                        return RobotCommand.TurnAngle(turnAngle.Value);
                    }

                    return null;
                }

                double total = 0.0;

                if (desiredDir == EnumMoveDirection.Right || desiredDir == EnumMoveDirection.Left)
                {
                    total += Math.Abs(firstX - x);
                }
                else
                {
                    total += Math.Abs(firstY - y);
                }

                int i = _pathIndex;
                while (i + 1 < _claimedPathPrefix.Count)
                {
                    GridPos cur = _claimedPathPrefix[i];
                    GridPos next = _claimedPathPrefix[i + 1];

                    if (GetStepDirection(cur, next) != desiredDir)
                    {
                        break;
                    }

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

        /// <summary>
        /// 旧接口保留：仍可用，但引擎自动模式应改用 TryBuildNextCommandFromClaimedPath()。
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
                    _claimedPathPrefix.Clear();
                    _pathIndex = 0;
                    return null;
                }

                GridPos first = _path[_pathIndex];
                double firstX = GridToCenterWorldX(first.X);
                double firstY = GridToCenterWorldY(first.Y);
                EnumMoveDirection desiredDir = ChooseDirectionToTarget(x, y, firstX, firstY);

                if (!_robotManager.IsTurning && desiredDir != _robotManager.Direction)
                {
                    double? turnAngle = TryGetTurnAngleRad(_robotManager.Direction, desiredDir);
                    if (turnAngle.HasValue)
                    {
                        return RobotCommand.TurnAngle(turnAngle.Value);
                    }

                    return null;
                }

                double total = 0.0;

                if (desiredDir == EnumMoveDirection.Right || desiredDir == EnumMoveDirection.Left)
                {
                    total += Math.Abs(firstX - x);
                }
                else
                {
                    total += Math.Abs(firstY - y);
                }

                int i = _pathIndex;
                while (i + 1 < _path.Count)
                {
                    GridPos cur = _path[i];
                    GridPos next = _path[i + 1];

                    if (GetStepDirection(cur, next) != desiredDir)
                    {
                        break;
                    }

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
                _claimedPathPrefix.Clear();
                _pathIndex = 0;
                return;
            }

            GridPos goal = _goal.Value;

            Func<GridPos, bool> baseWalkable = _isWalkableProvider ?? (p => true);

            Dictionary<int, int> goalOwnerMap = null;
            try
            {
                goalOwnerMap = _getGoalOwnerMap?.Invoke();
            }
            catch
            {
                goalOwnerMap = null;
            }

            Func<GridPos, bool> isWalkable = p =>
            {
                if (!baseWalkable(p))
                    return false;

                if (goalOwnerMap != null && goalOwnerMap.Count > 0)
                {
                    int key = p.Y * _gridW + p.X;
                    int ownerId;
                    if (goalOwnerMap.TryGetValue(key, out ownerId) && ownerId != _robotId)
                    {
                        return false;
                    }
                }

                return true;
            };

            List<GridPos> path;
            if (_algorithm == EnumPathfindingAlgorithm.Serpentine)
            {
                path = SerpentinePathfinder.BuildPath(
                    width: gridW,
                    height: gridH,
                    start: start,
                    goal: goal,
                    isWalkable: isWalkable);
            }
            else
            {
                path = GridPathfinder.FindPath(
                    width: gridW,
                    height: gridH,
                    start: start,
                    goal: goal,
                    isWalkable: isWalkable,
                    algorithm: _algorithm);
            }

            _path.Clear();
            _path.AddRange(path);

            // 重建路径后：抢占前缀需要重新计算（由引擎做），这里先清空
            _claimedPathPrefix.Clear();

            _pathIndex = 0;
            if (_path.Count > 0 && _path[0].Equals(start))
            {
                _pathIndex = Math.Min(1, _path.Count);
            }
        }

        private static EnumMoveDirection GetStepDirection(GridPos cur, GridPos next)
        {
            int dx = next.X - cur.X;
            int dy = next.Y - cur.Y;

            if (Math.Abs(dx) >= Math.Abs(dy))
            {
                return dx >= 0 ? EnumMoveDirection.Right : EnumMoveDirection.Left;
            }

            return dy >= 0 ? EnumMoveDirection.Down : EnumMoveDirection.Up;
        }

        private static double? TryGetTurnAngleRad(EnumMoveDirection currentDir, EnumMoveDirection targetDir)
        {
            if (currentDir == targetDir)
            {
                return null;
            }

            int cur = (int)currentDir;
            int des = (int)targetDir;

            int rightSteps = (des - cur + 4) % 4;
            int leftSteps = (cur - des + 4) % 4;

            if (rightSteps <= leftSteps)
            {
                return +Math.PI / 2.0 * rightSteps;
            }

            return -Math.PI / 2.0 * leftSteps;
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

                if (arriveX && arriveY)
                {
                    _pathIndex++;
                    continue;
                }

                if (_pathIndex > 0)
                {
                    GridPos prev = _path[_pathIndex - 1];

                    int dxCell = p.X - prev.X;
                    int dyCell = p.Y - prev.Y;

                    double prevX = GridToCenterWorldX(prev.X);
                    double prevY = GridToCenterWorldY(prev.Y);

                    if (dxCell != 0)
                    {
                        double dir = Math.Sign(tx - prevX);
                        if (dir != 0)
                        {
                            double robotProj = (robotX - prevX) * dir;
                            double targetProj = (tx - prevX) * dir;
                            if (robotProj >= targetProj)
                            {
                                _pathIndex++;
                                continue;
                            }
                        }
                    }
                    else if (dyCell != 0)
                    {
                        double dir = Math.Sign(ty - prevY);
                        if (dir != 0)
                        {
                            double robotProj = (robotY - prevY) * dir;
                            double targetProj = (ty - prevY) * dir;
                            if (robotProj >= targetProj)
                            {
                                _pathIndex++;
                                continue;
                            }
                        }
                    }
                }

                break;
            }
        }

        // 新增：按“已抢占前缀”推进索引（避免把 _pathIndex 推进到未抢占段）
        private void AdvanceWaypointIfArrived_Claimed_NoLock(double robotX, double robotY)
        {
            int limit = Math.Min(_claimedPathPrefix.Count, _path.Count);

            while (_pathIndex < limit)
            {
                GridPos p = _claimedPathPrefix[_pathIndex];
                double tx = GridToCenterWorldX(p.X);
                double ty = GridToCenterWorldY(p.Y);

                bool arriveX = Math.Abs(robotX - tx) <= ArriveEpsilonM;
                bool arriveY = Math.Abs(robotY - ty) <= ArriveEpsilonM;

                if (arriveX && arriveY)
                {
                    _pathIndex++;
                    continue;
                }

                if (_pathIndex > 0)
                {
                    GridPos prev = _claimedPathPrefix[_pathIndex - 1];

                    int dxCell = p.X - prev.X;
                    int dyCell = p.Y - prev.Y;

                    double prevX = GridToCenterWorldX(prev.X);
                    double prevY = GridToCenterWorldY(prev.Y);

                    if (dxCell != 0)
                    {
                        double dir = Math.Sign(tx - prevX);
                        if (dir != 0)
                        {
                            double robotProj = (robotX - prevX) * dir;
                            double targetProj = (tx - prevX) * dir;
                            if (robotProj >= targetProj)
                            {
                                _pathIndex++;
                                continue;
                            }
                        }
                    }
                    else if (dyCell != 0)
                    {
                        double dir = Math.Sign(ty - prevY);
                        if (dir != 0)
                        {
                            double robotProj = (robotY - prevY) * dir;
                            double targetProj = (ty - prevY) * dir;
                            if (robotProj >= targetProj)
                            {
                                _pathIndex++;
                                continue;
                            }
                        }
                    }
                }

                break;
            }
        }

        public List<(double X, double Y)> GetPathWorldPointsSnapshot()
        {
            lock (_robotLock)
            {
                // 绘制仍使用“完整路径”（你也可以改成绘制 claimed 前缀，看需求）
                var points = new List<(double X, double Y)>(_path.Count);
                for (int i = 0; i < _path.Count; i++)
                {
                    GridPos p = _path[i];
                    points.Add((GridToCenterWorldX(p.X), GridToCenterWorldY(p.Y)));
                }

                return points;
            }
        }

        public List<GridPos> GetPathGridSnapshot()
        {
            lock (_robotLock)
            {
                // 引擎抢占用“完整路径”，不要用 claimed（claimed 是抢占结果）
                var list = new List<GridPos>(_path.Count);
                for (int i = 0; i < _path.Count; i++)
                {
                    list.Add(_path[i]);
                }
                return list;
            }
        }

        private GridPos WorldToGrid(double wx, double wy, int gridW, int gridH)
        {
            int gx = (int)Math.Floor(wx / _cellSizeM);
            int gy = (int)Math.Floor(wy / _cellSizeM);

            if (gx < 0)
            {
                gx = 0;
            }
            if (gy < 0)
            {
                gy = 0;
            }
            if (gx >= gridW)
            {
                gx = gridW - 1;
            }
            if (gy >= gridH)
            {
                gy = gridH - 1;
            }
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

        public GridPos? GetGoalGridSnapshot()
        {
            lock (_robotLock)
            {
                return _goal;
            }
        }

        public bool TryGetNextStepSnapshot(out GridPos currentCell, out GridPos nextCell, out EnumMoveDirection desiredDir)
        {
            lock (_robotLock)
            {
                currentCell = default(GridPos);
                nextCell = default(GridPos);
                desiredDir = default(EnumMoveDirection);

                if (!IsEnabled || !_goal.HasValue)
                {
                    return false;
                }

                if (_path.Count == 0)
                {
                    RebuildPath_NoLock();
                }

                if (_path.Count == 0)
                {
                    return false;
                }

                double x = _getRobotX();
                double y = _getRobotY();

                // 若有抢占前缀，则 nextStep 以抢占前缀为准
                if (_claimedPathPrefix.Count > 0)
                {
                    AdvanceWaypointIfArrived_Claimed_NoLock(x, y);

                    if (_pathIndex >= _claimedPathPrefix.Count)
                    {
                        return false;
                    }
                }
                else
                {
                    AdvanceWaypointIfArrived_NoLock(x, y);

                    if (_pathIndex >= _path.Count)
                    {
                        return false;
                    }
                }

                double worldWidth = _getWorldWidthM();
                double worldHeight = _getWorldHeightM();
                int gridW = Math.Max(1, (int)Math.Round(worldWidth / _cellSizeM));
                int gridH = Math.Max(1, (int)Math.Round(worldHeight / _cellSizeM));
                currentCell = WorldToGrid(x, y, gridW, gridH);

                nextCell = (_claimedPathPrefix.Count > 0) ? _claimedPathPrefix[_pathIndex] : _path[_pathIndex];

                int dx = nextCell.X - currentCell.X;
                int dy = nextCell.Y - currentCell.Y;

                if (dx == 0 && dy == 0)
                {
                    double tx = GridToCenterWorldX(nextCell.X);
                    double ty = GridToCenterWorldY(nextCell.Y);
                    desiredDir = ChooseDirectionToTarget(x, y, tx, ty);
                    return true;
                }

                if (Math.Abs(dx) >= Math.Abs(dy))
                {
                    desiredDir = dx >= 0 ? EnumMoveDirection.Right : EnumMoveDirection.Left;
                }
                else
                {
                    desiredDir = dy >= 0 ? EnumMoveDirection.Down : EnumMoveDirection.Up;
                }

                return true;
            }
        }

        public List<GridPos> BuildPathSnapshotFromCurrentToGoal()
        {
            lock (_robotLock)
            {
                if (!IsEnabled || !_goal.HasValue)
                {
                    return null;
                }

                double worldWidth = _getWorldWidthM();
                double worldHeight = _getWorldHeightM();

                int gridW = Math.Max(1, (int)Math.Round(worldWidth / _cellSizeM));
                int gridH = Math.Max(1, (int)Math.Round(worldHeight / _cellSizeM));

                GridPos start = WorldToGrid(_getRobotX(), _getRobotY(), gridW, gridH);
                GridPos goal = _goal.Value;

                Func<GridPos, bool> isWalkable = _isWalkableProvider ?? (p => true);

                if (_algorithm == EnumPathfindingAlgorithm.Serpentine)
                {
                    return SerpentinePathfinder.BuildPath(
                        width: gridW,
                        height: gridH,
                        start: start,
                        goal: goal,
                        isWalkable: isWalkable);
                }

                return GridPathfinder.FindPath(
                    width: gridW,
                    height: gridH,
                    start: start,
                    goal: goal,
                    isWalkable: isWalkable,
                    algorithm: _algorithm);
            }
        }
    }
}