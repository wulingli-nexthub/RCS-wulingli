using GridDemo.Models.Pathfinding;
using System;
using System.Collections.Generic;

namespace GridDemo.Models
{
    /// <summary>
    /// 机器人自动导航器：
    /// - 负责从“当前世界坐标”到“目标格子”生成路径（格子序列）。
    /// - 负责将路径转换为“转向/直行距离”的离散指令。
    /// - 支持引擎层的“格子锁抢占”：只允许沿“已抢占路径前缀”发出移动指令，抢不到就等待。
    /// 
    /// 线程模型：
    /// - 所有对内部状态（路径、目标、索引）的读写都必须在 <see cref="_robotLock"/> 下进行。
    /// </summary>
    internal sealed class RobotAutoNavigator
    {
        private readonly object _robotLock;
        private readonly Func<double> _getRobotX;
        private readonly Func<double> _getRobotY;

        private readonly Func<double> _getWorldWidthM;
        private readonly Func<double> _getWorldHeightM;

        private readonly double _cellSizeM;
        private readonly RobotManager _robotManager;

        private const double ArriveEpsilonM = 0.05;   // 判定到达格子中心的容差（米）
        private const double CenterLineEpsilonM = 0.01; // 判定“已对齐中心线”的容差（米）

        private readonly List<GridPos> _path = new List<GridPos>();
        private int _pathIndex;

        // 由引擎“格子锁”抢占得到的路径前缀（只允许沿该前缀出指令）
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

        /// <summary>
        /// 设置地形可通行性判断函数（例如障碍物图）。
        /// </summary>
        public void SetIsWalkableProvider(Func<GridPos, bool> isWalkableProvider)
        {
            lock (_robotLock)
            {
                _isWalkableProvider = isWalkableProvider ?? throw new ArgumentNullException(nameof(isWalkableProvider));
            }
        }

        public bool IsEnabled { get; private set; }

        /// <summary>
        /// 启用自动导航：清理旧状态并尝试立即构建路径。
        /// </summary>
        public void Enable()
        {
            lock (_robotLock)
            {
                IsEnabled = true;

                // 开启时重置路径与索引，避免复用过期路径 / 过期抢占结果
                _path.Clear();
                _claimedPathPrefix.Clear();
                _pathIndex = 0;

                RebuildPath_NoLock();
            }
        }

        /// <summary>
        /// 禁用自动导航：清理路径与抢占前缀，停止自动出指令。
        /// </summary>
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

        /// <summary>
        /// 当前寻路算法。切换算法会清路径并要求重新生成自动指令。
        /// </summary>
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
                    // 算法变更意味着旧路径失效
                    _path.Clear();
                    _claimedPathPrefix.Clear();
                    _pathIndex = 0;

                    // 自动指令队列也应作废，避免机器人沿旧策略继续走
                    _robotManager.ResetAutoCommands();
                }
            }
        }

        /// <summary>
        /// 设置目标格子；可选：若已经启用则立即重建路径并重置自动指令。
        /// </summary>
        public void SetGoal(GridPos goal, bool rebuildIfEnabled)
        {
            lock (_robotLock)
            {
                _goal = goal;

                // 目标变化 -> 路径/抢占都需重算
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

        /// <summary>
        /// 返回目标点“格子中心”的世界坐标快照，用于 UI 绘制/调试展示。
        /// </summary>
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

        /// <summary>
        /// 清空目标并清理路径状态。
        /// </summary>
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

        /// <summary>
        /// 外部触发重新寻路。
        /// 注意：抢占前缀属于“引擎层格子锁结果”，需要下一帧重新抢占回灌，因此这里重建路径后会清空抢占前缀。
        /// </summary>
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

                // 当 _pathIndex 超出新前缀长度时，重新定位到第一个尚未到达的格子
                // 避免前缀缩短后 _pathIndex 越界导致无法生成移向格心的指令
                if (_pathIndex >= _claimedPathPrefix.Count)
                {
                    _pathIndex = 0;
                    double x = _getRobotX();
                    double y = _getRobotY();

                    for (int i = 0; i < _claimedPathPrefix.Count; i++)
                    {
                        GridPos p = _claimedPathPrefix[i];
                        double tx = GridToCenterWorldX(p.X);
                        double ty = GridToCenterWorldY(p.Y);

                        if (Math.Abs(x - tx) <= ArriveEpsilonM && Math.Abs(y - ty) <= ArriveEpsilonM)
                        {
                            _pathIndex = i + 1;
                        }
                        else
                        {
                            break;
                        }
                    }
                }
            }
        }

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
                    return null;
                }

                double x = _getRobotX();
                double y = _getRobotY();

                AdvanceWaypointIfArrived_Claimed_NoLock(x, y);

                if (_pathIndex >= _claimedPathPrefix.Count)
                {
                    return null;
                }

                GridPos first = _claimedPathPrefix[_pathIndex];
                double firstX = GridToCenterWorldX(first.X);
                double firstY = GridToCenterWorldY(first.Y);

                EnumMoveDirection desiredDir = ChooseDirectionToTarget(x, y, firstX, firstY);

                double? centerMove = TryBuildCenteringMoveDistance(x, y, firstX, firstY, desiredDir);
                if (centerMove.HasValue)
                {
                    EnumMoveDirection centerDir = GetCenteringDirection(x, y, firstX, firstY, desiredDir);
                    if (centerDir != _robotManager.Direction && !_robotManager.IsTurning)
                    {
                        double? turnAngleToCenter = TryGetTurnAngleRad(_robotManager.Direction, centerDir);
                        if (turnAngleToCenter.HasValue)
                        {
                            return RobotCommand.TurnAngle(turnAngleToCenter.Value);
                        }
                    }

                    return RobotCommand.MoveDistance(centerMove.Value);
                }

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

        private double? TryBuildCenteringMoveDistance(double x, double y, double firstX, double firstY, EnumMoveDirection desiredDir)
        {
            double delta;
            if (desiredDir == EnumMoveDirection.Right || desiredDir == EnumMoveDirection.Left)
            {
                delta = firstY - y;
                if (Math.Abs(delta) <= CenterLineEpsilonM)
                {
                    return null;
                }
                return Math.Abs(delta);
            }

            delta = firstX - x;
            if (Math.Abs(delta) <= CenterLineEpsilonM)
            {
                return null;
            }
            return Math.Abs(delta);
        }

        private static EnumMoveDirection GetCenteringDirection(double x, double y, double firstX, double firstY, EnumMoveDirection desiredDir)
        {
            if (desiredDir == EnumMoveDirection.Right || desiredDir == EnumMoveDirection.Left)
            {
                return (firstY - y) >= 0 ? EnumMoveDirection.Down : EnumMoveDirection.Up;
            }

            return (firstX - x) >= 0 ? EnumMoveDirection.Right : EnumMoveDirection.Left;
        }

        /// <summary>
        /// 在已持锁的情况下重建路径：
        /// - 采用与引擎一致的 gridW/gridH，保证障碍物/动态障碍/claimBoard 坐标系一致
        /// - 将机器人世界坐标映射成起点格子
        /// - 结合 baseWalkable + “其它机器人终点不可走”规则生成最终 isWalkable
        /// - 调用具体寻路算法得到路径
        /// </summary>
        private void RebuildPath_NoLock()
        {
            int gridW = _gridW;
            int gridH = _gridH;

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
                {
                    return false;
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

            _claimedPathPrefix.Clear();

            _pathIndex = 0;
            if (_path.Count > 0 && _path[0].Equals(start))
            {
                double cx = GridToCenterWorldX(start.X);
                double cy = GridToCenterWorldY(start.Y);
                double rx = _getRobotX();
                double ry = _getRobotY();

                // 仅当机器人确实在起点格心时才跳过，否则保持 _pathIndex=0 使其先移向格心
                if (Math.Abs(rx - cx) <= ArriveEpsilonM && Math.Abs(ry - cy) <= ArriveEpsilonM)
                {
                    _pathIndex = Math.Min(1, _path.Count);
                }
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
    }
}