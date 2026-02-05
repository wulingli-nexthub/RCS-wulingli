using GridDemo.RobotModels.Pathfinding;
using GridDemo.RobotRuns;
using GridDemo.Robots;
using System;
using System.Collections.Generic;

namespace GridDemo.RobotModels
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
        /// 自动指令生成（基于“已抢占的路径前缀”）：
        /// - 抢不到下一步（没有前缀，或已走完前缀）就返回 null，让上层选择等待/停车。
        /// - 同方向的连续格子会合并成一次 MoveDistance，减少指令数量。
        /// - 若当前方向与期望方向不同，则优先发 TurnAngle。
        /// </summary>
        public RobotCommand TryBuildNextCommandFromClaimedPath()
        {
            lock (_robotLock)
            {
                // 未启用或无目标：不生成指令
                if (!IsEnabled || !_goal.HasValue)
                {
                    return null;
                }

                // 必要时构建完整路径（抢占依赖完整路径，通常由引擎取 snapshot 后去抢占）
                if (_path.Count == 0)
                {
                    RebuildPath_NoLock();
                }

                if (_path.Count == 0)
                {
                    return null;
                }

                if (_claimedPathPrefix.Count == 0)
                { // 没抢到任何路径前缀：必须等待
                    return null;
                }

                double x = _getRobotX();
                double y = _getRobotY();

                // 在“已抢占前缀范围内”推进路径索引（避免推进到未抢占段）
                AdvanceWaypointIfArrived_Claimed_NoLock(x, y);

                // 已走完已抢占前缀：等待下一帧抢占更多
                if (_pathIndex >= _claimedPathPrefix.Count)
                {
                    return null;
                }

                // 以当前 index 指向的格子作为最近目标点（格子中心）
                GridPos first = _claimedPathPrefix[_pathIndex];
                double firstX = GridToCenterWorldX(first.X);
                double firstY = GridToCenterWorldY(first.Y);

                // 决定朝哪个主轴方向走（水平优先或垂直优先取决于 dx/dy 绝对值）
                EnumMoveDirection desiredDir = ChooseDirectionToTarget(x, y, firstX, firstY);

                // 关键修复：先对齐“中心线”，再沿主轴走
                // 否则只沿单轴累加 MoveDistance 会把轻微偏差一直带下去，视觉上变成贴网格线走。
                double? centerMove = TryBuildCenteringMoveDistance(x, y, firstX, firstY, desiredDir);
                if (centerMove.HasValue)
                {
                    // 对中阶段不强制转向：维持当前离散方向即可（MoveDistance 会按 Direction 执行）
                    // 只要把 Direction 临时设置为对中的方向即可确保 MoveDistance 方向正确。
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

                // 若需要转向且当前不在转向中，则先发转向指令
                if (!_robotManager.IsTurning && desiredDir != _robotManager.Direction)
                {
                    double? turnAngle = TryGetTurnAngleRad(_robotManager.Direction, desiredDir);
                    if (turnAngle.HasValue)
                    {
                        return RobotCommand.TurnAngle(turnAngle.Value);
                    }

                    // 理论上只有 current==target 才会 null；这里保守返回 null
                    return null;
                }

                // 计算本次 MoveDistance 总距离：
                // 1) 先补齐从当前位置到“第一个格子中心”在主轴方向上的剩余距离（可能不在中心点上）
                // 2) 再把后续“同方向连续格子”按整格长度合并
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

                    // 一旦下一步方向变化，就停止合并，留到下一次指令处理（可能需要转向）
                    if (GetStepDirection(cur, next) != desiredDir)
                    {
                        break;
                    }

                    total += _cellSizeM;
                    i++;
                }

                // 极小距离无意义：避免发出接近 0 的移动指令造成抖动
                if (total <= 0.000001)
                {
                    return null;
                }

                return RobotCommand.MoveDistance(total);
            }
        }

        /// <summary>
        /// 对齐中心线：让机器人先回到“即将前进的格子中心线”上。
        /// - 若 desiredDir 为水平，则把 y 对齐到 firstY；
        /// - 若 desiredDir 为垂直，则把 x 对齐到 firstX。
        /// </summary>
        private double? TryBuildCenteringMoveDistance(double x, double y, double firstX, double firstY, EnumMoveDirection desiredDir)
        {
            double delta;
            if (desiredDir == EnumMoveDirection.Right || desiredDir == EnumMoveDirection.Left)
            {
                // 水平走：先把 y 拉回中心线
                delta = firstY - y;
                if (Math.Abs(delta) <= CenterLineEpsilonM)
                {
                    return null;
                }
                return Math.Abs(delta);
            }

            // 垂直走：先把 x 拉回中心线
            delta = firstX - x;
            if (Math.Abs(delta) <= CenterLineEpsilonM)
            {
                return null;
            }
            return Math.Abs(delta);
        }

        /// <summary>
        /// 获取对中移动的方向（与 desiredDir 垂直的那条轴）。
        /// </summary>
        private static EnumMoveDirection GetCenteringDirection(double x, double y, double firstX, double firstY, EnumMoveDirection desiredDir)
        {
            if (desiredDir == EnumMoveDirection.Right || desiredDir == EnumMoveDirection.Left)
            {
                // 水平走：对中需要上下移动
                return (firstY - y) >= 0 ? EnumMoveDirection.Down : EnumMoveDirection.Up;
            }

            // 垂直走：对中需要左右移动
            return (firstX - x) >= 0 ? EnumMoveDirection.Right : EnumMoveDirection.Left;
        }

        /// <summary>
        /// 在已持锁的情况下重建路径：
        /// - 将世界尺寸离散为 gridW/gridH
        /// - 将机器人世界坐标映射成起点格子
        /// - 结合 baseWalkable + goalOwnerMap 生成最终 isWalkable
        /// - 调用具体寻路算法得到路径
        /// 
        /// 注意：该方法不直接触发引擎抢占，只产生“完整路径”；抢占由引擎层完成后，通过 SetClaimedPathPrefix 回灌。
        /// </summary>
        private void RebuildPath_NoLock()
        {
            double worldWidth = _getWorldWidthM();
            double worldHeight = _getWorldHeightM();

            // 以 cellSize 为分辨率将世界映射到格子；至少 1x1
            int gridW = Math.Max(1, (int)Math.Round(worldWidth / _cellSizeM));
            int gridH = Math.Max(1, (int)Math.Round(worldHeight / _cellSizeM));

            GridPos start = WorldToGrid(_getRobotX(), _getRobotY(), gridW, gridH);

            // 无目标则清空路径
            if (!_goal.HasValue)
            {
                _path.Clear();
                _claimedPathPrefix.Clear();
                _pathIndex = 0;
                return;
            }

            GridPos goal = _goal.Value;

            // 基础可通行判断（例如障碍物）；若未设置则全部可走
            Func<GridPos, bool> baseWalkable = _isWalkableProvider ?? (p => true);

            // goalOwnerMap：key = y*_gridW + x，value = ownerRobotId
            // 用途：把“被其他机器人拥有的格子”视为不可走（避免抢占/目标冲突）
            Dictionary<int, int> goalOwnerMap = null;
            try
            {
                goalOwnerMap = _getGoalOwnerMap?.Invoke();
            }
            catch
            {
                goalOwnerMap = null;
            }

            // 最终可通行：必须满足 baseWalkable，并且不能踩到其他机器人拥有的格子
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

            // 根据算法选择不同寻路实现
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

            // 初始化索引：如果路径第一个点就是 start，则跳过它，减少“走回起点中心”的无意义动作
            _pathIndex = 0;
            if (_path.Count > 0 && _path[0].Equals(start))
            {
                _pathIndex = Math.Min(1, _path.Count);
            }
        }

        /// <summary>
        /// 从两个相邻格子推断移动方向（水平/垂直）。
        /// 约定：若 dx 与 dy 同时存在（理论上不应出现），以 |dx|>=|dy| 优先水平。
        /// </summary>
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

        /// <summary>
        /// 从当前方向转到目标方向需要转多少弧度（以 90° 为单位）。
        /// - 返回 null 表示无需转向。
        /// - 正值表示“向右转”（顺时针），负值表示“向左转”（逆时针）。
        /// </summary>
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

        /// <summary>
        /// 沿完整路径推进 <see cref="_pathIndex"/>：
        /// - 若机器人已到达当前 waypoint 的中心（在 epsilon 内），则 index++。
        /// - 额外处理：若机器人已经“越过”目标点（按上一段方向投影判断），也可认为到达并推进。
        ///   目的：避免因为离散仿真/误差导致卡在某个点附近不前进。
        /// </summary>
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

                    // 若当前段是水平移动：比较 robotX 在该方向上的投影是否已超过 waypoint 的投影
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
                    // 若当前段是垂直移动：比较 robotY 的投影
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

        /// <summary>
        /// 按“已抢占前缀”推进索引（避免把 <see cref="_pathIndex"/> 推进到未抢占段）。
        /// 这保证了：TryBuildNextCommandFromClaimedPath() 只会针对已抢占（安全）的格子作用。
        /// </summary>
        private void AdvanceWaypointIfArrived_Claimed_NoLock(double robotX, double robotY)
        {
            // limit 同时受 claimed 与完整 path 的约束：避免 claimed 与 path 不一致时越界
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

        /// <summary>
        /// 返回完整路径的世界坐标点列表（格子中心点），通常用于绘制。
        /// 注意：这里绘制的是完整路径，不是 claimed 前缀（前缀是抢占结果）。
        /// </summary>
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

        /// <summary>
        /// 返回完整路径（格子序列）的快照：
        /// - 引擎层格子锁抢占应基于完整路径进行。
        /// </summary>
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

        /// <summary>
        /// 世界坐标（米）-> 格子坐标：
        /// - 用 floor 映射到格子索引
        /// - 并把结果 clamp 到 [0, gridW-1]/[0, gridH-1]，避免越界
        /// </summary>
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

        // 格子坐标 -> 该格子的中心世界坐标（米）
        private double GridToCenterWorldX(int gx) => gx * _cellSizeM + _cellSizeM / 2.0;
        private double GridToCenterWorldY(int gy) => gy * _cellSizeM + _cellSizeM / 2.0;

        /// <summary>
        /// 从当前位置指向目标点，选择主移动方向：
        /// - |dx|>=|dy| 优先水平，否则优先垂直。
        /// 用于决定是先“转向+水平走”还是“转向+垂直走”。
        /// </summary>
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

        /// <summary>
        /// 返回目标格子快照。
        /// </summary>
        public GridPos? GetGoalGridSnapshot()
        {
            lock (_robotLock)
            {
                return _goal;
            }
        }

        /// <summary>
        /// 返回“下一步”快照：当前格子、下一目标格子、预期移动方向。
        /// - 若存在 claimed 前缀，则下一步以 claimed 为准（保证安全）。
        /// - 否则以完整路径为准（旧模式/展示用途）。
        /// </summary>
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

                // 计算当前所在格子（用于 UI/调试展示“当前->下一步”）
                double worldWidth = _getWorldWidthM();
                double worldHeight = _getWorldHeightM();
                int gridW = Math.Max(1, (int)Math.Round(worldWidth / _cellSizeM));
                int gridH = Math.Max(1, (int)Math.Round(worldHeight / _cellSizeM));
                currentCell = WorldToGrid(x, y, gridW, gridH);

                // 下一目标格子：优先 claimed，否则 path
                nextCell = (_claimedPathPrefix.Count > 0) ? _claimedPathPrefix[_pathIndex] : _path[_pathIndex];

                int dx = nextCell.X - currentCell.X;
                int dy = nextCell.Y - currentCell.Y;

                // 若已经在同一格（但可能还没在中心点）：用中心点矢量决定方向
                if (dx == 0 && dy == 0)
                {
                    double tx = GridToCenterWorldX(nextCell.X);
                    double ty = GridToCenterWorldY(nextCell.Y);
                    desiredDir = ChooseDirectionToTarget(x, y, tx, ty);
                    return true;
                }

                // 正常情况：按格子差值决定方向
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

        /// <summary>
        /// 生成“从当前到目标”的路径快照（不改内部状态）：
        /// - 主要用于外部临时查询/对比（例如 UI 展示、调试）。
        /// - 与 RebuildPath_NoLock 的差异：不写入 _path/_pathIndex，也不清 claimed。
        /// </summary>
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