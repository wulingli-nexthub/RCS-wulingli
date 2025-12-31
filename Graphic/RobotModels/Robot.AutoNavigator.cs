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
        private readonly RobotManager _robotManager;

        private const double ArriveEpsilonM = 0.05;

        private readonly List<GridPos> _path = new List<GridPos>();
        private int _pathIndex;

        private EnumPathfindingAlgorithm _algorithm = EnumPathfindingAlgorithm.AStar;
        private GridPos? _goal;
        private Func<GridPos, bool> _isWalkableProvider = p => true;
        public RobotAutoNavigator(
            object robotLock,
            Func<double> getRobotX,
            Func<double> getRobotY,
            Action<double> setRobotSpeed, // 兼容旧构造参数：目前不再使用（可后续删除）
            Func<double> getWorldWidthM,
            Func<double> getWorldHeightM,
            double cellSizeM,
            RobotManager robotManager)
        {
            _robotLock = robotLock ?? throw new ArgumentNullException(nameof(robotLock));
            _getRobotX = getRobotX ?? throw new ArgumentNullException(nameof(getRobotX));
            _getRobotY = getRobotY ?? throw new ArgumentNullException(nameof(getRobotY));
            _getWorldWidthM = getWorldWidthM ?? throw new ArgumentNullException(nameof(getWorldWidthM));
            _getWorldHeightM = getWorldHeightM ?? throw new ArgumentNullException(nameof(getWorldHeightM));
            _cellSizeM = cellSizeM;
            _robotManager = robotManager ?? throw new ArgumentNullException(nameof(robotManager));
        }
        public void SetIsWalkableProvider(Func<GridPos, bool> isWalkableProvider)
        {
            lock (_robotLock)
            {
                _isWalkableProvider = isWalkableProvider ?? throw new ArgumentNullException(nameof(isWalkableProvider));
            }
        }
        public bool IsEnabled { get; private set; }

        /// <summary>
        /// 启用自动导航：
        /// - 清空旧路径与索引；
        /// - 立即尝试规划一次路径；
        /// - 同步朝向角度（仅内部状态）以避免“逻辑方向/渲染方向不同步”。
        /// </summary>
        public void Enable()
        {
            lock (_robotLock)
            {
                IsEnabled = true;

                _path.Clear();
                _pathIndex = 0;

                RebuildPath_NoLock();

                // 仅做内部状态同步（不驱动运动）
                double angle = RobotManager.DirectionToAngle(_robotManager.Direction);
                _robotManager.OrientationAngle = angle;
                _robotManager.TargetOrientationAngle = angle;
            }
        }

        /// <summary>
        /// 禁用自动导航：清空当前路径与索引，不再输出任何指令。
        /// </summary>
        public void Disable()
        {
            lock (_robotLock)
            {
                IsEnabled = false;

                _path.Clear();
                _pathIndex = 0;
            }
        }

        /// <summary>
        /// 寻路算法切换：
        /// - 切换时清空路径与索引，避免继续沿用旧算法生成的路径；
        /// - 是否立刻重建由调用方决定（本类在 set 中不自动 rebuild，避免频繁/重复规划）。
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
                    _path.Clear();
                    _pathIndex = 0;

                    // 算法切换时，强制刷新自动指令
                    _robotManager.ResetAutoCommands();
                }
            }
        }

        /// <summary>
        /// 设置导航目的地（网格坐标）：
        /// - 会清空当前路径与索引；
        /// - 若处于启用状态且 rebuildIfEnabled=true，则立即重建路径，保证目标切换后不会沿用旧路径。
        /// </summary>
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
                    // 目的地切换 + 已重建路径时，刷新自动指令
                    _robotManager.ResetAutoCommands();
                }
            }
        }

        /// <summary>
        /// 外部主动要求重建路径
        /// </summary>
        public void RebuildPath()
        {
            lock (_robotLock)
            {
                RebuildPath_NoLock();
                if (IsEnabled)
                { // 重新规划路径后，让自动模式立刻用新路径
                    _robotManager.ResetAutoCommands();
                }
            }
        }

        /// <summary>
        /// 生成下一条要执行的指令（若暂时没有则返回 null）。
        ///
        /// 规则（输出层面）：
        /// - 未启用/未设目标：返回 null；
        /// - 无路径：尝试重建；仍失败则返回 null；
        /// - 已到达（或越过）当前路径点：推进索引后再判断；
        /// - 若期望方向与当前方向不同：输出 TurnTo(...)；
        /// - 若方向已对齐：输出 MoveDistance(...)。
        ///
        /// MoveDistance 的合并策略：
        /// - 为减少“走一格发一条指令”的指令抖动，将同方向的连续格子合并为一次位移；
        /// - 合并时以格子中心间距（_cellSizeM）累加。
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
                { // 无路径，尝试重建
                    RebuildPath_NoLock();
                }

                if (_path.Count == 0)
                { // 仍无路径
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

                // 当前目标点，判断当前方向与位置
                GridPos first = _path[_pathIndex];
                double firstX = GridToCenterWorldX(first.X);
                double firstY = GridToCenterWorldY(first.Y);
                EnumMoveDirection desiredDir = ChooseDirectionToTarget(x, y, firstX, firstY);

                // 需要转向：先转向（只输出 Turn 指令）
                if (!_robotManager.IsTurning && desiredDir != _robotManager.Direction)
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

                    EnumMoveDirection stepDir;    // cur->next 的移动方向

                    if (Math.Abs(dx) >= Math.Abs(dy))
                    { // 水平优先，方向是左右，大于等于则向右运动
                        stepDir = dx >= 0 ? EnumMoveDirection.Right : EnumMoveDirection.Left;
                    }
                    else
                    { // 垂直方向，方向是上下，大于等于则向下运动
                        stepDir = dy >= 0 ? EnumMoveDirection.Down : EnumMoveDirection.Up;
                    }

                    if (stepDir != desiredDir)
                    { // 方向不一致，停止合并（说明下一步应该是转向）
                        break;
                    }

                    total += _cellSizeM;           // 每个格中心距离就是 cellSizeM，前进指令的格式是前进多少米（世界位移）
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
        /// 重建路径（外部调用）：
        /// - 将世界尺寸换算为网格宽高；
        /// - 将机器人当前位置映射为起点格子；
        /// - 用寻路算法生成从 start 到 goal 的路径；
        /// - 结果存入 _path，并初始化 _pathIndex。
        ///
        /// 注意：
        /// - gridW/gridH 用 Round 以匹配“世界尺寸 = GridCount * cellSizeM”的场景；
        /// - 若 path 第一格等于 start，则 _pathIndex 从 1 开始，避免把“当前位置”当成下一目标点导致零位移/指令抖动。
        /// </summary>
        private void RebuildPath_NoLock()
        {
            double worldWidth = _getWorldWidthM();
            double worldHeight = _getWorldHeightM();

            int gridW = Math.Max(1, (int)Math.Round(worldWidth / _cellSizeM));         // 将世界尺寸换算为网格宽高
            int gridH = Math.Max(1, (int)Math.Round(worldHeight / _cellSizeM));

            GridPos start = WorldToGrid(_getRobotX(), _getRobotY(), gridW, gridH);

            if (!_goal.HasValue)
            {
                _path.Clear();
                _pathIndex = 0;
                return;
            }

            GridPos goal = _goal.Value;

            Func<GridPos, bool> isWalkable = _isWalkableProvider ?? (p => true);      // 可行走判定（是否有障碍物）

            List<GridPos> path = GridPathfinder.FindPath(          // 调用寻路算法生成路径（List）
                width: gridW,
                height: gridH,
                start: start,
                goal: goal,
                isWalkable: isWalkable,
                algorithm: _algorithm);

            _path.Clear();        // 清空旧路径
            _path.AddRange(path);       // 存入新路径

            _pathIndex = 0;
            if (_path.Count > 0 && _path[0].Equals(start))
            { // 若路径第一格等于起点，则跳过该点
                _pathIndex = Math.Min(1, _path.Count);
            }
        }

        /// <summary>
        /// 推进当前路径索引
        /// </summary>
        private void AdvanceWaypointIfArrived_NoLock(double robotX, double robotY)
        {
            while (_pathIndex < _path.Count)
            {
                GridPos p = _path[_pathIndex];
                double tx = GridToCenterWorldX(p.X);
                double ty = GridToCenterWorldY(p.Y);

                // 1) 距离判定（到达阈值）
                bool arriveX = Math.Abs(robotX - tx) <= ArriveEpsilonM;
                bool arriveY = Math.Abs(robotY - ty) <= ArriveEpsilonM;

                if (arriveX && arriveY)
                { // 到达当前点，推进索引
                    _pathIndex++;
                    continue;   // 回到循环开头，继续判定下一个点是否也已到达，以支持一次推进多个点
                }

                // 2) 跨越中心线判定（只看当前段的轴向），用于“越过索引点但还没进入 arrive 半径”的场景
                if (_pathIndex > 0)
                {
                    GridPos prev = _path[_pathIndex - 1];

                    int dxCell = p.X - prev.X;
                    int dyCell = p.Y - prev.Y;

                    // prev/cur 的中心点：用于稳定判断“是否已经越过 cur”
                    double prevX = GridToCenterWorldX(prev.X);
                    double prevY = GridToCenterWorldY(prev.Y);

                    if (dxCell != 0)
                    { // 水平段：判断 robot 是否已越过 tx（沿 prev->cur 的方向）
                        double dir = Math.Sign(tx - prevX);          // +1=向右，-1=向左
                        if (dir != 0)
                        {
                            // 若 robot 在该方向上的投影距离 >= 目标点投影距离，则认为越过/到达
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
                    { // 垂直段：判断 robot 是否已越过 ty（沿 prev->cur 的方向）
                        double dir = Math.Sign(ty - prevY);          // +1=向下，-1=向上
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
                // 既没到达，也没穿过当前点，则停止推进索引
                break;
            }
        }

        /// <summary>
        /// 获取当前路径的世界坐标点（格子中心）快照：
        /// - 用于 UI 绘制（路径可视化），避免直接暴露内部 _path；
        /// - 返回新 list，调用方可安全枚举。
        /// </summary>
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

        /// <summary>
        /// 世界坐标 -> 网格坐标：
        /// - 使用 Floor 保证落入格子；
        /// - 并对边界进行夹紧，确保索引在 [0, gridW-1] / [0, gridH-1]。
        /// </summary>
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

        /// <summary>
        /// 选择接近目标点的期望方向（只输出四向）：
        /// - 比较 |dx| 与 |dy|，优先走“更远”的轴向，降低步数；
        /// - 这是离散控制策略，不做斜向移动。
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
    }
}