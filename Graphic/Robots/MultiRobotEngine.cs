using Graphic.Maps;
using GridDemo.RobotModels.Pathfinding;
using GridDemo.RobotRuns;
using System;
using System.Collections.Generic;

namespace GridDemo.Robots
{
    internal sealed class MultiRobotEngine
    {
        private readonly object _robotLock = new object();

        private readonly int _gridCount;
        private readonly double _cellSizeM;
        private readonly double _dt;
        private readonly double _worldWidthM;
        private readonly double _worldHeightM;

        private readonly ObstacleMap _obstacleMap;
        private readonly List<RobotAgent> _robots = new List<RobotAgent>();
        private readonly Random _random = new Random();

        private readonly double _robotRadiusM;

        private const int RandomPathLength = 12;
        private const double ArriveCenterEpsilonM = 0.06;

        public MultiRobotEngine(int gridCount, double cellSizeM, double dt, double initialMaxSpeed)
        {
            _gridCount = gridCount;
            _cellSizeM = cellSizeM;
            _dt = dt;

            _worldWidthM = gridCount * cellSizeM;
            _worldHeightM = gridCount * cellSizeM;

            _obstacleMap = new ObstacleMap(gridCount, gridCount);

            _robotRadiusM = _cellSizeM / 3.0;
            InitialMaxSpeed = initialMaxSpeed;
        }

        public double InitialMaxSpeed { get; }
        public double CollisionStopSeconds { get; set; } = 2.0;

        public double WorldWidthM => _worldWidthM;
        public double WorldHeightM => _worldHeightM;
        public double CellSizeM => _cellSizeM;
        public int GridCount => _gridCount;

        public void SetRobotCount(int count)
        {
            if (count < 0) throw new ArgumentOutOfRangeException(nameof(count));

            lock (_robotLock)
            {
                while (_robots.Count < count)
                {
                    int id = _robots.Count + 1; // 序号从 1 开始显示更直观
                    var r = new RobotAgent(id);
                    SpawnRobot_NoLock(r);
                    _robots.Add(r);
                }

                while (_robots.Count > count)
                {
                    _robots.RemoveAt(_robots.Count - 1);
                }
            }
        }

        public bool[,] GetObstacleSnapshot()
        {
            return _obstacleMap.GetSnapshot();
        }

        public IReadOnlyList<RobotStateSnapshot> GetRobotSnapshots()
        {
            lock (_robotLock)
            {
                var list = new List<RobotStateSnapshot>(_robots.Count);

                for (int i = 0; i < _robots.Count; i++)
                {
                    var r = _robots[i];

                    var future = BuildFuture3_NoLock(r);
                    var history = BuildHistory2_NoLock(r);

                    list.Add(new RobotStateSnapshot(
                        Id: r.Id,
                        X: r.X,
                        Y: r.Y,
                        Speed: r.Speed,
                        Acc: r.ForwardAcc,
                        OrientationAngle: r.Manager.OrientationAngle,
                        future3: future,
                        history2: history));
                }

                return list;
            }
        }

        public RobotStateSnapshot? TryGetRobotSnapshotById(int robotId)
        {
            lock (_robotLock)
            {
                for (int i = 0; i < _robots.Count; i++)
                {
                    if (_robots[i].Id != robotId)
                    {
                        continue;
                    }

                    var r = _robots[i];
                    return new RobotStateSnapshot(
                        Id: r.Id,
                        X: r.X,
                        Y: r.Y,
                        Speed: r.Speed,
                        Acc: r.ForwardAcc,
                        OrientationAngle: r.Manager.OrientationAngle,
                        future3: BuildFuture3_NoLock(r),
                        history2: BuildHistory2_NoLock(r));
                }

                return null;
            }
        }

        public int TryPickRobot(double worldX, double worldY, double pickRadiusM)
        {
            lock (_robotLock)
            {
                double bestD2 = double.MaxValue;
                int bestId = -1;

                double r2 = pickRadiusM * pickRadiusM;

                for (int i = 0; i < _robots.Count; i++)
                {
                    double dx = worldX - _robots[i].X;
                    double dy = worldY - _robots[i].Y;
                    double d2 = dx * dx + dy * dy;

                    if (d2 > r2)
                    {
                        continue;
                    }

                    if (d2 < bestD2)
                    {
                        bestD2 = d2;
                        bestId = _robots[i].Id;
                    }
                }

                return bestId;
            }
        }

        public void Tick()
        {
            lock (_robotLock)
            {
                // 0) 更新“格子进入事件”（用于历史记录 + 路径推进）
                for (int i = 0; i < _robots.Count; i++)
                {
                    UpdateRobotCellState_NoLock(_robots[i]);
                }

                // 碰撞停计时
                for (int i = 0; i < _robots.Count; i++)
                {
                    var r = _robots[i];
                    if (r.StopRemainSeconds > 0.0)
                    {
                        r.StopRemainSeconds -= _dt;
                        if (r.StopRemainSeconds < 0.0)
                        {
                            r.StopRemainSeconds = 0.0;
                        }
                    }
                }

                // ① 调度
                for (int i = 0; i < _robots.Count; i++)
                {
                    _robots[i].Manager.Tick(_dt, () => _robots[i].ForwardAcc);
                }

                // ② 积分
                for (int i = 0; i < _robots.Count; i++)
                {
                    _robots[i].Move.Update();
                }

                // ③ 碰撞判定（机器人-机器人）
                ResolveRobotCollisions_NoLock();
            }
        }

        private void SpawnRobot_NoLock(RobotAgent agent)
        {
            agent.ForwardAcc = 1.0;
            agent.Speed = 0.0;
            agent.StopRemainSeconds = 0.0;

            for (int tryCount = 0; tryCount < 500; tryCount++)
            {
                int gx = _random.Next(0, _gridCount);
                int gy = _random.Next(0, _gridCount);

                if (_obstacleMap.IsObstacle(new GridPos(gx, gy)))
                {
                    continue;
                }

                double x = GridToCenterWorldX(gx);
                double y = GridToCenterWorldY(gy);

                if (IsHitAnyRobot_NoLock(agent.Id, x, y))
                {
                    continue;
                }

                agent.X = x;
                agent.Y = y;

                agent.CurrentCell = new GridPos(gx, gy);
                agent.RecentCells.Clear();

                agent.Manager = new RobotManager(
                    acc: 0.0,
                    maxSpeed: InitialMaxSpeed,
                    direction: (EnumMoveDirection)_random.Next(0, 4));

                agent.Move = new RobotMove(
                    robotLock: _robotLock,
                    getRobotX: () => agent.X,
                    setRobotX: v => agent.X = v,
                    getRobotY: () => agent.Y,
                    setRobotY: v => agent.Y = v,
                    getRobotSpeed: () => agent.Speed,
                    setRobotSpeed: v => agent.Speed = v,
                    robotManager: agent.Manager,
                    getWorldWidthM: () => _worldWidthM,
                    getWorldHeightM: () => _worldHeightM,
                    cellSizeM: _cellSizeM,
                    dt: _dt,
                    isWorldWalkable: (wx, wy) =>
                    {
                        int cgx = (int)Math.Floor(wx / _cellSizeM);
                        int cgy = (int)Math.Floor(wy / _cellSizeM);

                        if (cgx < 0 || cgy < 0 || cgx >= _gridCount || cgy >= _gridCount)
                        {
                            return false;
                        }

                        return !_obstacleMap.IsObstacle(new GridPos(cgx, cgy));
                    },
                    isHitOtherRobot: (wx, wy) => IsHitAnyRobot_NoLock(agent.Id, wx, wy));

                agent.Manager.BindRuntime(
                    robotLock: _robotLock,
                    move: agent.Move,
                    turn: agent.Move.TurnController,
                    autoCommandProvider: () => BuildPathCommand_NoLock(agent),
                    getForwardAcc: () => agent.ForwardAcc);

                agent.Manager.SetMode(EnumRobotControlMode.Auto);
                agent.Manager.AlignOrientationToDirectionWithTurn();

                // 初始规划一条路径（避免开局空转）
                EnsureRandomPath_NoLock(agent);
                return;
            }

            throw new InvalidOperationException("SpawnRobot failed: cannot find placement.");
        }

        /// <summary>
        /// 自动指令提供器：从机器人“规划路径队列”中取下一步。
        /// 设计原则：避免在一个格子中心连续随机多次 Turn。
        /// </summary>
        private RobotCommand BuildPathCommand_NoLock(RobotAgent agent)
        {
            // 碰撞停期间：不产生命令，并确保停车
            if (agent.StopRemainSeconds > 0.0)
            {
                agent.Move.StopImmediately_NoLock();
                return null;
            }

            // 转向中：等待转向完成（不要覆盖目标角）
            if (agent.Manager.IsTurning)
            {
                return null;
            }

            EnsureRandomPath_NoLock(agent);

            // 若路径队列第一格就是当前格，则丢弃
            while (agent.PlannedPath.Count > 0 && agent.PlannedPath.Peek().Equals(agent.CurrentCell))
            {
                agent.PlannedPath.Dequeue();
            }

            if (agent.PlannedPath.Count == 0)
            {
                return null;
            }

            GridPos nextCell = agent.PlannedPath.Peek();

            // 只允许在“接近格子中心”时决定下一步，减少半格抖动
            if (!IsNearCellCenter_NoLock(agent.X, agent.Y))
            {
                return null;
            }

            EnumMoveDirection desired = GetStepDirection(agent.CurrentCell, nextCell);

            if (agent.Manager.Direction != desired)
            {
                double? delta = TryGetTurnAngleRad(agent.Manager.Direction, desired);
                if (delta.HasValue)
                {
                    return RobotCommand.TurnAngle(delta.Value);
                }
            }

            // 方向一致：走一格
            return RobotCommand.MoveDistance(_cellSizeM);
        }

        private void EnsureRandomPath_NoLock(RobotAgent agent)
        {
            if (agent.PlannedPath.Count >= 3)
            {
                return;
            }

            // 从当前格子开始继续规划补齐
            GridPos cur = agent.CurrentCell;

            // 若队列有元素，用队尾作为起点继续延伸，避免突然改道
            if (agent.PlannedPath.Count > 0)
            {
                GridPos tail = default;
                foreach (GridPos p in agent.PlannedPath)
                {
                    tail = p;
                }
                cur = tail;
            }

            int need = RandomPathLength - agent.PlannedPath.Count;
            if (need <= 0)
            {
                return;
            }

            for (int i = 0; i < need; i++)
            {
                GridPos next = PickRandomNextCell_NoLock(agent, cur);
                agent.PlannedPath.Enqueue(next);
                cur = next;
            }
        }

        private GridPos PickRandomNextCell_NoLock(RobotAgent agent, GridPos from)
        {
            EnumMoveDirection[] dirs = new[]
            {
                EnumMoveDirection.Right,
                EnumMoveDirection.Down,
                EnumMoveDirection.Left,
                EnumMoveDirection.Up
            };

            Shuffle_NoLock(dirs);

            for (int i = 0; i < dirs.Length; i++)
            {
                GridPos candidate = StepCell(from, dirs[i]);

                if (!IsCellInWorld_NoLock(candidate))
                {
                    continue;
                }

                if (_obstacleMap.IsObstacle(candidate))
                {
                    continue;
                }

                // 允许“规划路径”时忽略其它机器人动态占位，否则很容易规划不到
                return candidate;
            }

            return from;
        }

        private void UpdateRobotCellState_NoLock(RobotAgent agent)
        {
            GridPos cell = WorldToGrid(agent.X, agent.Y);

            if (cell.Equals(agent.CurrentCell))
            {
                return;
            }

            // 进入了新格子：写入历史（最多保存 2 个）
            agent.RecentCells.Enqueue(agent.CurrentCell);
            while (agent.RecentCells.Count > 2)
            {
                agent.RecentCells.Dequeue();
            }

            agent.CurrentCell = cell;

            // 若已经到达路径头部的格子，则弹出
            if (agent.PlannedPath.Count > 0 && agent.PlannedPath.Peek().Equals(cell))
            {
                agent.PlannedPath.Dequeue();
            }
        }

        private List<(double X, double Y)> BuildFuture3_NoLock(RobotAgent agent)
        {
            var list = new List<(double X, double Y)>(3);

            int added = 0;
            foreach (GridPos p in agent.PlannedPath)
            {
                list.Add((GridToCenterWorldX(p.X), GridToCenterWorldY(p.Y)));
                added++;
                if (added >= 3)
                {
                    break;
                }
            }

            return list;
        }

        private List<(double X, double Y)> BuildHistory2_NoLock(RobotAgent agent)
        {
            var list = new List<(double X, double Y)>(2);

            foreach (GridPos p in agent.RecentCells)
            {
                list.Add((GridToCenterWorldX(p.X), GridToCenterWorldY(p.Y)));
            }

            return list;
        }

        private void ResolveRobotCollisions_NoLock()
        {
            double hitDist = 2.0 * _robotRadiusM;
            double hitDist2 = hitDist * hitDist;

            for (int i = 0; i < _robots.Count; i++)
            {
                for (int j = i + 1; j < _robots.Count; j++)
                {
                    var a = _robots[i];
                    var b = _robots[j];

                    double dx = a.X - b.X;
                    double dy = a.Y - b.Y;

                    if (dx * dx + dy * dy <= hitDist2)
                    {
                        a.Move.StopImmediately_NoLock();
                        b.Move.StopImmediately_NoLock();

                        a.StopRemainSeconds = CollisionStopSeconds;
                        b.StopRemainSeconds = CollisionStopSeconds;
                    }
                }
            }
        }

        private bool IsHitAnyRobot_NoLock(int selfId, double x, double y)
        {
            double hitDist = 2.0 * _robotRadiusM;
            double hitDist2 = hitDist * hitDist;

            for (int i = 0; i < _robots.Count; i++)
            {
                if (_robots[i].Id == selfId)
                {
                    continue;
                }

                double dx = x - _robots[i].X;
                double dy = y - _robots[i].Y;

                if (dx * dx + dy * dy <= hitDist2)
                {
                    return true;
                }
            }

            return false;
        }

        private bool IsNearCellCenter_NoLock(double x, double y)
        {
            GridPos cell = WorldToGrid(x, y);
            double cx = GridToCenterWorldX(cell.X);
            double cy = GridToCenterWorldY(cell.Y);

            return Math.Abs(x - cx) <= ArriveCenterEpsilonM && Math.Abs(y - cy) <= ArriveCenterEpsilonM;
        }

        private GridPos WorldToGrid(double wx, double wy)
        {
            int gx = (int)Math.Floor(wx / _cellSizeM);
            int gy = (int)Math.Floor(wy / _cellSizeM);

            if (gx < 0) gx = 0;
            if (gy < 0) gy = 0;
            if (gx >= _gridCount) gx = _gridCount - 1;
            if (gy >= _gridCount) gy = _gridCount - 1;

            return new GridPos(gx, gy);
        }

        private bool IsCellInWorld_NoLock(GridPos p)
        {
            return p.X >= 0 && p.Y >= 0 && p.X < _gridCount && p.Y < _gridCount;
        }

        private GridPos StepCell(GridPos from, EnumMoveDirection dir)
        {
            switch (dir)
            {
                case EnumMoveDirection.Right: return new GridPos(from.X + 1, from.Y);
                case EnumMoveDirection.Left: return new GridPos(from.X - 1, from.Y);
                case EnumMoveDirection.Down: return new GridPos(from.X, from.Y + 1);
                case EnumMoveDirection.Up: return new GridPos(from.X, from.Y - 1);
                default: return from;
            }
        }

        private static EnumMoveDirection GetStepDirection(GridPos from, GridPos to)
        {
            int dx = to.X - from.X;
            int dy = to.Y - from.Y;

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

        private double GridToCenterWorldX(int gx) => gx * _cellSizeM + _cellSizeM / 2.0;
        private double GridToCenterWorldY(int gy) => gy * _cellSizeM + _cellSizeM / 2.0;

        private void Shuffle_NoLock<T>(T[] a)
        {
            for (int i = a.Length - 1; i > 0; i--)
            {
                int j = _random.Next(0, i + 1);
                T tmp = a[i];
                a[i] = a[j];
                a[j] = tmp;
            }
        }

        private sealed class RobotAgent
        {
            public RobotAgent(int id)
            {
                Id = id;
            }

            public int Id { get; }
            public double X { get; set; }
            public double Y { get; set; }
            public double Speed { get; set; }
            public double ForwardAcc { get; set; }

            public double StopRemainSeconds { get; set; }

            public GridPos CurrentCell { get; set; }

            /// <summary>
            /// 未来要走的格子（队列头=下一格）。
            /// </summary>
            public Queue<GridPos> PlannedPath { get; } = new Queue<GridPos>();

            /// <summary>
            /// 已走过的后两格（队列头=更老）。
            /// </summary>
            public Queue<GridPos> RecentCells { get; } = new Queue<GridPos>();

            public RobotManager Manager { get; set; }
            public RobotMove Move { get; set; }
        }
    }
}