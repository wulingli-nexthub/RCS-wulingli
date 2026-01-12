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
                    int id = _robots.Count;
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
                    list.Add(new RobotStateSnapshot(
                        X: r.X,
                        Y: r.Y,
                        Speed: r.Speed,
                        Acc: r.ForwardAcc,
                        OrientationAngle: r.Manager.OrientationAngle));
                }
                return list;
            }
        }

        public void Tick()
        {
            lock (_robotLock)
            {
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

            // 随机找一个不撞其它机器人/不在障碍的格子中心
            for (int tryCount = 0; tryCount < 500; tryCount++)
            {
                int gx = _random.Next(0, _gridCount);
                int gy = _random.Next(0, _gridCount);

                if (_obstacleMap.IsObstacle(new GridPos(gx, gy)))
                {
                    continue;
                }

                double x = gx * _cellSizeM + _cellSizeM / 2.0;
                double y = gy * _cellSizeM + _cellSizeM / 2.0;

                if (IsHitAnyRobot_NoLock(agent.Id, x, y))
                {
                    continue;
                }

                agent.X = x;
                agent.Y = y;

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
                    autoCommandProvider: () => BuildRandomCommand_NoLock(agent),
                    getForwardAcc: () => agent.ForwardAcc);

                agent.Manager.SetMode(EnumRobotControlMode.Auto);
                agent.Manager.AlignOrientationToDirectionWithTurn();
                return;
            }

            throw new InvalidOperationException("SpawnRobot failed: cannot find placement.");
        }

        private RobotCommand BuildRandomCommand_NoLock(RobotAgent agent)
        {
            // 碰撞停期间：不产生命令，且确保停车
            if (agent.StopRemainSeconds > 0.0)
            {
                agent.Move.StopImmediately_NoLock();
                return null;
            }

            // 若正在转向，让调度层先把转向走完
            if (agent.Manager.IsTurning)
            {
                return null;
            }

            // 选择一个“下一格可走且不撞其它机器人”的方向（随机打乱）
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
                EnumMoveDirection d = dirs[i];
                if (!CanStep_NoLock(agent, d))
                {
                    continue;
                }

                if (agent.Manager.Direction != d)
                {
                    double? delta = TryGetTurnAngleRad(agent.Manager.Direction, d);
                    if (delta.HasValue)
                    {
                        return RobotCommand.TurnAngle(delta.Value);
                    }

                    continue;
                }

                return RobotCommand.MoveDistance(_cellSizeM);
            }

            // 四个方向都走不了：原地随机转一下（避免卡死在死角）
            double[] candidates = new[] { -Math.PI / 2.0, Math.PI / 2.0, Math.PI };
            return RobotCommand.TurnAngle(candidates[_random.Next(0, candidates.Length)]);
        }

        private bool CanStep_NoLock(RobotAgent agent, EnumMoveDirection dir)
        {
            double x = agent.X;
            double y = agent.Y;

            double nx = x;
            double ny = y;

            switch (dir)
            {
                case EnumMoveDirection.Right:
                    nx += _cellSizeM;
                    break;
                case EnumMoveDirection.Left:
                    nx -= _cellSizeM;
                    break;
                case EnumMoveDirection.Down:
                    ny += _cellSizeM;
                    break;
                case EnumMoveDirection.Up:
                    ny -= _cellSizeM;
                    break;
            }

            // 边界+
            if (nx < _cellSizeM / 2.0 || ny < _cellSizeM / 2.0 || nx > _worldWidthM - _cellSizeM / 2.0 || ny > _worldHeightM - _cellSizeM / 2.0)
            {
                return false;
            }

            int gx = (int)Math.Floor(nx / _cellSizeM);
            int gy = (int)Math.Floor(ny / _cellSizeM);

            if (gx < 0 || gy < 0 || gx >= _gridCount || gy >= _gridCount)
            {
                return false;
            }

            if (_obstacleMap.IsObstacle(new GridPos(gx, gy)))
            {
                return false;
            }

            // 目标格中心位置不应撞其它机器人
            if (IsHitAnyRobot_NoLock(agent.Id, nx, ny))
            {
                return false;
            }

            return true;
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

            public RobotManager Manager { get; set; }
            public RobotMove Move { get; set; }
        }
    }
}