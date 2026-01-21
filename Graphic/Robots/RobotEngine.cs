using GridDemo.Maps;
using GridDemo.MultiRobots;
using GridDemo.RobotModels;
using GridDemo.RobotModels.Pathfinding;
using GridDemo.RobotRuns;
using System;
using System.Collections.Generic;

namespace GridDemo.Robots
{
    /// <summary>
    /// 流程状态枚举
    /// </summary>
    internal enum EnumRobotProcessState
    {
        Idle,                  // 待命状态
        AutoNavigating,       // 自动导航中，拉取指令进入指令队列
        ManualControl,        // 手动控制中，响应按键输入
        ObstacleEditing,      // 障碍物编辑中，进入该状态立即停车
        Error                  // 错误状态（停机）
    }

    /// <summary>
    /// 业务引擎（无 UI 依赖）：
    /// - 统一封装机器人运动学（Move/Turn）、自动寻路（AutoNavigator）、手动控制（Manual）、障碍物地图（ObstacleMap）；
    /// - 对 UI 暴露“控制接口 + 快照接口”，UI 不直接接触底层执行器细节；
    /// - 通过一把共享锁 <see cref="_robotLock"/> 保护所有机器人状态的一致性（位置/速度/加速度/转向/指令等）。
    /// 
    /// 说明：
    /// - Tick()：先逻辑层调度指令，再物理层积分更新位置与朝向。
    /// </summary>
    internal sealed class RobotEngine
    {
        private sealed class RobotContext
        {
            public int Id;

            public EnumRobotProcessState State;

            public double X;
            public double Y;
            public double Speed;
            public double Acc;

            public RobotManager Manager;
            public RobotMove Move;
            public RobotAutoNavigator AutoNavigator;
            public RobotManual Manual;

            // 随机目标（网格）
            public bool HasGoal;
            public GridPos Goal;

            // 碰撞让路：临时避让格（把对方格子视为障碍）
            public GridPos? TempAvoidCell;
            public double AvoidTtlSec;
            public double ReplanCooldownSec;

            // 碰撞等待：一方原地停住一小段时间，另一方继续走
            public double WaitTtlSec;
        }

        private readonly object _robotLock = new object();

        private readonly ObstacleMap _obstacleMap;

        private readonly double _cellSizeM;
        private readonly double _dt;
        private readonly int _gridCount;

        private readonly double _worldWidthM;
        private readonly double _worldHeightM;

        private readonly List<RobotContext> _robots = new List<RobotContext>();
        private int _selectedIndex;

        private EnumRobotProcessState _globalState = EnumRobotProcessState.Idle;
        private bool _isObstacleEditMode;

        private readonly Random _rng = new Random();

        // 与 RobotMove 内部一致：cell/3
        private readonly double _robotRadiusM;

        // 策略参数
        private const int DefaultRobotCount = 5;
        private const double DefaultRobotAcc = 0.8;
        private const double ArriveGoalEpsilonM = 0.25;
        private const double AvoidCellTtlSec = 1.0;
        private const double CollisionReplanCooldownSec = 0.35;
        private const double CollisionWaitTtlSec = 0.5;

        public RobotEngine(
            int gridCount,
            double cellSizeM,
            double dt,
            double initialMaxSpeed,
            EnumMoveDirection initialDirection)
        {
            _gridCount = gridCount;
            _cellSizeM = cellSizeM;
            _dt = dt;
            _worldWidthM = gridCount * cellSizeM;
            _worldHeightM = gridCount * cellSizeM;

            _robotRadiusM = _cellSizeM / 3.0;

            _obstacleMap = new ObstacleMap(_gridCount, _gridCount);

            CreateRobots(DefaultRobotCount, initialMaxSpeed, initialDirection);

            _selectedIndex = _robots.Count > 0 ? 0 : -1;

            // 默认全自动 + 每台随机目标
            _globalState = EnumRobotProcessState.AutoNavigating;
            for (int i = 0; i < _robots.Count; i++)
            {
                SetRobotState_NoLock(_robots[i], EnumRobotProcessState.AutoNavigating);
                EnsureRandomGoal_NoLock(_robots[i], force: true);
            }
        }

        #region 公共属性/方法（供 UI 调用）

        public double WorldWidthM => _worldWidthM;
        public double WorldHeightM => _worldHeightM;
        public double CellSizeM => _cellSizeM;
        public int GridCount => _gridCount;
        public int SelectedRobotId
        {
            get
            {
                lock (_robotLock)
                {
                    if (_selectedIndex < 0 || _selectedIndex >= _robots.Count)
                    {
                        return 0;
                    }

                    return _robots[_selectedIndex].Id;
                }
            }
        }

        // 兼容旧 Form1：返回“选中机器人”的 AutoNavigator（DestinationPicker 仍可存在，但不再作为目标来源）
        public RobotAutoNavigator AutoNavigator
        {
            get
            {
                lock (_robotLock)
                {
                    if (_selectedIndex < 0 || _selectedIndex >= _robots.Count)
                    {
                        return null;
                    }

                    return _robots[_selectedIndex].AutoNavigator;
                }
            }
        }

        public EnumPathfindingAlgorithm Algorithm
        {
            get
            {
                lock (_robotLock)
                {
                    if (_selectedIndex < 0 || _selectedIndex >= _robots.Count)
                    {
                        return EnumPathfindingAlgorithm.AStar;
                    }

                    return _robots[_selectedIndex].AutoNavigator.Algorithm;
                }
            }
            set
            {
                lock (_robotLock)
                {
                    for (int i = 0; i < _robots.Count; i++)
                    {
                        _robots[i].AutoNavigator.Algorithm = value;
                    }

                    // 算法切换后统一刷新
                    for (int i = 0; i < _robots.Count; i++)
                    {
                        if (_robots[i].State == EnumRobotProcessState.AutoNavigating)
                        {
                            _robots[i].AutoNavigator.RebuildPath();
                            _robots[i].Manager.ResetAutoCommands();
                        }
                    }
                }
            }
        }

        public bool AutoEnabled
        {
            get
            {
                lock (_robotLock)
                {
                    if (_selectedIndex < 0 || _selectedIndex >= _robots.Count)
                    {
                        return false;
                    }

                    return _robots[_selectedIndex].State == EnumRobotProcessState.AutoNavigating;
                }
            }
        }

        public MultiRobotStateSnapshot GetMultiStateSnapshot()
        {
            lock (_robotLock)
            {
                var list = new List<RobotItemSnapshot>(_robots.Count);

                for (int i = 0; i < _robots.Count; i++)
                {
                    var r = _robots[i];

                    double goalWorldX = 0;
                    double goalWorldY = 0;
                    int goalX = 0;
                    int goalY = 0;

                    if (r.HasGoal)
                    {
                        goalX = r.Goal.X;
                        goalY = r.Goal.Y;
                        goalWorldX = r.Goal.X * _cellSizeM + _cellSizeM / 2.0;
                        goalWorldY = r.Goal.Y * _cellSizeM + _cellSizeM / 2.0;
                    }

                    list.Add(new RobotItemSnapshot(
                        id: r.Id,
                        x: r.X,
                        y: r.Y,
                        speed: r.Speed,
                        acc: r.Acc,
                        orientationAngle: r.Manager.OrientationAngle,
                        direction: r.Manager.Direction,
                        isSelected: i == _selectedIndex,
                        hasGoal: r.HasGoal,
                        goalGridX: goalX,
                        goalGridY: goalY,
                        goalWorldX: goalWorldX,
                        goalWorldY: goalWorldY));
                }

                return new MultiRobotStateSnapshot(list);
            }
        }

        public bool TrySelectRobotByWorld(double worldX, double worldY)
        {
            lock (_robotLock)
            {
                if (_robots.Count == 0)
                {
                    return false;
                }

                double pickR = _robotRadiusM * 1.5;
                double pickR2 = pickR * pickR;

                int bestIndex = -1;
                double bestD2 = double.MaxValue;

                for (int i = 0; i < _robots.Count; i++)
                {
                    double dx = _robots[i].X - worldX;
                    double dy = _robots[i].Y - worldY;
                    double d2 = dx * dx + dy * dy;

                    if (d2 <= pickR2 && d2 < bestD2)
                    {
                        bestD2 = d2;
                        bestIndex = i;
                    }
                }

                if (bestIndex < 0)
                {
                    return false;
                }

                _selectedIndex = bestIndex;
                return true;
            }
        }

        public List<(double X, double Y)> GetPathWorldPointsSnapshot()
        {
            lock (_robotLock)
            {
                if (_selectedIndex < 0 || _selectedIndex >= _robots.Count)
                {
                    return new List<(double X, double Y)>();
                }

                // 仅画选中机器人的路径
                return _robots[_selectedIndex].AutoNavigator.GetPathWorldPointsSnapshot();
            }
        }

        public bool[,] GetObstacleSnapshot()
        {
            return _obstacleMap.GetSnapshot();
        }

        public void ToggleObstacle(GridPos p)
        {
            _obstacleMap.Toggle(p);

            if (_isObstacleEditMode)
            {
                return;
            }

            lock (_robotLock)
            {
                for (int i = 0; i < _robots.Count; i++)
                {
                    if (_robots[i].State == EnumRobotProcessState.AutoNavigating)
                    {
                        _robots[i].AutoNavigator.RebuildPath();
                        _robots[i].Manager.ResetAutoCommands();
                    }
                }
            }
        }

        public void ClearObstacles()
        {
            _obstacleMap.Clear();

            if (_isObstacleEditMode)
            {
                return;
            }

            lock (_robotLock)
            {
                for (int i = 0; i < _robots.Count; i++)
                {
                    if (_robots[i].State == EnumRobotProcessState.AutoNavigating)
                    {
                        _robots[i].AutoNavigator.RebuildPath();
                        _robots[i].Manager.ResetAutoCommands();
                    }
                }
            }
        }

        public void SetObstacleEditMode(bool enabled)
        {
            lock (_robotLock)
            {
                _isObstacleEditMode = enabled;

                if (enabled)
                {
                    _globalState = EnumRobotProcessState.ObstacleEditing;

                    for (int i = 0; i < _robots.Count; i++)
                    {
                        SetRobotState_NoLock(_robots[i], EnumRobotProcessState.ObstacleEditing);
                    }
                }
                else
                {
                    _globalState = EnumRobotProcessState.AutoNavigating;

                    for (int i = 0; i < _robots.Count; i++)
                    {
                        SetRobotState_NoLock(_robots[i], EnumRobotProcessState.AutoNavigating);
                        EnsureRandomGoal_NoLock(_robots[i], force: false);
                        _robots[i].AutoNavigator.RebuildPath();
                        _robots[i].Manager.ResetAutoCommands();
                    }
                }
            }
        }

        public void EnableAuto()
        {
            lock (_robotLock)
            {
                _globalState = EnumRobotProcessState.AutoNavigating;

                for (int i = 0; i < _robots.Count; i++)
                {
                    SetRobotState_NoLock(_robots[i], EnumRobotProcessState.AutoNavigating);
                    EnsureRandomGoal_NoLock(_robots[i], force: false);
                    _robots[i].AutoNavigator.RebuildPath();
                    _robots[i].Manager.ResetAutoCommands();
                    _robots[i].Manager.AlignOrientationToDirectionWithTurn();
                }
            }
        }

        public void EnableManual()
        {
            lock (_robotLock)
            {
                _globalState = EnumRobotProcessState.AutoNavigating;

                for (int i = 0; i < _robots.Count; i++)
                {
                    if (i == _selectedIndex)
                    {
                        SetRobotState_NoLock(_robots[i], EnumRobotProcessState.ManualControl);
                    }
                    else
                    {
                        SetRobotState_NoLock(_robots[i], EnumRobotProcessState.AutoNavigating);
                        EnsureRandomGoal_NoLock(_robots[i], force: false);
                    }
                }
            }
        }

        public void RebuildPath()
        {
            lock (_robotLock)
            {
                for (int i = 0; i < _robots.Count; i++)
                {
                    if (_robots[i].State == EnumRobotProcessState.AutoNavigating)
                    {
                        _robots[i].AutoNavigator.RebuildPath();
                        _robots[i].Manager.ResetAutoCommands();
                    }
                }
            }
        }

        public void SetForwardAcc(double acc)
        {
            lock (_robotLock)
            {
                if (_selectedIndex < 0 || _selectedIndex >= _robots.Count)
                {
                    return;
                }

                _robots[_selectedIndex].Acc = acc;
            }
        }

        public void SetMaxSpeed(double vmax)
        {
            lock (_robotLock)
            {
                for (int i = 0; i < _robots.Count; i++)
                {
                    _robots[i].Manager.MaxSpeed = vmax;
                }
            }
        }

        public void ManualForwardKey(bool down)
        {
            lock (_robotLock)
            {
                if (_selectedIndex < 0 || _selectedIndex >= _robots.Count)
                {
                    return;
                }

                var r = _robots[_selectedIndex];
                r.Manual.InputForwardKey(down);
                r.Manager.ResetManualCommands();
            }
        }

        public void ManualTurnLeftKey(bool down)
        {
            lock (_robotLock)
            {
                if (_selectedIndex < 0 || _selectedIndex >= _robots.Count)
                {
                    return;
                }

                var r = _robots[_selectedIndex];
                r.Manual.InputTurnLeftKey(down);
                r.Manager.ResetManualCommands();
            }
        }

        public void ManualTurnRightKey(bool down)
        {
            lock (_robotLock)
            {
                if (_selectedIndex < 0 || _selectedIndex >= _robots.Count)
                {
                    return;
                }

                var r = _robots[_selectedIndex];
                r.Manual.InputTurnRightKey(down);
                r.Manager.ResetManualCommands();
            }
        }

        public void Tick()
        {
            lock (_robotLock)
            {
                if (_globalState == EnumRobotProcessState.ObstacleEditing
                    || _globalState == EnumRobotProcessState.Idle
                    || _globalState == EnumRobotProcessState.Error)
                {
                    return;
                }

                // 1) 更新临时避让 TTL + 冷却
                for (int i = 0; i < _robots.Count; i++)
                {
                    var r = _robots[i];

                    if (r.ReplanCooldownSec > 0)
                    {
                        r.ReplanCooldownSec -= _dt;
                        if (r.ReplanCooldownSec < 0) r.ReplanCooldownSec = 0;
                    }

                    if (r.TempAvoidCell.HasValue)
                    {
                        r.AvoidTtlSec -= _dt;
                        if (r.AvoidTtlSec <= 0)
                        {
                            r.TempAvoidCell = null;
                            r.AvoidTtlSec = 0;
                        }
                    }

                    if (r.WaitTtlSec > 0)
                    {
                        r.WaitTtlSec -= _dt;
                        if (r.WaitTtlSec < 0) r.WaitTtlSec = 0;
                    }
                }

                // 2) 自动机器人：到达目标 -> 生成新目标并重规划
                for (int i = 0; i < _robots.Count; i++)
                {
                    var r = _robots[i];
                    if (r.State != EnumRobotProcessState.AutoNavigating)
                    {
                        continue;
                    }

                    if (!r.HasGoal)
                    {
                        EnsureRandomGoal_NoLock(r, force: true);
                        r.AutoNavigator.RebuildPath();
                        r.Manager.ResetAutoCommands();
                        continue;
                    }

                    if (IsArrivedGoal_NoLock(r))
                    {
                        EnsureRandomGoal_NoLock(r, force: true);
                        r.AutoNavigator.RebuildPath();
                        r.Manager.ResetAutoCommands();
                    }
                }

                // 3) 推进每台机器人
                for (int i = 0; i < _robots.Count; i++)
                {
                    var r = _robots[i];

                    // 碰撞等待：原地停住，不继续 Tick/Move，避免不断转向/重规划
                    if (r.WaitTtlSec > 0)
                    {
                        r.Move.StopImmediately_NoLock();
                        r.Manager.ResetAutoCommands();
                        continue;
                    }

                    if (r.State == EnumRobotProcessState.AutoNavigating || r.State == EnumRobotProcessState.ManualControl)
                    {
                        r.Manager.Tick(_dt, () => r.Acc);
                        r.Move.Update();
                    }
                }

                // 4) 碰撞 -> 让路并重规划
                ResolveRobotRobotCollisionAndReplan_NoLock();
            }
        }

        public RobotStateSnapshot GetStateSnapshot()
        {
            lock (_robotLock)
            {
                if (_selectedIndex < 0 || _selectedIndex >= _robots.Count)
                {
                    return default(RobotStateSnapshot);
                }

                var r = _robots[_selectedIndex];

                return new RobotStateSnapshot(
                    X: r.X,
                    Y: r.Y,
                    Speed: r.Speed,
                    Acc: r.Acc,
                    OrientationAngle: r.Manager.OrientationAngle);
            }
        }

        #endregion

        #region Private

        private void CreateRobots(int robotCount, double initialMaxSpeed, EnumMoveDirection initialDirection)
        {
            _robots.Clear();

            // 避免出生点重叠：记录已占用格子
            var occupied = new HashSet<GridPos>();

            // 若障碍很多，允许多尝试
            int maxTryPerRobot = 500;

            for (int i = 0; i < robotCount; i++)
            {
                int id = i + 1;

                // 1) 随机挑一个可用起点格
                GridPos start = default(GridPos);
                bool found = false;

                for (int t = 0; t < maxTryPerRobot; t++)
                {
                    int gx = _rng.Next(0, _gridCount);
                    int gy = _rng.Next(0, _gridCount);
                    var p = new GridPos(gx, gy);

                    if (_obstacleMap.IsObstacle(p))
                    {
                        continue;
                    }

                    if (occupied.Contains(p))
                    {
                        continue;
                    }

                    start = p;
                    found = true;
                    break;
                }

                if (!found)
                {
                    // 找不到空位：允许重复，但继续运行（避免直接崩）
                    start = new GridPos(_rng.Next(0, _gridCount), _rng.Next(0, _gridCount));
                }

                occupied.Add(start);

                double x = start.X * _cellSizeM + _cellSizeM / 2.0;
                double y = start.Y * _cellSizeM + _cellSizeM / 2.0;

                var ctx = new RobotContext();
                ctx.Id = id;
                ctx.State = EnumRobotProcessState.Idle;
                ctx.X = x;
                ctx.Y = y;
                ctx.Speed = 0.0;
                ctx.Acc = DefaultRobotAcc;
                ctx.HasGoal = false;
                ctx.TempAvoidCell = null;
                ctx.AvoidTtlSec = 0;
                ctx.ReplanCooldownSec = 0;
                ctx.WaitTtlSec = 0;

                ctx.Manager = new RobotManager(
                    acc: ctx.Acc,
                    maxSpeed: initialMaxSpeed,
                    direction: initialDirection);

                ctx.Move = new RobotMove(
                    robotLock: _robotLock,
                    getRobotX: () => ctx.X,
                    setRobotX: v => ctx.X = v,
                    getRobotY: () => ctx.Y,
                    setRobotY: v => ctx.Y = v,
                    getRobotSpeed: () => ctx.Speed,
                    setRobotSpeed: v => ctx.Speed = v,
                    robotManager: ctx.Manager,
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

                        if (_obstacleMap.IsObstacle(new GridPos(cgx, cgy)))
                        {
                            return false;
                        }

                        if (ctx.TempAvoidCell.HasValue && ctx.TempAvoidCell.Value.Equals(new GridPos(cgx, cgy)))
                        {
                            return false;
                        }

                        return true;
                    });

                ctx.AutoNavigator = new RobotAutoNavigator(
                    robotLock: _robotLock,
                    getRobotX: () => ctx.X,
                    getRobotY: () => ctx.Y,
                    setRobotSpeed: v => ctx.Speed = v,
                    getWorldWidthM: () => _worldWidthM,
                    getWorldHeightM: () => _worldHeightM,
                    cellSizeM: _cellSizeM,
                    robotManager: ctx.Manager);

                ctx.AutoNavigator.SetIsWalkableProvider(p =>
                {
                    if (_obstacleMap.IsObstacle(p))
                    {
                        return false;
                    }

                    if (ctx.TempAvoidCell.HasValue && ctx.TempAvoidCell.Value.Equals(p))
                    {
                        return false;
                    }

                    return true;
                });

                ctx.Manual = new RobotManual(
                    robotLock: _robotLock,
                    robotManager: ctx.Manager);

                ctx.Manager.BindRuntime(
                    robotLock: _robotLock,
                    move: ctx.Move,
                    turn: ctx.Move.TurnController,
                    autoCommandProvider: () => ctx.AutoNavigator.TryBuildNextCommand(),
                    getForwardAcc: () => ctx.Acc);

                ctx.Manager.BindManualCommandProvider(() => ctx.Manual.TryBuildNextCommand());

                _robots.Add(ctx);
            }
        }

        private void SetRobotState_NoLock(RobotContext r, EnumRobotProcessState newState)
        {
            if (r.State == newState)
            {
                return;
            }

            switch (r.State)
            {
                case EnumRobotProcessState.AutoNavigating:
                    r.AutoNavigator.Disable();
                    r.Manager.ResetAutoCommands();
                    r.Move.StopImmediately_NoLock();
                    break;

                case EnumRobotProcessState.ManualControl:
                    r.Manual.Disable();
                    r.Move.StopImmediately_NoLock();
                    break;
            }

            r.State = newState;

            switch (newState)
            {
                case EnumRobotProcessState.AutoNavigating:
                    r.Manager.SetMode(EnumRobotControlMode.Auto);
                    r.AutoNavigator.Enable();

                    EnsureRandomGoal_NoLock(r, force: false);
                    if (r.HasGoal)
                    {
                        r.AutoNavigator.SetGoal(r.Goal, rebuildIfEnabled: true);
                    }

                    r.Manager.ResetAutoCommands();
                    r.Manager.AlignOrientationToDirectionWithTurn();
                    break;

                case EnumRobotProcessState.ManualControl:
                    r.Manager.SetMode(EnumRobotControlMode.Manual);
                    r.Manual.Enable();
                    break;

                case EnumRobotProcessState.ObstacleEditing:
                    r.Speed = 0.0;
                    r.Manager.Acc = 0.0;
                    r.Move.StopImmediately_NoLock();

                    r.AutoNavigator.Disable();
                    r.Manual.Disable();
                    r.Manager.ResetAutoCommands();
                    break;

                case EnumRobotProcessState.Idle:
                    r.Manager.SetMode(EnumRobotControlMode.Auto);
                    r.AutoNavigator.Disable();
                    r.Move.StopImmediately_NoLock();
                    break;

                case EnumRobotProcessState.Error:
                    r.Move.StopImmediately_NoLock();
                    r.AutoNavigator.Disable();
                    r.Manual.Disable();
                    r.Manager.ResetAutoCommands();
                    break;
            }
        }

        private bool IsArrivedGoal_NoLock(RobotContext r)
        {
            if (!r.HasGoal)
            {
                return true;
            }

            double gx = r.Goal.X * _cellSizeM + _cellSizeM / 2.0;
            double gy = r.Goal.Y * _cellSizeM + _cellSizeM / 2.0;

            double dx = r.X - gx;
            double dy = r.Y - gy;

            return dx * dx + dy * dy <= ArriveGoalEpsilonM * ArriveGoalEpsilonM;
        }

        private void EnsureRandomGoal_NoLock(RobotContext r, bool force)
        {
            if (!force && r.HasGoal)
            {
                return;
            }

            const int maxTry = 300;

            GridPos start = WorldToGrid_NoLock(r.X, r.Y);

            // 收集其它机器人当前目标，尽量避开（降低拥挤）
            var otherGoals = new HashSet<GridPos>();
            for (int i = 0; i < _robots.Count; i++)
            {
                var o = _robots[i];
                if (o != r && o.HasGoal)
                {
                    otherGoals.Add(o.Goal);
                }
            }

            for (int k = 0; k < maxTry; k++)
            {
                int gx = _rng.Next(0, _gridCount);
                int gy = _rng.Next(0, _gridCount);
                var goal = new GridPos(gx, gy);

                if (_obstacleMap.IsObstacle(goal))
                {
                    continue;
                }

                if (goal.Equals(start))
                {
                    continue;
                }

                if (r.TempAvoidCell.HasValue && r.TempAvoidCell.Value.Equals(goal))
                {
                    continue;
                }

                // 尽量避免与其他机器人目标重复
                if (otherGoals.Contains(goal))
                {
                    continue;
                }

                r.Goal = goal;
                r.HasGoal = true;

                r.AutoNavigator.SetGoal(goal, rebuildIfEnabled: true);
                return;
            }

            r.HasGoal = false;
        }

        private GridPos WorldToGrid_NoLock(double wx, double wy)
        {
            int gx = (int)Math.Floor(wx / _cellSizeM);
            int gy = (int)Math.Floor(wy / _cellSizeM);

            if (gx < 0) gx = 0;
            if (gy < 0) gy = 0;
            if (gx >= _gridCount) gx = _gridCount - 1;
            if (gy >= _gridCount) gy = _gridCount - 1;

            return new GridPos(gx, gy);
        }

        private void ResolveRobotRobotCollisionAndReplan_NoLock()
        {
            for (int i = 0; i < _robots.Count; i++)
            {
                for (int j = i + 1; j < _robots.Count; j++)
                {
                    var a = _robots[i];
                    var b = _robots[j];

                    GridPos cellA = WorldToGrid_NoLock(a.X, a.Y);
                    GridPos cellB = WorldToGrid_NoLock(b.X, b.Y);

                    int dx = Math.Abs(cellA.X - cellB.X);
                    int dy = Math.Abs(cellA.Y - cellB.Y);

                    // 只判“同格”或“四邻域”（上下左右），不含对角
                    // - 同格：dx==0 && dy==0
                    // - 四邻域：dx+dy==1
                    bool isSameCell = dx == 0 && dy == 0;
                    bool is4Neighbor = (dx + dy) == 1;

                    if (!isSameCell && !is4Neighbor)
                    {
                        continue;
                    }

                    // 等待中的机器人不再重复处理，避免抖动
                    if (a.WaitTtlSec > 0 || b.WaitTtlSec > 0)
                    {
                        continue;
                    }

                    // 自动机器人参与时才处理（手动的不强制改道，但可让自动的等待）
                    bool aAuto = a.State == EnumRobotProcessState.AutoNavigating;
                    bool bAuto = b.State == EnumRobotProcessState.AutoNavigating;

                    if (!aAuto && !bAuto)
                    {
                        continue;
                    }

                    // 选择“让步者”：优先选自动；若两者都自动，选 ID 大的等待（稳定、避免两边来回切）
                    RobotContext yield;
                    RobotContext go;

                    if (aAuto && !bAuto)
                    {
                        yield = a;
                        go = b;
                    }
                    else if (!aAuto && bAuto)
                    {
                        yield = b;
                        go = a;
                    }
                    else
                    {
                        if (a.Id >= b.Id)
                        {
                            yield = a;
                            go = b;
                        }
                        else
                        {
                            yield = b;
                            go = a;
                        }
                    }

                    // 让步者：原地等待 + 清空自动指令 + 停车（关键：不再换目标、不再重规划）
                    yield.WaitTtlSec = CollisionWaitTtlSec;
                    yield.Move.StopImmediately_NoLock();
                    yield.Manager.ResetAutoCommands();

                    // 前进者：可选做轻量避让（把让步者所在格临时视为障碍），但不强制换目标
                    if (go.State == EnumRobotProcessState.AutoNavigating && go.ReplanCooldownSec <= 0)
                    {
                        go.TempAvoidCell = WorldToGrid_NoLock(yield.X, yield.Y);
                        go.AvoidTtlSec = AvoidCellTtlSec;

                        // 只重规划一次，且不生成新目标
                        go.AutoNavigator.RebuildPath();
                        go.Manager.ResetAutoCommands();
                        go.ReplanCooldownSec = CollisionReplanCooldownSec;
                    }
                }
            }
        }

        #endregion
    }

    /// <summary>
    /// 机器人状态快照（值类型）：用于 UI/绘制读取。
    /// </summary>
    internal readonly struct RobotStateSnapshot
    {
        /// <summary>
        /// 构造快照：一次性拷贝当前帧需要展示的状态。
        /// </summary>
        public RobotStateSnapshot(double X, double Y, double Speed, double Acc, double OrientationAngle)
        {
            this.X = X;
            this.Y = Y;
            this.Speed = Speed;
            this.Acc = Acc;
            this.OrientationAngle = OrientationAngle;
        }

        public double X { get; }
        public double Y { get; }
        public double Speed { get; }
        public double Acc { get; }
        public double OrientationAngle { get; }
    }
}