using Graphic.Maps;
using GridDemo.RobotModels.Pathfinding;
using System;
using System.Collections.Generic;

namespace GridDemo.Robots
{
    internal enum EnumRobotProcessState
    {
        Idle,
        AutoNavigating,
        ManualControl,
        ObstacleEditing,
        Error
    }

    internal sealed class RobotEngine
    {
        private EnumRobotProcessState _processState = EnumRobotProcessState.Idle;
        private readonly object _robotLock = new object();

        private readonly ObstacleMap _obstacleMap;

        private readonly double _cellSizeM;
        private readonly double _dt;
        private readonly int _gridCount;

        private readonly double _worldWidthM;
        private readonly double _worldHeightM;

        private readonly Random _rng = new Random();

        private readonly List<RobotInstance> _robots = new List<RobotInstance>();
        private int _selectedRobotId = 0;

        // 单机器人重置后的“随机运动”
        private bool _singleRandomRoamEnabled;

        // --- 碰撞让步冷却：避免每帧 Stop + Rebuild 导致抖动 ---
        private const int YieldCooldownFrames = 12; // 12 帧 * 20ms ≈ 240ms
        private readonly Dictionary<int, int> _yieldCooldownTicks = new Dictionary<int, int>();

        public RobotEngine(int gridCount, double cellSizeM, double dt, double initialMaxSpeed, EnumMoveDirection initialDirection)
        {
            _gridCount = gridCount;
            _cellSizeM = cellSizeM;
            _dt = dt;
            _worldWidthM = gridCount * cellSizeM;
            _worldHeightM = gridCount * cellSizeM;

            _obstacleMap = new ObstacleMap(gridCount, gridCount);

            SetRobotCount(1, initialMaxSpeed, initialDirection);

            _processState = EnumRobotProcessState.AutoNavigating;
            GetSelectedRobot_NoLock().AutoNavigator.Enable();
        }

        public double WorldWidthM => _worldWidthM;
        public double WorldHeightM => _worldHeightM;
        public double CellSizeM => _cellSizeM;
        public int GridCount => _gridCount;

        public int RobotCount
        {
            get
            {
                lock (_robotLock)
                {
                    return _robots.Count;
                }
            }
        }

        public int SelectedRobotId
        {
            get
            {
                lock (_robotLock)
                {
                    return _selectedRobotId;
                }
            }
        }

        public EnumPathfindingAlgorithm Algorithm
        {
            get
            {
                lock (_robotLock)
                {
                    return GetSelectedRobot_NoLock().AutoNavigator.Algorithm;
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
                }
            }
        }

        public bool AutoEnabled
        {
            get
            {
                lock (_robotLock)
                {
                    return GetSelectedRobot_NoLock().AutoNavigator.IsEnabled;
                }
            }
        }

        public bool[,] GetObstacleSnapshot()
        {
            return _obstacleMap.GetSnapshot();
        }

        public void ToggleObstacle(GridPos p)
        {
            _obstacleMap.Toggle(p);

            if (_processState == EnumRobotProcessState.ObstacleEditing)
            {
                return;
            }

            lock (_robotLock)
            {
                for (int i = 0; i < _robots.Count; i++)
                {
                    if (_robots[i].AutoNavigator.IsEnabled)
                    {
                        _robots[i].AutoNavigator.RebuildPath();
                    }
                }
            }
        }

        public void ClearObstacles()
        {
            _obstacleMap.Clear();

            if (_processState == EnumRobotProcessState.ObstacleEditing)
            {
                return;
            }

            lock (_robotLock)
            {
                for (int i = 0; i < _robots.Count; i++)
                {
                    if (_robots[i].AutoNavigator.IsEnabled)
                    {
                        _robots[i].AutoNavigator.RebuildPath();
                    }
                }
            }
        }

        public void SetRobotCount(int count, double initialMaxSpeed, EnumMoveDirection initialDirection)
        {
            if (count < 1)
            {
                count = 1;
            }

            lock (_robotLock)
            {
                _singleRandomRoamEnabled = false;

                // 1) 多 -> 少：只删尾部（保持前面机器人位置不变）
                while (_robots.Count > count)
                {
                    _robots.RemoveAt(_robots.Count - 1);
                }

                // 修正选中项
                if (_selectedRobotId >= _robots.Count)
                {
                    _selectedRobotId = Math.Max(0, _robots.Count - 1);
                }

                // 2) 少 -> 多：只新增，不动已有机器人
                if (_robots.Count < count)
                {
                    var used = BuildUsedCellKeySet_NoLock();

                    while (_robots.Count < count)
                    {
                        int id = _robots.Count;

                        GridPos cell = PickRandomFreeCell_NoLock(used);
                        int key = cell.Y * _gridCount + cell.X;
                        used.Add(key);

                        double x = cell.X * _cellSizeM + _cellSizeM / 2.0;
                        double y = cell.Y * _cellSizeM + _cellSizeM / 2.0;

                        var r = new RobotInstance(
                            id: id,
                            robotLock: _robotLock,
                            obstacleMap: _obstacleMap,
                            gridCount: _gridCount,
                            cellSizeM: _cellSizeM,
                            dt: _dt,
                            worldWidthM: _worldWidthM,
                            worldHeightM: _worldHeightM,
                            initialMaxSpeed: initialMaxSpeed,
                            initialDirection: initialDirection,
                            initialX: x,
                            initialY: y);

                        // 新机器人：启用自动并给一个随机目标，否则不会动
                        r.AutoNavigator.Enable();
                        r.AutoNavigator.ClearGoal();
                        r.Manager.ResetAutoCommands();
                        r.AutoNavigator.SetGoal(PickRandomFreeCell_NoLock(used: null), rebuildIfEnabled: true);

                        // 继承当前“全局加速度”——取第一个机器人的值作为当前配置
                        if (_robots.Count > 0)
                        {
                            r.Acc = _robots[0].Acc;
                        }

                        _robots.Add(r);
                    }
                }

                // 3) 重新绑定动态障碍（包含“占用格”）
                RebindDynamicWalkable_NoLock();

                // 4) 确保全部机器人处于“自动巡航可运行”状态（你要求未选中继续自动）
                for (int i = 0; i < _robots.Count; i++)
                {
                    if (!_robots[i].AutoNavigator.IsEnabled)
                    {
                        _robots[i].AutoNavigator.Enable();
                    }
                }
            }
        }

        private HashSet<int> BuildUsedCellKeySet_NoLock()
        {
            var used = new HashSet<int>(_robots.Count);

            for (int i = 0; i < _robots.Count; i++)
            {
                GridPos c = _robots[i].GetGridPos_NoLock();
                used.Add(c.Y * _gridCount + c.X);
            }

            return used;
        }

        public void ResetToSingleRobotRandomRoam(double initialMaxSpeed, EnumMoveDirection initialDirection)
        {
            lock (_robotLock)
            {
                SetRobotCount(1, initialMaxSpeed, initialDirection);
                _singleRandomRoamEnabled = true;

                // 给一个随机目标，启动随机巡航
                RobotInstance r0 = _robots[0];
                r0.AutoNavigator.ClearGoal();
                r0.Manager.ResetAutoCommands();
                r0.AutoNavigator.SetGoal(PickRandomFreeCell_NoLock(new HashSet<int>()), rebuildIfEnabled: true);
            }
        }

        public bool SelectRobot(int id)
        {
            lock (_robotLock)
            {
                if (id < 0 || id >= _robots.Count)
                {
                    return false;
                }

                _selectedRobotId = id;
                return true;
            }
        }

        public List<RobotStateSnapshot> GetRobotStatesSnapshot()
        {
            lock (_robotLock)
            {
                var list = new List<RobotStateSnapshot>(_robots.Count);
                for (int i = 0; i < _robots.Count; i++)
                {
                    list.Add(_robots[i].GetSnapshot());
                }
                return list;
            }
        }

        public RobotStateSnapshot GetStateSnapshot()
        {
            lock (_robotLock)
            {
                return GetSelectedRobot_NoLock().GetSnapshot();
            }
        }

        public List<(double X, double Y)> GetPathWorldPointsSnapshot()
        {
            lock (_robotLock)
            {
                return GetSelectedRobot_NoLock().AutoNavigator.GetPathWorldPointsSnapshot();
            }
        }

        public void SetForwardAcc(double acc)
        {
            lock (_robotLock)
            {
                // 关键修复：对所有机器人同步（否则只有选中机器人 Acc != 0）
                for (int i = 0; i < _robots.Count; i++)
                {
                    _robots[i].Acc = acc;

                    // 自动模式下需要刷新命令，让下一帧 MoveDistance 读取到新的 forwardAcc
                    _robots[i].Manager.ResetAutoCommands();
                }
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

        public void EnableAuto()
        {
            lock (_robotLock)
            {
                ChangeProcessState_NoLock(EnumRobotProcessState.AutoNavigating);

                RobotInstance r = GetSelectedRobot_NoLock();
                r.AutoNavigator.Enable();
                r.Manager.ResetAutoCommands();
                r.Manager.AlignOrientationToDirectionWithTurn();
            }
        }

        public void EnableManual()
        {
            lock (_robotLock)
            {
                ChangeProcessState_NoLock(EnumRobotProcessState.ManualControl);

                RobotInstance r = GetSelectedRobot_NoLock();
                r.Manual.Enable();
            }
        }

        public void RebuildPath()
        {
            lock (_robotLock)
            {
                GetSelectedRobot_NoLock().AutoNavigator.RebuildPath();
            }
        }

        public void ManualForwardKey(bool down)
        {
            lock (_robotLock)
            {
                RobotInstance r = GetSelectedRobot_NoLock();
                r.Manual.InputForwardKey(down);
                r.Manager.ResetManualCommands();
            }
        }

        public void ManualTurnLeftKey(bool down)
        {
            lock (_robotLock)
            {
                RobotInstance r = GetSelectedRobot_NoLock();
                r.Manual.InputTurnLeftKey(down);
                r.Manager.ResetManualCommands();
            }
        }

        public void ManualTurnRightKey(bool down)
        {
            lock (_robotLock)
            {
                RobotInstance r = GetSelectedRobot_NoLock();
                r.Manual.InputTurnRightKey(down);
                r.Manager.ResetManualCommands();
            }
        }

        public bool TrySetSelectedRobotGoal(GridPos goal)
        {
            lock (_robotLock)
            {
                RobotInstance r = GetSelectedRobot_NoLock();
                r.AutoNavigator.SetGoal(goal, rebuildIfEnabled: true);
                return true;
            }
        }

        public void SetObstacleEditMode(bool enabled)
        {
            lock (_robotLock)
            {
                if (enabled)
                {
                    ChangeProcessState_NoLock(EnumRobotProcessState.ObstacleEditing);

                    for (int i = 0; i < _robots.Count; i++)
                    {
                        _robots[i].Speed = 0.0;
                        _robots[i].Manager.Acc = 0.0;
                        _robots[i].Move.StopImmediately_NoLock();
                        if (_robots[i].AutoNavigator.IsEnabled)
                        {
                            _robots[i].AutoNavigator.Disable();
                        }
                        _robots[i].Manual.Disable();
                        _robots[i].Manager.ResetAutoCommands();
                    }
                }
                else
                {
                    ChangeProcessState_NoLock(EnumRobotProcessState.AutoNavigating);

                    for (int i = 0; i < _robots.Count; i++)
                    {
                        _robots[i].AutoNavigator.Enable();
                        _robots[i].Manager.ResetAutoCommands();
                    }
                }
            }
        }

        public void Tick()
        {
            switch (_processState)
            {
                case EnumRobotProcessState.ObstacleEditing:
                case EnumRobotProcessState.Idle:
                case EnumRobotProcessState.Error:
                    return;
            }

            lock (_robotLock)
            {
                // 0) 冷却计数递减
                if (_yieldCooldownTicks.Count > 0)
                {
                    var keys = new List<int>(_yieldCooldownTicks.Keys);
                    for (int i = 0; i < keys.Count; i++)
                    {
                        int id = keys[i];
                        int t = _yieldCooldownTicks[id] - 1;
                        if (t <= 0)
                        {
                            _yieldCooldownTicks.Remove(id);
                        }
                        else
                        {
                            _yieldCooldownTicks[id] = t;
                        }
                    }
                }

                // 1) 动态障碍：把其它机器人占用的格子注入 WalkableProvider
                RebindDynamicWalkable_NoLock();

                // 2) 碰撞让步：以网格为单位检测“前后左右/同格”
                HandleRobotCollisions_NoLock();

                // 3) 未选中机器人继续 Auto（不响应手动）
                for (int i = 0; i < _robots.Count; i++)
                {
                    RobotInstance r = _robots[i];
                    bool isSelected = r.Id == _selectedRobotId;

                    if (!isSelected)
                    {
                        // 强制未选中机器人保持自动巡航
                        r.Manual.Disable();
                        if (!r.AutoNavigator.IsEnabled)
                        {
                            r.AutoNavigator.Enable();
                        }
                    }

                    if (r.AutoNavigator.IsEnabled)
                    {
                        // 没目标时给一个随机目标，确保“自动巡航”一定会走
                        // （避免 EnableAuto() 里 ClearGoal 后一直不动）
                        if (r.AutoNavigator.GetPathWorldPointsSnapshot() == null
                            || r.AutoNavigator.GetPathWorldPointsSnapshot().Count == 0)
                        {
                            // 注意：这里不使用 used，目标允许和别的机器人当前位置冲突由动态障碍避让+重规划解决
                            r.AutoNavigator.SetGoal(PickRandomFreeCell_NoLock(used: null), rebuildIfEnabled: true);
                            r.Manager.ResetAutoCommands();
                        }
                    }

                    // 调度命令 + 运动学
                    r.Manager.Tick(_dt, () => r.Acc);
                    r.Move.Update();
                }

                // 4) 单机器人随机巡航：无路径/到达后重置随机目标
                if (_singleRandomRoamEnabled && _robots.Count == 1)
                {
                    RobotInstance r0 = _robots[0];
                    var p = r0.AutoNavigator.GetPathWorldPointsSnapshot();
                    if (p == null || p.Count < 2)
                    {
                        r0.AutoNavigator.SetGoal(PickRandomFreeCell_NoLock(new HashSet<int>()), rebuildIfEnabled: true);
                    }
                }
            }
        }

        // -------------------------- 碰撞优化：新实现 -------------------------- //

        private void HandleRobotCollisions_NoLock()
        {
            if (_robots.Count <= 1)
            {
                return;
            }

            var cur = new GridPos[_robots.Count];
            var next = new GridPos[_robots.Count];

            for (int i = 0; i < _robots.Count; i++)
            {
                RobotInstance r = _robots[i];
                cur[i] = r.GetGridPos_NoLock();
                next[i] = GetIntendedNextCell_NoLock(r, cur[i]);
            }

            for (int i = 0; i < _robots.Count; i++)
            {
                for (int j = i + 1; j < _robots.Count; j++)
                {
                    // 三类冲突：
                    // 1) 同目标格：两者都想进同一格
                    bool sameTarget = next[i].Equals(next[j]);

                    // 2) 交换格：对向互换
                    bool swap = next[i].Equals(cur[j]) && next[j].Equals(cur[i]);

                    // 3) 穿入对方格：任一方下一步将进入对方当前格（防穿模核心）
                    bool enterOther =
                        next[i].Equals(cur[j]) ||
                        next[j].Equals(cur[i]);

                    if (!sameTarget && !swap && !enterOther)
                    {
                        continue;
                    }

                    RobotInstance a = _robots[i];
                    RobotInstance b = _robots[j];

                    if (IsInYieldCooldown_NoLock(a.Id) && IsInYieldCooldown_NoLock(b.Id))
                    {
                        continue;
                    }

                    RobotInstance yield;
                    RobotInstance go;

                    if (IsInYieldCooldown_NoLock(a.Id))
                    {
                        yield = b;
                        go = a;
                    }
                    else if (IsInYieldCooldown_NoLock(b.Id))
                    {
                        yield = a;
                        go = b;
                    }
                    else
                    {
                        // 默认策略：Id 大者让步（确定性）
                        if (a.Id > b.Id)
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

                    // 让步方立即刹停并重规划
                    ApplyYield_NoLock(yield);

                    // 通行方也刷新一下队列，避免仍在执行“会穿入”的旧 MoveDistance
                    go.Manager.ResetAutoCommands();
                }
            }
        }

        private bool IsInYieldCooldown_NoLock(int robotId)
        {
            int t;
            return _yieldCooldownTicks.TryGetValue(robotId, out t) && t > 0;
        }

        private void ApplyYield_NoLock(RobotInstance yield)
        {
            // 进入冷却：短时间内不重复 Rebuild，避免抖动
            _yieldCooldownTicks[yield.Id] = YieldCooldownFrames;

            // 让步行为：停车 + 清空命令 + 重规划
            yield.Move.StopImmediately_NoLock();
            yield.Manager.ResetAutoCommands();

            // 强制把渲染角度拉回离散方向，防止“斜角残留”
            NormalizeDirAngle_NoLock(yield);

            if (yield.AutoNavigator.IsEnabled)
            {
                yield.AutoNavigator.RebuildPath();
            }
        }

        private void NormalizeDirAngle_NoLock(RobotInstance r)
        {
            // 把 OrientationAngle 强制对齐到 Direction 对应角度，避免频繁打断转向导致停在中间角
            r.Manager.IsTurning = false;
            r.Manager.TargetOrientationAngle = RobotManager.DirectionToAngle(r.Manager.Direction);
            r.Manager.OrientationAngle = r.Manager.TargetOrientationAngle;
        }

        private GridPos GetIntendedNextCell_NoLock(RobotInstance r, GridPos curCell)
        {
            // 默认意图为“不动”
            GridPos target = curCell;

            // 只对自动巡航做预测（手动连续角移动更复杂，先不在这里做阻拦）
            if (!r.AutoNavigator.IsEnabled)
            {
                return target;
            }

            // 转向中不预测（避免误判）
            if (r.Manager.IsTurning)
            {
                return target;
            }

            // 关键：按“下一帧预测位置”推算下一格，避免速度较大时跨格穿模
            // 预测距离：max(speed * dt, 一个很小的最小值)，保证低速也能预测到相邻格意图
            double v = r.Speed;
            double move = v * _dt;
            if (move < _cellSizeM * 0.15) // 经验值：低速时也至少预测 0.15 格
            {
                move = _cellSizeM * 0.15;
            }

            int dx = 0;
            int dy = 0;

            switch (r.Manager.Direction)
            {
                case EnumMoveDirection.Right:
                    dx = +1;
                    break;
                case EnumMoveDirection.Left:
                    dx = -1;
                    break;
                case EnumMoveDirection.Down:
                    dy = +1;
                    break;
                case EnumMoveDirection.Up:
                    dy = -1;
                    break;
            }

            // 用当前格中心 + 预测位移，映射到将要进入的格子
            double curCenterX = curCell.X * _cellSizeM + _cellSizeM / 2.0;
            double curCenterY = curCell.Y * _cellSizeM + _cellSizeM / 2.0;

            double predX = curCenterX + dx * move;
            double predY = curCenterY + dy * move;

            int gx = (int)Math.Floor(predX / _cellSizeM);
            int gy = (int)Math.Floor(predY / _cellSizeM);

            if (gx < 0) gx = 0;
            if (gy < 0) gy = 0;
            if (gx >= _gridCount) gx = _gridCount - 1;
            if (gy >= _gridCount) gy = _gridCount - 1;

            target = new GridPos(gx, gy);
            return target;
        }

        private void ChangeProcessState_NoLock(EnumRobotProcessState newState)
        {
            _processState = newState;
        }

        private RobotInstance GetSelectedRobot_NoLock()
        {
            if (_selectedRobotId < 0)
            {
                _selectedRobotId = 0;
            }
            if (_selectedRobotId >= _robots.Count)
            {
                _selectedRobotId = _robots.Count - 1;
            }

            return _robots[_selectedRobotId];
        }

        private GridPos PickRandomFreeCell_NoLock(HashSet<int> used)
        {
            for (int tries = 0; tries < 5000; tries++)
            {
                int x = _rng.Next(0, _gridCount);
                int y = _rng.Next(0, _gridCount);
                int key = y * _gridCount + x;

                if (used != null && used.Contains(key))
                {
                    continue;
                }

                var p = new GridPos(x, y);
                if (_obstacleMap.IsObstacle(p))
                {
                    continue;
                }

                return p;
            }

            return new GridPos(0, 0);
        }

        private void RebindDynamicWalkable_NoLock()
        {
            // occupied：所有机器人当前占用格
            var occupied = new HashSet<int>(_robots.Count);
            var cellKeys = new int[_robots.Count];

            for (int i = 0; i < _robots.Count; i++)
            {
                GridPos c = _robots[i].GetGridPos_NoLock();
                int key = c.Y * _gridCount + c.X;
                cellKeys[i] = key;
                occupied.Add(key);
            }

            for (int i = 0; i < _robots.Count; i++)
            {
                RobotInstance me = _robots[i];
                int myKey = cellKeys[i];

                me.AutoNavigator.SetIsWalkableProvider(p =>
                {
                    if (_obstacleMap.IsObstacle(p))
                    {
                        return false;
                    }

                    int key = p.Y * _gridCount + p.X;

                    // 自己所在格允许，否则会把自己当障碍卡死
                    if (key == myKey)
                    {
                        return true;
                    }

                    return !occupied.Contains(key);
                });
            }
        }
    }

    internal readonly struct RobotStateSnapshot
    {
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