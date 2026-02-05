using Graphic.Maps;
using GridDemo.RobotModels.Pathfinding;
using GridDemo.RobotRuns;
using System;
using System.Collections.Generic;

namespace GridDemo.Robots
{
    /// <summary>
    /// 机器人引擎当前运行/交互模式
    /// </summary>
    internal enum EnumRobotProcessState
    {
        Idle, // 空闲：不更新仿真
        AutoNavigating, // 自动巡航：自动寻路 + 移动
        ManualControl, // 手动控制：键盘等输入控制
        ObstacleEditing, // 障碍编辑：暂停运动，仅编辑地图
        Error // 错误：不更新仿真
    }

    /// <summary>
    /// 多机器人仿真/调度引擎：
    /// - 持有所有机器人实例（<see cref="RobotInstance"/>）与地图数据（障碍、抢占板）。
    /// - 在仿真线程中每帧调用 <see cref="Tick"/>：
    ///   1) 重建动态可通行性（把其它机器人/目标格视为动态障碍）
    ///   2) 路径格子锁抢占（按完整路径 -> 抢占前缀）
    ///   3) 根据抢占前缀生成指令，并驱动运动学更新
    ///   4) 机器人移动后释放已走过格子锁（边走边释放）
    ///
    /// 并发模型：
    /// - UI 与仿真线程共享状态，通过 <see cref="_robotLock"/> 保证一致性。
    /// </summary>
    internal sealed class RobotEngine
    {
        private EnumRobotProcessState _processState = EnumRobotProcessState.Idle; // 当前引擎状态（默认 Idle）

        // 全局互斥锁：保护 _robots、导航器路径、抢占结果、速度/位置等跨线程共享数据
        private readonly object _robotLock = new object();

        // 静态障碍物地图（线程安全：内部有自己的锁）
        private readonly ObstacleMap _obstacleMap;

        // 仿真离散参数：网格大小（米）、仿真步长（秒）、网格数量（宽高一致）
        private readonly double _cellSizeM;
        private readonly double _dt;
        private readonly int _gridCount;

        // 世界尺寸（米），用于边界限制与坐标换算
        private readonly double _worldWidthM;
        private readonly double _worldHeightM;

        // 随机生成器：用于随机生成机器人初始格、随机目标
        private readonly Random _rng = new Random();

        // 所有机器人实例（Id 通常为 0..Count-1）
        private readonly List<RobotInstance> _robots = new List<RobotInstance>();

        // UI 选中的机器人（-1 表示未选中）
        private int _selectedRobotId = -1;

        // --- 碰撞让步冷却（目前用于避免频繁让步时抖动/震荡） ---
        private const int YieldCooldownFrames = 12; // 12 帧 * 20ms ≈ 240ms
        private readonly Dictionary<int, int> _yieldCooldownTicks = new Dictionary<int, int>(); // robotId -> 冷却剩余帧数

        // 格子锁抢占板：负责对路径格子进行抢占/释放，避免多机器人同格冲突
        private readonly GridCellClaimBoard _claimBoard;

        // 全局运行开关：false 则 Tick() 直接 return（UI 暂停）
        private bool _isRunning;

        // 单机器人暂停：不会影响其它机器人；Tick 中遇到 paused 机器人会强制停车且不派发指令
        private readonly HashSet<int> _pausedRobotIds = new HashSet<int>();

        // 边走边释放：记录每台机器人上一帧所在格子，检测换格后释放“已走出”的旧格锁
        private readonly Dictionary<int, GridPos> _lastGridCellByRobotId = new Dictionary<int, GridPos>();

        public RobotEngine(int gridCount, double cellSizeM, double dt, double initialMaxSpeed, EnumMoveDirection initialDirection)
        {
            // 初始化世界与仿真参数
            _gridCount = gridCount;
            _cellSizeM = cellSizeM;
            _dt = dt;

            // 世界尺寸 = 网格数 * 单格尺寸
            _worldWidthM = gridCount * cellSizeM;
            _worldHeightM = gridCount * cellSizeM;

            // 静态障碍地图
            _obstacleMap = new ObstacleMap(gridCount, gridCount);

            // 格子锁抢占板：需要知道地图尺寸与障碍（障碍格天然不可抢占）
            _claimBoard = new GridCellClaimBoard(_gridCount, _obstacleMap);

            // 构造时至少一个机器人
            SetRobotCount(1, initialMaxSpeed, initialDirection);

            // 默认不运行（等待 UI 点击“启动”）
            _processState = EnumRobotProcessState.Idle;
            _isRunning = false;
        }

        // 对外暴露世界与网格参数（供 UI 绘制/坐标换算）
        public double WorldWidthM => _worldWidthM;
        public double WorldHeightM => _worldHeightM;
        public double CellSizeM => _cellSizeM;
        public int GridCount => _gridCount;

        /// <summary>
        /// 当前选中机器人 Id（线程安全）。
        /// </summary>
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

        /// <summary>
        /// 全局启动：
        /// - 恢复 Tick 推进；
        /// - 清空所有单机暂停；
        /// - 确保所有机器人启用自动导航。
        /// </summary>
        public void StartAll()
        {
            lock (_robotLock)
            {
                _isRunning = true;
                ChangeProcessState_NoLock(EnumRobotProcessState.AutoNavigating);

                _pausedRobotIds.Clear();
                // 关键修复：启动前先重绑 walkable，保证 Enable()->RebuildPath 使用正确的障碍判定
                RebindDynamicWalkable_NoLock();

                for (int i = 0; i < _robots.Count; i++)
                {
                    if (!_robots[i].AutoNavigator.IsEnabled)
                    {
                        _robots[i].AutoNavigator.Enable();
                    }
                }
            }
        }

        /// <summary>
        /// 全局暂停：
        /// - 停止 Tick 推进；
        /// - 硬停所有机器人（不清目标、不清路径，后续 StartAll 可继续）。
        /// </summary>
        public void PauseAll()
        {
            lock (_robotLock)
            {
                _isRunning = false;
                ChangeProcessState_NoLock(EnumRobotProcessState.Idle);

                _pausedRobotIds.Clear();

                for (int i = 0; i < _robots.Count; i++)
                {
                    _robots[i].Speed = 0.0;
                    _robots[i].Manager.Acc = 0.0;
                    _robots[i].Move.StopImmediately_NoLock();
                }
            }
        }

        /// <summary>
        /// 清空选中机器人（UI 不再对任何机器人施加“选中逻辑”）。
        /// </summary>
        public void ClearSelectedRobot()
        {
            lock (_robotLock)
            {
                _selectedRobotId = -1;
            }
        }

        /// <summary>
        /// 当前寻路算法（对外提供统一入口）：
        /// - get：读取选中机器人的算法作为“当前配置”；
        /// - set：同步到所有机器人。
        /// </summary>
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

        /// <summary>
        /// 选中机器人是否启用自动导航（供 UI 状态显示）。
        /// </summary>
        public bool AutoEnabled
        {
            get
            {
                lock (_robotLock)
                {
                    if (_selectedRobotId < 0 || _selectedRobotId >= _robots.Count)
                    {
                        return false;
                    }

                    return _robots[_selectedRobotId].AutoNavigator.IsEnabled;
                }
            }
        }

        /// <summary>
        /// 获取障碍物地图快照（用于渲染）。
        /// </summary>
        public bool[,] GetObstacleSnapshot()
        {
            return _obstacleMap.GetSnapshot();
        }

        /// <summary>
        /// 切换指定格子的障碍状态。
        /// 自动模式下会触发所有启用自动的机器人重建路径（障碍改变会使旧路径失效）。
        /// </summary>
        public void ToggleObstacle(GridPos p)
        {
            _obstacleMap.Toggle(p);

            // 障碍编辑模式下不重规划：避免一边涂障碍一边机器人不断停/重算
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

        /// <summary>
        /// 清空所有障碍物。
        /// 自动模式下会触发所有启用自动的机器人重建路径。
        /// </summary>
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

        /// <summary>
        /// 设置机器人数量：
        /// - 多 -> 少：删除尾部机器人
        /// - 少 -> 多：新增机器人，随机放到空闲格，并给随机目标以确保其能动起来
        /// - 完成后重绑动态可行走判定（把“其它机器人所在格”当作动态障碍）
        /// </summary>
        public void SetRobotCount(int count, double initialMaxSpeed, EnumMoveDirection initialDirection)
        {
            if (count < 1)
            {
                count = 1;
            }

            lock (_robotLock)
            {
                // 1) 删除多余机器人（从尾部删，保持前面不动）
                while (_robots.Count > count)
                {
                    _robots.RemoveAt(_robots.Count - 1);
                }

                // 修正选中项，避免越界
                if (_selectedRobotId >= _robots.Count)
                {
                    _selectedRobotId = Math.Max(0, _robots.Count - 1);
                }

                // 2) 新增机器人（随机找空闲格）
                if (_robots.Count < count)
                {
                    var used = BuildUsedCellKeySet_NoLock();

                    while (_robots.Count < count)
                    {
                        int id = _robots.Count;

                        GridPos cell = PickRandomFreeCell_NoLock(used);
                        int key = cell.Y * _gridCount + cell.X;
                        used.Add(key);

                        // 初始位置放到格子中心（世界坐标）
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
                            initialY: y,
                            getGoalOwnerMap: () => BuildGoalOwnerMap_NoLock());

                        // 新机器人必须有目标，否则自动模式没有指令输出
                        r.AutoNavigator.Enable();
                        r.AutoNavigator.ClearGoal();
                        r.Manager.ResetAutoCommands();
                        r.AutoNavigator.SetGoal(PickRandomFreeCell_NoLock(used: null), rebuildIfEnabled: true);

                        // 继承当前全局加速度配置（从第一个机器人拷贝）
                        if (_robots.Count > 0)
                        {
                            r.Acc = _robots[0].Acc;
                        }

                        _robots.Add(r);
                    }
                }

                // 3) 重绑动态障碍：把所有机器人占用格注入 WalkableProvider / WorldWalkableProvider
                RebindDynamicWalkable_NoLock();

                // 4) 确保所有机器人自动启用（你当前需求：未选中机器人也应持续自动运行）
                for (int i = 0; i < _robots.Count; i++)
                {
                    if (!_robots[i].AutoNavigator.IsEnabled)
                    {
                        _robots[i].AutoNavigator.Enable();
                        _robots[i].AutoNavigator.ClearGoal();
                        _robots[i].Manager.ResetAutoCommands();
                        _robots[i].AutoNavigator.SetGoal(PickRandomFreeCell_NoLock(used: null), rebuildIfEnabled: true);
                    }
                }
            }
        }

        /// <summary>
        /// 构建所有机器人当前占用格 key（y*W+x）集合（要求已持有 _robotLock）。
        /// 用途：新增机器人时避免出生点重叠。
        /// </summary>
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

        /// <summary>
        /// 将引擎重置成单机器人（保留接口，当前随机巡航逻辑已被注释/关闭）。
        /// </summary>
        public void ResetToSingleRobotRandomRoam(double initialMaxSpeed, EnumMoveDirection initialDirection)
        {
            lock (_robotLock)
            {
                SetRobotCount(1, initialMaxSpeed, initialDirection);

                RobotInstance r0 = _robots[0];
                r0.AutoNavigator.ClearGoal();
                r0.Manager.ResetAutoCommands();
            }
        }

        /// <summary>
        /// 选中某台机器人：
        /// - 更新 _selectedRobotId；
        /// - 将其加入“单机暂停集合”；
        /// - 强制停车（等待用户设置目标或进入手动）。
        /// </summary>
        public bool SelectRobot(int id)
        {
            lock (_robotLock)
            {
                if (id < 0 || id >= _robots.Count)
                {
                    return false;
                }

                _selectedRobotId = id;

                RobotInstance r = _robots[id];
                _pausedRobotIds.Add(r.Id);

                r.Speed = 0.0;
                r.Manager.Acc = 0.0;
                r.Move.StopImmediately_NoLock();

                return true;
            }
        }

        /// <summary>
        /// 获取所有机器人状态快照（供渲染线程读取）。
        /// </summary>
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

        /// <summary>
        /// 获取选中机器人状态快照（未选中时返回 default）。
        /// </summary>
        public RobotStateSnapshot GetStateSnapshot()
        {
            lock (_robotLock)
            {
                if (_selectedRobotId < 0 || _selectedRobotId >= _robots.Count)
                {
                    return default(RobotStateSnapshot);
                }

                return _robots[_selectedRobotId].GetSnapshot();
            }
        }

        /// <summary>
        /// 获取选中机器人的路径点（世界坐标）快照，用于 UI 绘制。
        /// </summary>
        public List<(double X, double Y)> GetPathWorldPointsSnapshot()
        {
            lock (_robotLock)
            {
                if (_selectedRobotId < 0 || _selectedRobotId >= _robots.Count)
                {
                    return null;
                }

                return _robots[_selectedRobotId].AutoNavigator.GetPathWorldPointsSnapshot();
            }
        }

        /// <summary>
        /// 获取所有机器人的抢占格子快照（用于渲染显示“格子锁”效果）。
        /// </summary>
        public Dictionary<int, List<GridPos>> GetClaimedCellsSnapshot()
        {
            lock (_robotLock)
            {
                return _claimBoard.GetClaimedCellsSnapshot();
            }
        }

        /// <summary>
        /// 获取所有机器人目标点世界坐标快照（用于渲染）。
        /// </summary>
        public List<(int Id, double X, double Y)> GetRobotsGoalWorldSnapshot()
        {
            lock (_robotLock)
            {
                var list = new List<(int Id, double X, double Y)>(_robots.Count);

                for (int i = 0; i < _robots.Count; i++)
                {
                    var g = _robots[i].AutoNavigator.GetGoalWorldSnapshot();
                    if (g.HasValue)
                    {
                        list.Add((_robots[i].Id, g.Value.X, g.Value.Y));
                    }
                }

                return list;
            }
        }

        /// <summary>
        /// 设置前进加速度（同步到所有机器人）。
        /// 自动模式下需要 ResetAutoCommands，使下一帧派发的 MoveDistance 读取到新加速度。
        /// </summary>
        public void SetForwardAcc(double acc)
        {
            lock (_robotLock)
            {
                for (int i = 0; i < _robots.Count; i++)
                {
                    _robots[i].Acc = acc;
                    _robots[i].Manager.ResetAutoCommands();
                }
            }
        }

        /// <summary>
        /// 设置最大速度（同步到所有机器人）。
        /// </summary>
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

        /// <summary>
        /// 切到自动模式（当前设计：只对“选中机器人”做对齐/清命令；未选中机器人由 Tick 保持自动）。
        /// </summary>
        public void EnableAuto()
        {
            lock (_robotLock)
            {
                ChangeProcessState_NoLock(EnumRobotProcessState.AutoNavigating);

                if (_selectedRobotId < 0 || _selectedRobotId >= _robots.Count)
                {
                    return;
                }

                RobotInstance r = _robots[_selectedRobotId];
                r.Manual.Disable();
                r.Manager.SetMode(EnumRobotControlMode.Auto);

                r.AutoNavigator.Enable();
                r.Manager.ResetAutoCommands();
                r.Manager.AlignOrientationToDirectionWithTurn();
            }
        }

        /// <summary>
        /// 切到手动模式：
        /// - 禁用选中机器人的自动导航；
        /// - 释放该机器人格子锁（手动不按路径走，否则会长期占用锁）；
        /// - 立即停车并清命令；
        /// - 启用手动输入并解除单机暂停。
        /// </summary>
        public void EnableManual()
        {
            lock (_robotLock)
            {
                _isRunning = true;
                ChangeProcessState_NoLock(EnumRobotProcessState.ManualControl);

                if (_selectedRobotId < 0 || _selectedRobotId >= _robots.Count)
                {
                    return;
                }

                RobotInstance r = _robots[_selectedRobotId];

                if (r.AutoNavigator.IsEnabled)
                {
                    r.AutoNavigator.Disable();
                }

                _claimBoard.ReleaseAllByRobot(r.Id);
                _lastGridCellByRobotId.Remove(r.Id);
                r.AutoNavigator.SetClaimedPathPrefix(null);

                r.Speed = 0.0;
                r.Manager.Acc = 0.0;
                r.Move.StopImmediately_NoLock();
                r.Manager.ResetAutoCommands();

                r.Manual.Enable();
                _pausedRobotIds.Remove(r.Id);
            }
        }

        /// <summary>
        /// 触发选中机器人重建路径（用于 UI 手动点击“重建路径”）。
        /// </summary>
        public void RebuildPath()
        {
            lock (_robotLock)
            {
                if (_selectedRobotId < 0 || _selectedRobotId >= _robots.Count)
                {
                    return;
                }

                _robots[_selectedRobotId].AutoNavigator.RebuildPath();
            }
        }

        /// <summary>
        /// 手动：前进键按下/抬起（W）。
        /// 仅当选中机器人处于“手动启用”时才接受输入。
        /// </summary>
        public void ManualForwardKey(bool down)
        {
            lock (_robotLock)
            {
                if (_selectedRobotId < 0 || _selectedRobotId >= _robots.Count)
                {
                    return;
                }

                RobotInstance r = _robots[_selectedRobotId];

                if (!r.Manual.IsEnabled)
                {
                    return;
                }

                r.Manual.InputForwardKey(down);
                _pausedRobotIds.Remove(r.Id);
            }
        }

        /// <summary>
        /// 手动：左转键按下/抬起（A）。
        /// </summary>
        public void ManualTurnLeftKey(bool down)
        {
            lock (_robotLock)
            {
                if (_selectedRobotId < 0 || _selectedRobotId >= _robots.Count)
                {
                    return;
                }

                RobotInstance r = _robots[_selectedRobotId];

                if (!r.Manual.IsEnabled)
                {
                    return;
                }

                r.Manual.InputTurnLeftKey(down);
                _pausedRobotIds.Remove(r.Id);
            }
        }

        /// <summary>
        /// 手动：右转键按下/抬起（D）。
        /// </summary>
        public void ManualTurnRightKey(bool down)
        {
            lock (_robotLock)
            {
                if (_selectedRobotId < 0 || _selectedRobotId >= _robots.Count)
                {
                    return;
                }

                RobotInstance r = _robots[_selectedRobotId];

                if (!r.Manual.IsEnabled)
                {
                    return;
                }

                r.Manual.InputTurnRightKey(down);
                _pausedRobotIds.Remove(r.Id);
            }
        }

        /// <summary>
        /// 给选中机器人设置目标格：
        /// - 设置前释放其旧路径锁（避免锁残留导致其它机器人永远抢不到某些格子）。
        /// </summary>
        public bool TrySetSelectedRobotGoal(GridPos goal)
        {
            lock (_robotLock)
            {
                if (_selectedRobotId < 0 || _selectedRobotId >= _robots.Count)
                {
                    return false;
                }

                RobotInstance r = _robots[_selectedRobotId];

                _claimBoard.ReleaseAllByRobot(r.Id);
                _lastGridCellByRobotId.Remove(r.Id);

                r.AutoNavigator.SetGoal(goal, rebuildIfEnabled: true);
                _pausedRobotIds.Remove(r.Id);

                return true;
            }
        }

        /// <summary>
        /// 切换障碍编辑模式：
        /// - enabled=true：暂停所有运动（禁用自动/手动）
        /// - enabled=false：恢复自动巡航，并重置自动命令
        /// </summary>
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
                    // 关键修复：先重绑 walkable（包含静态障碍），再 Enable() 触发 RebuildPath
                    RebindDynamicWalkable_NoLock();

                    for (int i = 0; i < _robots.Count; i++)
                    {
                        _robots[i].AutoNavigator.Enable();
                        _robots[i].Manager.ResetAutoCommands();
                    }
                }
            }
        }

        /// <summary>
        /// 仿真主循环入口（由 RobotSimulationLoop 在后台线程中每 dt 调用一次）。
        /// 一帧 Tick 做的事：
        /// 1) 冷却计数衰减
        /// 2) 绑定动态 walkable（静态障碍 + 其他机器人占用格 + 其他机器人目标格）
        /// 3) 路径抢占（完整路径 -> 抢占前缀）并回灌到导航器
        /// 4) 对每台机器人：派发指令 -> 更新运动学 -> 释放已走出的格子锁
        /// </summary>
        public void Tick()
        {
            if (!_isRunning)
            {
                return;
            }

            // 不推进仿真的状态直接 return（避免编辑障碍时机器人乱动）
            switch (_processState)
            {
                case EnumRobotProcessState.ObstacleEditing:
                case EnumRobotProcessState.Idle:
                case EnumRobotProcessState.Error:
                    return;
            }

            lock (_robotLock)
            {
                // 让步冷却推进（每帧减 1，归零则移除）
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

                // 1) 动态可通行性重绑（把其它机器人占用格当动态障碍）
                RebindDynamicWalkable_NoLock();

                // 2) 先做格子锁抢占，再出指令（保证出指令一定在“已抢占前缀”内）
                ApplyPathClaiming_NoLock();

                // 3) 逐机器人更新
                for (int i = 0; i < _robots.Count; i++)
                {
                    RobotInstance r = _robots[i];
                    bool isSelected = r.Id == _selectedRobotId;

                    // 单机暂停：该机器人不派发指令，强制停车
                    bool isPausedBySingle = _pausedRobotIds.Contains(r.Id);
                    if (isPausedBySingle)
                    {
                        r.Speed = 0.0;
                        r.Manager.Acc = 0.0;
                        r.Move.StopImmediately_NoLock();
                        r.Move.Update();
                        continue;
                    }

                    // 未选中机器人：强制处于自动控制（避免 UI 残留手动状态）
                    if (!isSelected)
                    {
                        r.Manual.Disable();
                        if (!r.AutoNavigator.IsEnabled)
                        {
                            r.AutoNavigator.Enable();
                        }
                    }

                    // 未选中机器人如果没路径，给随机目标以继续“巡航”
                    if (!isSelected && r.AutoNavigator.IsEnabled)
                    {
                        List<(double X, double Y)> path = r.AutoNavigator.GetPathWorldPointsSnapshot();
                        if (path == null || path.Count == 0)
                        {
                            r.AutoNavigator.SetGoal(PickRandomFreeCell_NoLock(used: null), rebuildIfEnabled: true);
                            r.Manager.ResetAutoCommands();
                        }
                    }

                    // RobotManager.Tick：保留为统一生命周期入口（本项目内目前为空实现）
                    r.Manager.Tick(_dt);

                    // 自动模式：沿“抢占路径前缀”生成指令；生成失败则停车等待下一帧抢占
                    if (r.AutoNavigator.IsEnabled)
                    {
                        RobotCommand cmd = r.AutoNavigator.TryBuildNextCommandFromClaimedPath();
                        if (cmd != null)
                        {
                            r.Manager.DispatchDirect_NoLock(cmd);
                        }
                        else
                        {
                            r.Move.StopImmediately_NoLock();
                        }
                    }
                    // 手动模式：仅对选中机器人派发手动命令
                    else if (r.Manual.IsEnabled && r.Id == _selectedRobotId)
                    {
                        RobotCommand cmd = r.Manual.TryBuildNextCommand();
                        r.Manager.DispatchDirect_NoLock(cmd);
                    }

                    // 运动学执行：推进位置/速度/朝向
                    r.Move.Update();

                    // 边走边释放：换格后释放上一个格子的锁，降低堵塞
                    ReleaseClaimByMovement_NoLock(r);
                }
            }
        }

        /// <summary>
        /// 对每台机器人执行路径格子锁抢占，并把抢占到的“路径前缀”回灌给对应导航器。
        /// 抢占顺序：按“距目标 Manhattan 距离”由近到远，近者更容易抢到整段，降低临近终点互堵。
        /// </summary>
        private void ApplyPathClaiming_NoLock()
        {
            var items = new List<(RobotInstance R, int Dist)>(_robots.Count);

            // 收集机器人及其到目标的距离
            for (int i = 0; i < _robots.Count; i++)
            {
                RobotInstance r = _robots[i];

                GridPos? goal = r.AutoNavigator.GetGoalGridSnapshot();
                if (!goal.HasValue)
                {
                    // 无目标：释放全部锁，清空 claimed 前缀，避免残留占用影响其它机器人
                    _claimBoard.ReleaseAllByRobot(r.Id);
                    r.AutoNavigator.SetClaimedPathPrefix(null);
                    continue;
                }

                GridPos cur = r.GetGridPos_NoLock();
                int dist = Math.Abs(cur.X - goal.Value.X) + Math.Abs(cur.Y - goal.Value.Y);
                items.Add((r, dist));
            }

            // 距离近优先；距离相同按 Id 排序，确保确定性
            items.Sort((a, b) =>
            {
                int c = a.Dist.CompareTo(b.Dist);
                if (c != 0) return c;
                return a.R.Id.CompareTo(b.R.Id);
            });

            // 逐台抢占并回灌 claimed 前缀
            for (int i = 0; i < items.Count; i++)
            {
                RobotInstance r = items[i].R;

                // 引擎抢占基于“完整路径快照”
                List<GridPos> path = r.AutoNavigator.GetPathGridSnapshot();

                // 没路径时尝试重建
                if ((path == null || path.Count == 0) && r.AutoNavigator.IsEnabled)
                {
                    r.AutoNavigator.RebuildPath();
                    path = r.AutoNavigator.GetPathGridSnapshot();
                }

                if (path == null || path.Count == 0)
                {
                    _claimBoard.ReleaseAllByRobot(r.Id);
                    r.AutoNavigator.SetClaimedPathPrefix(null);
                    continue;
                }

                // 从当前所在格开始抢占（若 path 中包含当前格，则 startIndex 指向该格）
                GridPos curCell = r.GetGridPos_NoLock();
                int startIndex = 0;

                for (int j = 0; j < path.Count; j++)
                {
                    if (path[j].Equals(curCell))
                    {
                        startIndex = j;
                        break;
                    }
                }

                // 抢占到“终点”或“可抢占的最长前缀”，并回灌
                List<GridPos> claimedPrefix = _claimBoard.ClaimPathToGoalOrPrefix(r.Id, path, startIndex);
                r.AutoNavigator.SetClaimedPathPrefix(claimedPrefix);
            }
        }

        /// <summary>
        /// 边走边释放：当机器人换格后，释放其上一个格子的锁。
        /// 目的：让后车更快抢到后续格，降低堵塞。
        /// </summary>
        private void ReleaseClaimByMovement_NoLock(RobotInstance r)
        {
            GridPos current = r.GetGridPos_NoLock();

            GridPos last;
            if (!_lastGridCellByRobotId.TryGetValue(r.Id, out last))
            {
                _lastGridCellByRobotId[r.Id] = current;
                return;
            }

            if (!current.Equals(last))
            {
                _claimBoard.ReleaseCell(r.Id, last);
                _lastGridCellByRobotId[r.Id] = current;
            }
        }

        /// <summary>
        /// 修改引擎状态（要求调用方已持有锁）。
        /// </summary>
        private void ChangeProcessState_NoLock(EnumRobotProcessState newState)
        {
            _processState = newState;
        }

        /// <summary>
        /// 获取选中机器人实例（要求调用方已持有锁）。
        /// 若当前未选中或越界，会自动修正到一个有效索引。
        /// </summary>
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

        /// <summary>
        /// 随机选择一个非障碍且不在 used 集合中的格子（要求已持有锁）。
        /// used 通常用于“生成新机器人位置”时避免初始重叠。
        /// </summary>
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

            // 兜底返回：调用方需容错（(0,0) 可能仍不可用）
            return new GridPos(0, 0);
        }

        /// <summary>
        /// 重绑动态可通行判定（要求已持有锁）：
        /// - occupied：其它机器人当前占用格视为不可通行
        /// - goalOwnerByKey：其它机器人目标格视为不可通行（避免多机器人抢同终点）
        /// - 自己所在格允许（否则会把起点当障碍导致寻路失败）
        /// </summary>
        private void RebindDynamicWalkable_NoLock()
        {
            var occupied = new HashSet<int>(_robots.Count);
            var cellKeys = new int[_robots.Count];

            // 统计所有机器人当前占用格
            for (int i = 0; i < _robots.Count; i++)
            {
                GridPos c = _robots[i].GetGridPos_NoLock();
                int key = c.Y * _gridCount + c.X;
                cellKeys[i] = key;
                occupied.Add(key);
            }

            // 统计所有机器人目标格的“拥有者”
            var goalOwnerByKey = new Dictionary<int, int>();
            for (int i = 0; i < _robots.Count; i++)
            {
                var g = _robots[i].AutoNavigator.GetGoalGridSnapshot();
                if (g.HasValue)
                {
                    GridPos gp = g.Value;
                    if (gp.X >= 0 && gp.Y >= 0 && gp.X < _gridCount && gp.Y < _gridCount)
                    {
                        int k = gp.Y * _gridCount + gp.X;
                        if (!goalOwnerByKey.ContainsKey(k))
                        {
                            goalOwnerByKey.Add(k, _robots[i].Id);
                        }
                    }
                }
            }

            // 为每台机器人绑定两套判定：
            // - AutoNavigator 用 GridPos 判定寻路可通行
            // - Move 用世界坐标判定碰撞/穿越
            for (int i = 0; i < _robots.Count; i++)
            {
                RobotInstance me = _robots[i];
                int myKey = cellKeys[i];
                int myId = me.Id;

                me.AutoNavigator.SetIsWalkableProvider(p =>
                {
                    if (_obstacleMap.IsObstacle(p))
                    {
                        return false;
                    }

                    int key = p.Y * _gridCount + p.X;

                    // 允许走自己起点格
                    if (key == myKey)
                    {
                        return true;
                    }

                    // 禁止进入其它机器人终点格（自己的终点格必须可走）
                    int ownerId;
                    if (goalOwnerByKey.TryGetValue(key, out ownerId) && ownerId != myId)
                    {
                        return false;
                    }

                    // 禁止进入其它机器人当前占用格
                    return !occupied.Contains(key);
                });

                me.Move.SetIsWorldWalkableProvider((wx, wy) =>
                {
                    int gx = (int)Math.Floor(wx / _cellSizeM);
                    int gy = (int)Math.Floor(wy / _cellSizeM);

                    if (gx < 0 || gy < 0 || gx >= _gridCount || gy >= _gridCount)
                        return false;

                    var p = new GridPos(gx, gy);
                    if (_obstacleMap.IsObstacle(p))
                        return false;

                    int key = p.Y * _gridCount + p.X;

                    if (key == myKey)
                        return true;

                    // 修复点：
                    // Move/碰撞层不再把“其它机器人目标格”当成不可通行。
                    // 否则会出现：引擎已抢占该格（claim 成功），但 Move 判定不可走 -> Stop 卡死 -> 长期占锁。
                    // 目标格互斥应由“寻路层 + claimBoard”保证，而不是由 Move 层硬阻挡。
                    return !occupied.Contains(key);
                });
            }
        }

        /// <summary>
        /// 构建“终点拥有者”映射（要求已持有锁）：cellKey -> robotId。
        /// 给 AutoNavigator.RebuildPath_NoLock 用于把其它机器人的目标格视为不可走。
        /// </summary>
        internal Dictionary<int, int> BuildGoalOwnerMap_NoLock()
        {
            var goalOwnerByKey = new Dictionary<int, int>();

            for (int i = 0; i < _robots.Count; i++)
            {
                var g = _robots[i].AutoNavigator.GetGoalGridSnapshot();
                if (g.HasValue)
                {
                    GridPos gp = g.Value;
                    if (gp.X >= 0 && gp.Y >= 0 && gp.X < _gridCount && gp.Y < _gridCount)
                    {
                        int k = gp.Y * _gridCount + gp.X;
                        if (!goalOwnerByKey.ContainsKey(k))
                        {
                            goalOwnerByKey.Add(k, _robots[i].Id);
                        }
                    }
                }
            }

            return goalOwnerByKey;
        }
    }

    /// <summary>
    /// 机器人状态快照（值类型）：用于跨线程安全读取（UI 线程绘制）。
    /// 注意：这里不包含 Id，调用方通常按列表索引对应机器人 Id。
    /// </summary>
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