using GridDemo.MultiRobots;
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
        Idle,             // 空闲：不更新仿真
        AutoNavigating,   // 自动巡航：自动寻路 + 移动
        ManualControl,    // 手动控制：键盘等输入控制
        ObstacleEditing,  // 障碍编辑：暂停运动，仅编辑地图
        Error             // 错误：不更新仿真
    }

    /// <summary>
    /// 多机器人仿真/调度引擎（重构版）：
    /// - 作为"外观层"，对 UI 暴露统一接口；
    /// - 把复杂逻辑下沉到：
    ///    * RobotWorld：世界/机器人集合/障碍图等基础数据
    ///    * DynamicWalkableBinder：动态可通行性绑定
    ///    * PathClaimManager：路径格子锁抢占 + 死锁/让步
    ///    * RobotControlManager：自动/手动控制 + 单机暂停 + 手动输入
    ///
    /// Tick 过程：
    /// 1) PathClaimManager 冷却计数衰减（让步冷却、死锁淡出）
    /// 2) DynamicWalkableBinder 重绑 walkable（静态障碍 + 其他机器人占用格 + 目标格）
    /// 3) PathClaimManager 路径抢占（完整路径 -> 抢占前缀）
    /// 4) 对每台机器人：
    ///    - 自动：按抢占前缀发指令；
    ///    - 手动：根据键盘输入发指令；
    ///    - 执行 Move.Update()，然后边走边释放旧格子锁。
    /// </summary>
    internal sealed class RobotEngine
    {
        // 全局互斥锁：保护所有跨线程共享状态（世界/机器人/路径/锁等）
        private readonly object _robotLock = new object();

        // 世界基础数据（障碍图、机器人集合、网格参数等）
        private readonly RobotWorld _world;

        // 动态可通行性绑定（基于其它机器人位置/终点）
        private readonly DynamicWalkableBinder _walkableBinder;

        // 路径格子锁抢占 + 死锁/让步管理
        private readonly PathClaimManager _pathClaimManager;

        // 控制模式与手动输入/单机暂停
        private readonly RobotControlManager _control;

        // 当前引擎状态
        private EnumRobotProcessState _processState = EnumRobotProcessState.Idle;

        // 全局运行开关：false 则 Tick 直接 return（UI 暂停）
        private bool _isRunning;

        // 仅为便于保持原 API 属性，镜像保留世界/网格参数
        private readonly int _gridCount;
        private readonly double _cellSizeM;
        private readonly double _dt;
        private readonly double _worldWidthM;
        private readonly double _worldHeightM;

        /// <summary>
        /// 到达终点后停顿秒数（所有机器人共用）。
        /// </summary>
        private const double ArrivalPauseDurationS = 2.0;

        /// <summary>
        /// 到达目标格中心的世界坐标容差（米），与 AutoNavigator.ArriveEpsilonM 保持一致。
        /// </summary>
        private const double ArriveEpsilonM = 0.05;

        /// <summary>
        /// 构造引擎：
        /// - 初始化世界与仿真参数；
        /// - 创建 RobotWorld / PathClaimManager / DynamicWalkableBinder / RobotControlManager；
        /// - 至少创建 1 台机器人并加入世界；
        /// - 默认不运行（等待 UI 点击"启动"）。
        /// </summary>
        public RobotEngine(int gridCount, double cellSizeM, double dt,
                           double initialMaxSpeed, EnumMoveDirection initialDirection)
        {
            _gridCount = gridCount;
            _cellSizeM = cellSizeM;
            _dt = dt;

            _worldWidthM = gridCount * cellSizeM;
            _worldHeightM = gridCount * cellSizeM;

            _world = new RobotWorld(_robotLock, gridCount, cellSizeM, dt);
            _pathClaimManager = new PathClaimManager(_world);
            _walkableBinder = new DynamicWalkableBinder(_world);
            _control = new RobotControlManager(_world);

            // 创建至少 1 台机器人
            SetRobotCount(1, initialMaxSpeed, initialDirection);

            _processState = EnumRobotProcessState.Idle;
            _isRunning = false;
        }

        #region 对外属性/基础信息

        public double WorldWidthM => _worldWidthM;
        public double WorldHeightM => _worldHeightM;
        public double CellSizeM => _cellSizeM;
        public int GridCount => _gridCount;

        /// <summary> 当前选中机器人 Id（线程安全）。 </summary>
        public int SelectedRobotId
        {
            get
            {
                lock (_robotLock)
                {
                    return _world.SelectedRobotId;
                }
            }
        }

        /// <summary>
        /// 当前寻路算法（读：选中机器人；写：同步所有机器人）。
        /// 实际委托给 RobotControlManager。
        /// </summary>
        public EnumPathfindingAlgorithm Algorithm
        {
            get => _control.Algorithm;
            set => _control.Algorithm = value;
        }

        /// <summary> 选中机器人是否启用自动导航。 </summary>
        public bool AutoEnabled => _control.AutoEnabled;

        /// <summary> 获取障碍物地图快照（用于渲染）。 </summary>
        public bool[,] GetObstacleSnapshot()
        {
            return _world.ObstacleMap.GetSnapshot();
        }

        #endregion

        #region 全局启动/暂停 & 选中管理

        /// <summary>
        /// 全局启动：
        /// - 恢复 Tick 推进；
        /// - 清空所有单机暂停；
        /// - 重绑 walkable；
        /// - 确保所有机器人启用自动导航。
        /// </summary>
        public void StartAll()
        {
            lock (_robotLock)
            {
                _isRunning = true;
                ChangeProcessState_NoLock(EnumRobotProcessState.AutoNavigating);

                _control.ClearAllPause();
                _walkableBinder.RebindDynamicWalkable_NoLock();

                foreach (var r in _world.Robots)
                {
                    if (!r.AutoNavigator.IsEnabled)
                        r.AutoNavigator.Enable();
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

                _control.ClearAllPause();

                foreach (var r in _world.Robots)
                {
                    r.Speed = 0.0;
                    r.Manager.Acc = 0.0;
                    r.Move.StopImmediately_NoLock();
                }
            }
        }

        /// <summary>
        /// 清空选中机器人（UI 不再对任何机器人施加"选中逻辑"）。
        /// </summary>
        public void ClearSelectedRobot()
        {
            _world.SelectedRobotId = -1;
        }

        /// <summary>
        /// 选中某台机器人：
        /// - 更新 SelectedRobotId；
        /// - 清除对该机器人的单机暂停，使其继续运动。
        /// </summary>
        public bool SelectRobot(int id)
        {
            lock (_robotLock)
            {
                if (id < 0 || id >= _world.Robots.Count)
                    return false;

                _world.SelectedRobotId = id;
                _control.ResumeRobot(id);
                return true;
            }
        }

        #endregion

        #region 障碍物编辑

        /// <summary>
        /// 切换指定格子的障碍状态。
        /// 自动模式下会触发所有启用自动的机器人重建路径。
        /// 障碍编辑模式下不重规划。
        /// </summary>
        public void ToggleObstacle(GridPos p)
        {
            _world.ObstacleMap.Toggle(p);

            if (_processState == EnumRobotProcessState.ObstacleEditing)
                return;

            lock (_robotLock)
            {
                foreach (var r in _world.Robots)
                {
                    if (r.AutoNavigator.IsEnabled)
                        r.AutoNavigator.RebuildPath();
                }
            }
        }

        /// <summary>
        /// 清空所有障碍物。
        /// 自动模式下会触发所有启用自动的机器人重建路径。
        /// 障碍编辑模式下不重规划。
        /// </summary>
        public void ClearObstacles()
        {
            _world.ObstacleMap.Clear();

            if (_processState == EnumRobotProcessState.ObstacleEditing)
                return;

            lock (_robotLock)
            {
                foreach (var r in _world.Robots)
                {
                    if (r.AutoNavigator.IsEnabled)
                        r.AutoNavigator.RebuildPath();
                }
            }
        }

        /// <summary>
        /// 切换障碍编辑模式：
        /// - enabled=true：暂停所有运动（禁用自动/手动，停止命令）
        /// - enabled=false：恢复自动巡航，并重绑 walkable 后重新启用 AutoNavigator。
        /// </summary>
        public void SetObstacleEditMode(bool enabled)
        {
            lock (_robotLock)
            {
                if (enabled)
                {
                    ChangeProcessState_NoLock(EnumRobotProcessState.ObstacleEditing);

                    foreach (var r in _world.Robots)
                    {
                        r.Speed = 0.0;
                        r.Manager.Acc = 0.0;
                        r.Move.StopImmediately_NoLock();

                        if (r.AutoNavigator.IsEnabled)
                            r.AutoNavigator.Disable();

                        r.Manual.Disable();
                        r.Manager.ResetAutoCommands();
                    }
                }
                else
                {
                    ChangeProcessState_NoLock(EnumRobotProcessState.AutoNavigating);

                    // 先重绑 walkable（包含静态障碍），再 Enable()
                    _walkableBinder.RebindDynamicWalkable_NoLock();

                    foreach (var r in _world.Robots)
                    {
                        r.AutoNavigator.Enable();
                        r.Manager.ResetAutoCommands();
                    }
                }
            }
        }

        /// <summary>
        /// 保存当前障碍物地图到文件。
        /// </summary>
        /// <param name="filePath">目标文件路径。</param>
        public void SaveMap(string filePath)
        {
            bool[,] snapshot = _world.ObstacleMap.GetSnapshot();
            Maps.MapFileService.Save(filePath, _gridCount, _cellSizeM, snapshot);
        }

        /// <summary>
        /// 从文件载入障碍物地图：
        /// 1) 校验文件中 GridCount 与引擎一致；
        /// 2) 清空当前障碍物；
        /// 3) 按文件数据设置障碍物；
        /// 4) 自动模式下触发所有机器人重建路径。
        /// </summary>
        /// <param name="filePath">地图文件路径。</param>
        public void LoadMap(string filePath)
        {
            Maps.MapFileService.Load(filePath, out int fileGridCount, out double fileCellSizeM,
                out System.Collections.Generic.List<GridPos> obstacles);

            if (fileGridCount != _gridCount)
            {
                throw new System.ArgumentException(
                    "地图文件 GridCount(" + fileGridCount + ") 与当前引擎 GridCount(" + _gridCount + ") 不一致，无法载入。");

            }

            lock (_robotLock)
            {
                // 清空现有障碍物
                _world.ObstacleMap.Clear();

                // 逐个设置障碍物（Clear 后全为 false，Toggle 一次变为 true）
                for (int i = 0; i < obstacles.Count; i++)
                {
                    _world.ObstacleMap.Toggle(obstacles[i]);
                }

                // 非障碍编辑模式下，触发所有自动机器人重建路径
                if (_processState != EnumRobotProcessState.ObstacleEditing)
                {
                    foreach (var r in _world.Robots)
                    {
                        if (r.AutoNavigator.IsEnabled)
                            r.AutoNavigator.RebuildPath();
                    }
                }
            }
        }

        #endregion

        #region 机器人数量 / 初始化 / 重置

        /// <summary>
        /// 设置机器人数量：
        /// - 多 -> 少：删除尾部机器人（先清理其所有抢占/状态）；
        /// - 少 -> 多：新增机器人，随机放到空闲格，并给随机目标；
        /// - 最后重绑动态 walkable。
        /// </summary>
        public void SetRobotCount(int count, double initialMaxSpeed, EnumMoveDirection initialDirection)
        {
            if (count < 1)
                count = 1;

            lock (_robotLock)
            {
                var robots = _world.Robots;

                // 1) 删除多余机器人（从尾部删，保持前面不动）
                while (robots.Count > count)
                {
                    int removeIndex = robots.Count - 1;
                    int removeRobotId = robots[removeIndex].Id;

                    // 清理该机器人所有抢占/冷却/历史状态
                    _pathClaimManager.ClearRobotState(removeRobotId);

                    // 清理导航器侧已声明的路径前缀
                    robots[removeIndex].AutoNavigator.SetClaimedPathPrefix(null);

                    robots.RemoveAt(removeIndex);
                }

                // 修正选中项，避免越界
                if (_world.SelectedRobotId >= robots.Count)
                    _world.SelectedRobotId = robots.Count > 0 ? robots.Count - 1 : -1;

                // 2) 新增机器人（随机找空闲格）
                var newRobotIds = new List<int>();

                if (robots.Count < count)
                {
                    var used = _world.BuildUsedCellKeySet_NoLock();

                    while (robots.Count < count)
                    {
                        int id = robots.Count;

                        GridPos cell = _world.PickRandomFreeCell_NoLock(used);
                        int key = cell.Y * _gridCount + cell.X;
                        used.Add(key);

                        double x = cell.X * _cellSizeM + _cellSizeM / 2.0;
                        double y = cell.Y * _cellSizeM + _cellSizeM / 2.0;

                        var r = new RobotInstance(
                            id: id,
                            robotLock: _robotLock,
                            obstacleMap: _world.ObstacleMap,
                            gridCount: _gridCount,
                            cellSizeM: _cellSizeM,
                            dt: _dt,
                            worldWidthM: _worldWidthM,
                            worldHeightM: _worldHeightM,
                            initialMaxSpeed: initialMaxSpeed,
                            initialDirection: initialDirection,
                            initialX: x,
                            initialY: y,
                            getGoalOwnerMap: () => _world.BuildGoalOwnerMap_NoLock());

                        // 继承当前全局加速度配置（从第一个机器人拷贝）
                        if (robots.Count > 0)
                            r.Acc = robots[0].Acc;

                        robots.Add(r);
                        newRobotIds.Add(r.Id);
                    }
                }

                // 3) 重绑动态 walkable
                _walkableBinder.RebindDynamicWalkable_NoLock();

                // 4) 对"本次新建机器人"，在 walkable 已绑定后再 Enable/SetGoal（触发寻路）
                foreach (int newId in newRobotIds)
                {
                    RobotInstance r = _world.Robots[newId];
                    r.AutoNavigator.Enable();
                    r.AutoNavigator.ClearGoal();
                    r.Manager.ResetAutoCommands();
                    r.AutoNavigator.SetGoal(
                        _world.PickRandomFreeCell_NoLock(used: null),
                        rebuildIfEnabled: true);
                }

                // 5) 确保所有机器人自动启用，未选中机器人也持续自动运行
                foreach (var r in robots)
                {
                    if (!r.AutoNavigator.IsEnabled)
                    {
                        r.AutoNavigator.Enable();
                        r.AutoNavigator.ClearGoal();
                        r.Manager.ResetAutoCommands();
                        r.AutoNavigator.SetGoal(
                            _world.PickRandomFreeCell_NoLock(used: null),
                            rebuildIfEnabled: true);
                    }
                }
            }
        }

        /// <summary>
        /// 将引擎重置成单机器人（保留接口，当前为随机巡航逻辑）。
        /// </summary>
        public void ResetToSingleRobotRandomRoam(double initialMaxSpeed, EnumMoveDirection initialDirection)
        {
            lock (_robotLock)
            {
                // 1) 重置后必须停在 Idle，等待 UI 点击"启动"
                _isRunning = false;
                ChangeProcessState_NoLock(EnumRobotProcessState.Idle);

                // 2) 重置为单机器人
                SetRobotCount(1, initialMaxSpeed, initialDirection);

                // 3) 清理引擎侧"路径/抢占/释放/让步冷却/暂停"等残留
                _control.ClearAllPause();
                _pathClaimManager.ClearRobotState(0);

                // 4) 清理机器人侧"目标/路径/命令"，并硬停
                var r0 = _world.Robots[0];

                r0.Speed = 0.0;
                r0.Manager.Acc = 0.0;
                r0.Move.StopImmediately_NoLock();
                r0.Manager.ResetAutoCommands();

                r0.AutoNavigator.SetClaimedPathPrefix(null);
                r0.AutoNavigator.ClearGoal();

                // 清理到达停顿计时器
                r0.ResetArrivalPause();

                // 5) 重绑 walkable，确保后续 StartAll Enable/RebuildPath 能用最新判定
                _walkableBinder.RebindDynamicWalkable_NoLock();

                // 6) 维持"自动已启用但无目标"的等待态
                if (!r0.AutoNavigator.IsEnabled)
                    r0.AutoNavigator.Enable();
            }
        }

        #endregion

        #region 快照获取（供 UI 渲染）

        /// <summary> 获取所有机器人状态快照（供渲染线程读取）。 </summary>
        public List<RobotStateSnapshot> GetRobotStatesSnapshot()
        {
            lock (_robotLock)
            {
                var list = new List<RobotStateSnapshot>(_world.Robots.Count);
                foreach (var r in _world.Robots)
                    list.Add(r.GetSnapshot());
                return list;
            }
        }

        /// <summary> 获取选中机器人状态快照（未选中时返回 default）。 </summary>
        public RobotStateSnapshot GetStateSnapshot()
        {
            lock (_robotLock)
            {
                int id = _world.SelectedRobotId;
                if (id < 0 || id >= _world.Robots.Count)
                    return default;
                return _world.Robots[id].GetSnapshot();
            }
        }

        /// <summary> 获取选中机器人的路径点（世界坐标）快照，用于 UI 绘制。 </summary>
        public List<(double X, double Y)> GetPathWorldPointsSnapshot()
        {
            lock (_robotLock)
            {
                int id = _world.SelectedRobotId;
                if (id < 0 || id >= _world.Robots.Count)
                    return null;

                return _world.Robots[id].AutoNavigator.GetPathWorldPointsSnapshot();
            }
        }

        /// <summary> 获取所有机器人的抢占格子快照（用于显示"格子锁"效果）。 </summary>
        public Dictionary<int, List<GridPos>> GetClaimedCellsSnapshot()
        {
            lock (_robotLock)
            {
                return _pathClaimManager.ClaimBoard.GetClaimedCellsSnapshot();
            }
        }

        /// <summary> 获取所有机器人目标点世界坐标快照（用于渲染）。 </summary>
        public List<(int Id, double X, double Y)> GetRobotsGoalWorldSnapshot()
        {
            lock (_robotLock)
            {
                var list = new List<(int Id, double X, double Y)>(_world.Robots.Count);

                foreach (var r in _world.Robots)
                {
                    var g = r.AutoNavigator.GetGoalWorldSnapshot();
                    if (g.HasValue)
                        list.Add((r.Id, g.Value.X, g.Value.Y));
                }

                return list;
            }
        }

        #endregion

        #region 控制参数 / 模式切换 / 手动输入

        /// <summary>
        /// 设置前进加速度（同步到所有机器人）。
        /// 自动模式下需要 ResetAutoCommands，使下一帧派发的 MoveDistance 读取到新加速度。
        /// </summary>
        public void SetForwardAcc(double acc) => _control.SetForwardAcc(acc);

        /// <summary> 设置最大速度（同步到所有机器人）。 </summary>
        public void SetMaxSpeed(double vmax) => _control.SetMaxSpeed(vmax);

        /// <summary>
        /// 切到自动模式（当前只对选中机器人做切换；未选中机器人由 Tick 保持自动）。
        /// </summary>
        public void EnableAuto()
        {
            _control.EnableAutoForSelected();
            lock (_robotLock)
            {
                ChangeProcessState_NoLock(EnumRobotProcessState.AutoNavigating);
            }
        }

        /// <summary>
        /// 切到手动模式：
        /// - 禁用选中机器人的自动导航；
        /// - 释放其所有格子锁和路径前缀；
        /// - 立即停车并清命令；
        /// - 启用手动输入并解除单机暂停；
        /// - 引擎模式切换为 ManualControl。
        /// </summary>
        public void EnableManual()
        {
            lock (_robotLock)
            {
                _isRunning = true;
                ChangeProcessState_NoLock(EnumRobotProcessState.ManualControl);
                _control.EnableManualForSelected(_pathClaimManager);
            }
        }

        /// <summary>
        /// 将指定 Id 的机器人切换到自动模式。
        /// </summary>
        public void EnableAutoForRobot(int robotId)
        {
            _control.EnableAutoForRobot(robotId);
        }

        /// <summary>
        /// 将指定 Id 的机器人切换到手动模式。
        /// </summary>
        public void EnableManualForRobot(int robotId)
        {
            lock (_robotLock)
            {
                _control.EnableManualForRobot(robotId, _pathClaimManager);
            }
        }

        /// <summary>
        /// 查询指定 Id 的机器人是否为自动模式。
        /// </summary>
        public bool IsRobotAutoMode(int robotId)
        {
            lock (_robotLock)
            {
                if (robotId < 0 || robotId >= _world.Robots.Count)
                    return false;
                return _world.Robots[robotId].AutoNavigator.IsEnabled;
            }
        }

        /// <summary> 触发选中机器人重建路径（供 UI 手动点击）。 </summary>
        public void RebuildPath()
        {
            lock (_robotLock)
            {
                int id = _world.SelectedRobotId;
                if (id < 0 || id >= _world.Robots.Count)
                    return;

                _world.Robots[id].AutoNavigator.RebuildPath();
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
                int id = _world.SelectedRobotId;
                if (id < 0 || id >= _world.Robots.Count)
                    return false;

                RobotInstance r = _world.Robots[id];

                _pathClaimManager.ClaimBoard.ReleaseAllByRobot(r.Id);
                _pathClaimManager.ClearRobotState(r.Id);

                // 手动设置目标时清除到达停顿，立即出发
                r.ResetArrivalPause();

                r.AutoNavigator.SetGoal(goal, rebuildIfEnabled: true);
                _control.ResumeRobot(r.Id);

                return true;
            }
        }

        /// <summary> 手动：前进键按下/抬起（W）。 </summary>
        public void ManualForwardKey(bool down) => _control.ManualForwardKey(down);

        /// <summary> 手动：左转键按下/抬起（A）。 </summary>
        public void ManualTurnLeftKey(bool down) => _control.ManualTurnLeftKey(down);

        /// <summary> 手动：右转键按下/抬起（D）。 </summary>
        public void ManualTurnRightKey(bool down) => _control.ManualTurnRightKey(down);

        #endregion

        #region 主循环 Tick

        /// <summary>
        /// 仿真主循环入口（由后台线程每 dt 调用一次）。
        /// 一帧 Tick 主要流程：
        /// 1) 冷却计数衰减（让步冷却 + 死锁关系淡出）；
        /// 2) 重绑动态 walkable（把其它机器人占用格/目标格当作动态障碍/保留格）；
        /// 3) 执行路径格子锁抢占，并把"抢占前缀"写回 AutoNavigator；
        /// 4) 逐机器人：
        ///    - 若单机暂停：停车；
        ///    - 否则自动/手动生成指令并执行运动学更新；
        ///    - 边走边释放：检测换格后释放上一格锁。
        /// </summary>
        public void Tick()
        {
            if (!_isRunning)
                return;

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
                // 1) 冷却/死锁淡出推进
                _pathClaimManager.TickCooling_NoLock();

                // 2) 动态可通行性重绑（把其它机器人占用格/终点当动态障碍）
                _walkableBinder.RebindDynamicWalkable_NoLock();

                // 3) 格子锁抢占 + 死锁检测与处理
                _pathClaimManager.ApplyPathClaiming_NoLock();

                // 4) 逐机器人更新控制与运动
                foreach (var r in _world.Robots)
                {
                    bool isSelected = (r.Id == _world.SelectedRobotId);

                    // 单机暂停：该机器人不派发指令，强制停车
                    if (_control.IsPaused(r.Id))
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
                            r.AutoNavigator.Enable();
                    }

                    // 自动巡航：到达目标格中心后停顿 2 秒再分配新随机目标
                    if (r.AutoNavigator.IsEnabled)
                    {
                        // 正在停顿倒计时中：递减计时器，保持停车
                        if (r.ArrivalPauseRemainS >= 0.0)
                        {
                            r.ArrivalPauseRemainS -= _dt;

                            if (r.ArrivalPauseRemainS <= 0.0)
                            {
                                // 停顿结束：释放旧锁，分配新目标
                                r.ResetArrivalPause();

                                _pathClaimManager.ClaimBoard.ReleaseAllByRobot(r.Id);
                                _pathClaimManager.ClearRobotState(r.Id);

                                r.AutoNavigator.SetGoal(
                                    _world.PickRandomFreeCell_NoLock(used: null),
                                    rebuildIfEnabled: true);
                                r.Manager.ResetAutoCommands();
                            }
                            else
                            {
                                // 仍在停顿中：保持停车，跳过后续运动
                                r.Speed = 0.0;
                                r.Manager.Acc = 0.0;
                                r.Move.StopImmediately_NoLock();
                                r.Move.Update();
                                continue;
                            }
                        }
                        else
                        {
                            // 非停顿状态：检测是否精确到达目标格中心
                            GridPos? goal = r.AutoNavigator.GetGoalGridSnapshot();

                            bool reachedGoal = false;

                            if (goal.HasValue)
                            {
                                double goalCenterX = r.GridToCenterX(goal.Value.X);
                                double goalCenterY = r.GridToCenterY(goal.Value.Y);

                                bool arriveX = Math.Abs(r.X - goalCenterX) <= ArriveEpsilonM;
                                bool arriveY = Math.Abs(r.Y - goalCenterY) <= ArriveEpsilonM;

                                reachedGoal = arriveX && arriveY;
                            }
                            else
                            {
                                // 没有目标也视为需要分配新目标
                                reachedGoal = true;
                            }

                            if (reachedGoal)
                            {
                                // 到达目标格中心：启动停顿倒计时，立即停车
                                r.ArrivalPauseRemainS = ArrivalPauseDurationS;
                                r.Speed = 0.0;
                                r.Manager.Acc = 0.0;
                                r.Move.StopImmediately_NoLock();
                                r.Move.Update();
                                continue;
                            }
                        }
                    }

                    // RobotManager 统一生命周期入口
                    r.Manager.Tick(_world.Dt);

                    // 自动模式：沿"抢占路径前缀"生成指令；生成失败则适当停车等待下一帧抢占
                    if (r.AutoNavigator.IsEnabled)
                    {
                        RobotCommand cmd = r.AutoNavigator.TryBuildNextCommandFromClaimedPath();
                        if (cmd != null)
                        {
                            r.Manager.DispatchDirect_NoLock(cmd);
                        }
                        else
                        {
                            // 没有新指令时，不中断正在进行的 MoveDistance/转向
                            if (!r.Manager.IsTurning &&
                                !r.Move.IsMoveDistanceActive_NoLock())
                            {
                                r.Move.StopImmediately_NoLock();
                            }
                        }
                    }
                    // 手动模式：仅对选中机器人派发手动命令
                    else if (r.Manual.IsEnabled && isSelected)
                    {
                        RobotCommand cmd = r.Manual.TryBuildNextCommand();
                        r.Manager.DispatchDirect_NoLock(cmd);
                    }

                    // 运动学执行：推进位置/速度/朝向
                    r.Move.Update();

                    // 边走边释放：换格后释放上一个格子的锁
                    _pathClaimManager.ReleaseClaimByMovement_NoLock(r);
                }
            }
        }

        #endregion

        #region 内部小工具

        /// <summary>
        /// 修改引擎状态（要求调用方已持有锁）。
        /// </summary>
        private void ChangeProcessState_NoLock(EnumRobotProcessState newState)
        {
            _processState = newState;
        }

        #endregion
    }

    /// <summary>
    /// 机器人状态快照（值类型）：用于跨线程安全读取（UI 线程绘制）。
    /// 注意：这里不包含 Id，调用方通常按列表索引对应机器人 Id。
    /// </summary>
    internal readonly struct RobotStateSnapshot
    {
        public RobotStateSnapshot(double X, double Y, double Speed, double Acc, double OrientationAngle, bool IsAutoMode)
        {
            this.X = X;
            this.Y = Y;
            this.Speed = Speed;
            this.Acc = Acc;
            this.OrientationAngle = OrientationAngle;
            this.IsAutoMode = IsAutoMode;
        }

        public double X { get; }
        public double Y { get; }
        public double Speed { get; }
        public double Acc { get; }
        public double OrientationAngle { get; }

        /// <summary> 该机器人是否处于自动模式（true=自动，false=手动）。 </summary>
        public bool IsAutoMode { get; }
    }
}