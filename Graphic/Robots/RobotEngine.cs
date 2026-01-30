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
    /// 多机器人仿真/调度引擎：管理机器人、障碍、模式切换、Tick 更新
    /// </summary>
    internal sealed class RobotEngine
    {
        private EnumRobotProcessState _processState = EnumRobotProcessState.Idle; // 当前引擎状态（默认 Idle）
        private readonly object _robotLock = new object(); // 机器人数据互斥锁（保证跨线程读写一致）

        private readonly ObstacleMap _obstacleMap; // 障碍物地图（静态障碍）

        private readonly double _cellSizeM; // 单个网格边长（米）
        private readonly double _dt; // 仿真时间步长（秒）
        private readonly int _gridCount; // 网格数量（宽高相同）

        private readonly double _worldWidthM; // 世界宽度（米）= gridCount * cellSizeM
        private readonly double _worldHeightM; // 世界高度（米）= gridCount * cellSizeM

        private readonly Random _rng = new Random(); // 随机数发生器（用于随机目标/随机初始位置）

        private readonly List<RobotInstance> _robots = new List<RobotInstance>(); // 机器人实例集合（0..N-1）
        private int _selectedRobotId = -1; // 当前选中机器人 Id（用于 UI/手动控制）

        // 单机器人重置后的“随机运动”
        private bool _singleRandomRoamEnabled; // 是否启用“单机器人随机巡航”模式

        // --- 碰撞让步冷却：避免每帧 Stop + Rebuild 导致抖动 ---
        private const int YieldCooldownFrames = 12; // 冷却帧数：12 帧 * 20ms ≈ 240ms（防止频繁让步抖动）
        private readonly Dictionary<int, int> _yieldCooldownTicks = new Dictionary<int, int>(); // robotId -> 冷却剩余帧数

        private readonly GridCellClaimBoard _claimBoard;

        public RobotEngine(int gridCount, double cellSizeM, double dt, double initialMaxSpeed, EnumMoveDirection initialDirection)
        {
            _gridCount = gridCount;
            _cellSizeM = cellSizeM;
            _dt = dt;
            _worldWidthM = gridCount * cellSizeM;
            _worldHeightM = gridCount * cellSizeM;

            _obstacleMap = new ObstacleMap(gridCount, gridCount);

            _claimBoard = new GridCellClaimBoard(_gridCount, _obstacleMap);

            SetRobotCount(1, initialMaxSpeed, initialDirection);

            _processState = EnumRobotProcessState.AutoNavigating;
        }

        public double WorldWidthM => _worldWidthM; // 对外暴露世界宽度（米）
        public double WorldHeightM => _worldHeightM; // 对外暴露世界高度（米）
        public double CellSizeM => _cellSizeM; // 对外暴露网格尺寸（米）
        public int GridCount => _gridCount; // 对外暴露网格数量

        /// <summary>
        /// 当前选中机器人 Id（线程安全）
        /// </summary>
        public int SelectedRobotId
        {
            get
            {
                lock (_robotLock) // 加锁读取选中 Id
                {
                    return _selectedRobotId; // 返回选中机器人 Id
                }
            }
        }

        /// <summary>
        /// 是否存在选中机器人
        /// </summary>
        public bool HasSelectedRobot
        {
            get
            {
                lock (_robotLock)
                {
                    return _selectedRobotId >= 0 && _selectedRobotId < _robots.Count;
                }
            }
        }

        /// <summary>
        /// 清空选中机器人（变为未选中）
        /// </summary>
        public void ClearSelectedRobot()
        {
            lock (_robotLock)
            {
                _selectedRobotId = -1;
            }
        }

        /// <summary>
        /// 当前（以及将要同步到所有机器人）的寻路算法
        /// </summary>
        public EnumPathfindingAlgorithm Algorithm
        {
            get
            {
                lock (_robotLock) // 加锁读取选中机器人的算法
                {
                    return GetSelectedRobot_NoLock().AutoNavigator.Algorithm; // 从选中机器人自动导航器读取算法
                }
            }
            set
            {
                lock (_robotLock) // 加锁同步设置所有机器人算法
                {
                    for (int i = 0; i < _robots.Count; i++) // 遍历所有机器人
                    {
                        _robots[i].AutoNavigator.Algorithm = value; // 设置自动导航器算法
                    }
                }
            }
        }

        /// <summary>
        /// 选中机器人是否启用自动导航
        /// </summary>
        public bool AutoEnabled
        {
            get
            {
                lock (_robotLock) // 加锁读取状态
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
        /// 获取障碍物地图快照（供渲染/显示）
        /// </summary>
        public bool[,] GetObstacleSnapshot()
        {
            return _obstacleMap.GetSnapshot(); // 直接返回障碍物快照（由 ObstacleMap 负责复制/隔离）
        }

        /// <summary>
        /// 设置障碍物
        /// 切换某网格的障碍状态（空地<->障碍）
        /// </summary>
        public void ToggleObstacle(GridPos p)
        {
            _obstacleMap.Toggle(p); // 切换障碍状态

            if (_processState == EnumRobotProcessState.ObstacleEditing)
            { // 障碍编辑模式下不触发重规划（避免不断打断编辑）
                return; // 直接返回
            }

            lock (_robotLock) // 加锁，避免机器人导航状态并发修改
            {
                for (int i = 0; i < _robots.Count; i++) // 遍历所有机器人
                {
                    if (_robots[i].AutoNavigator.IsEnabled) // 仅对启用自动导航的机器人重建路径
                    {
                        _robots[i].AutoNavigator.RebuildPath(); // 触发寻路重算
                    }
                }
            }
        }

        /// <summary>
        /// 清空障碍物
        /// </summary>
        public void ClearObstacles()
        {
            _obstacleMap.Clear(); // 清空障碍物地图

            if (_processState == EnumRobotProcessState.ObstacleEditing) // 障碍编辑模式下不触发重规划
            {
                return; // 直接返回
            }

            lock (_robotLock) // 加锁更新导航状态
            {
                for (int i = 0; i < _robots.Count; i++) // 遍历所有机器人
                {
                    if (_robots[i].AutoNavigator.IsEnabled) // 仅对自动模式机器人重建路径
                    {
                        _robots[i].AutoNavigator.RebuildPath(); // 重建路径
                    }
                }
            }
        }

        /// <summary>
        /// 调整机器人数量，保持已有机器人位置不变
        /// </summary>
        /// <param name="count">机器人数量</param>
        /// <param name="initialMaxSpeed">最大速度</param>
        /// <param name="initialDirection">初始朝向</param>
        public void SetRobotCount(int count, double initialMaxSpeed, EnumMoveDirection initialDirection)
        {
            if (count < 1) // 最少保持 1 个机器人
            {
                count = 1; // 修正到 1
            }

            lock (_robotLock) // 加锁：调整机器人集合是写操作
            {
                _singleRandomRoamEnabled = false; // 调整数量意味着退出“单机器人随机巡航”

                // 1) 多 -> 少：只删尾部（保持前面机器人位置不变）
                while (_robots.Count > count) // 若当前数量大于目标数量
                {
                    _robots.RemoveAt(_robots.Count - 1); // 删除末尾机器人（Id 最大者）
                }

                // 修正选中项
                if (_selectedRobotId >= _robots.Count) // 如果选中 Id 超出范围
                {
                    _selectedRobotId = Math.Max(0, _robots.Count - 1); // 选中最后一个（或 0）
                }

                // 2) 少 -> 多：只新增，不动已有机器人
                if (_robots.Count < count) // 若当前数量小于目标数量
                {
                    var used = BuildUsedCellKeySet_NoLock(); // 构建已占用格集合（避免新机器人生成重叠）

                    while (_robots.Count < count) // 循环新增直到达到目标数量
                    {
                        int id = _robots.Count; // 新机器人的 Id = 当前数量（保证连续）

                        GridPos cell = PickRandomFreeCell_NoLock(used); // 随机挑选一个未占用且非障碍的格子
                        int key = cell.Y * _gridCount + cell.X; // 将 (x,y) 映射为一维 key
                        used.Add(key); // 记录该格已被占用（为下一次新增做排除）

                        double x = cell.X * _cellSizeM + _cellSizeM / 2.0; // 将格子中心转换为世界坐标 X
                        double y = cell.Y * _cellSizeM + _cellSizeM / 2.0; // 将格子中心转换为世界坐标 Y

                        var r = new RobotInstance( // 创建机器人实例
                            id: id,
                            robotLock: _robotLock, // 注入同一把锁，供 RobotInstance 内部共享
                            obstacleMap: _obstacleMap, // 注入障碍物地图
                            gridCount: _gridCount, // 注入网格数量
                            cellSizeM: _cellSizeM, // 注入网格尺寸
                            dt: _dt, // 注入时间步长
                            worldWidthM: _worldWidthM, // 注入世界宽度
                            worldHeightM: _worldHeightM, // 注入世界高度
                            initialMaxSpeed: initialMaxSpeed, // 初始最大速度
                            initialDirection: initialDirection, // 初始方向
                            initialX: x, // 初始位置 X（世界坐标）
                            initialY: y, // 初始位置 Y（世界坐标）
                            getGoalOwnerMap: () => BuildGoalOwnerMap_NoLock()); // 新增：获取“终点拥有者”映射

                        // 新机器人：启用自动并给一个随机目标，否则不会动
                        r.AutoNavigator.Enable(); // 启用自动导航
                        r.AutoNavigator.ClearGoal(); // 清空目标（先重置状态）
                        r.Manager.ResetAutoCommands(); // 清空自动命令队列，避免遗留
                        r.AutoNavigator.SetGoal(PickRandomFreeCell_NoLock(used: null), rebuildIfEnabled: true); // 设置随机目标并重建路径

                        // 继承当前“全局加速度”——取第一个机器人的值作为当前配置
                        if (_robots.Count > 0) // 若已有机器人存在
                        {
                            r.Acc = _robots[0].Acc; // 继承第一个机器人的加速度配置
                        }

                        _robots.Add(r); // 将新机器人加入集合
                    }
                }

                // 3) 重新绑定动态障碍（包含“占用格”）
                RebindDynamicWalkable_NoLock(); // 将所有机器人占用格注入“可行走判断”以实现动态避让

                // 4) 确保全部机器人处于“自动巡航可运行”状态（你要求未选中继续自动）
                for (int i = 0; i < _robots.Count; i++) // 遍历所有机器人
                {
                    if (!_robots[i].AutoNavigator.IsEnabled) // 若自动导航未启用
                    {
                        _robots[i].AutoNavigator.Enable(); // 强制启用，保证持续运行
                    }
                }
            }
        }

        /// <summary>
        /// 构建当前所有机器人占用格的 key 集合（要求调用方已持有锁）
        /// </summary>
        /// <returns></returns>
        private HashSet<int> BuildUsedCellKeySet_NoLock()
        {
            var used = new HashSet<int>(_robots.Count); // 初始化 HashSet 并预估容量

            for (int i = 0; i < _robots.Count; i++) // 遍历所有机器人
            {
                GridPos c = _robots[i].GetGridPos_NoLock(); // 获取机器人所在格（无锁版本）
                used.Add(c.Y * _gridCount + c.X); // 记录占用格 key
            }

            return used; // 返回占用集合
        }

        /// <summary>
        /// 重置成单个机器人并启用随机巡航模式
        /// </summary>
        /// <param name="initialMaxSpeed"></param>
        /// <param name="initialDirection"></param>
        public void ResetToSingleRobotRandomRoam(double initialMaxSpeed, EnumMoveDirection initialDirection)
        {
            lock (_robotLock) // 加锁：将重置机器人数量/状态
            {
                SetRobotCount(1, initialMaxSpeed, initialDirection); // 调整为单机器人
                _singleRandomRoamEnabled = true; // 启用单机器人随机巡航标记

                // 给一个随机目标，启动随机巡航
                RobotInstance r0 = _robots[0]; // 取唯一机器人
                r0.AutoNavigator.ClearGoal(); // 清理旧目标
                r0.Manager.ResetAutoCommands(); // 清空命令
                r0.AutoNavigator.SetGoal(PickRandomFreeCell_NoLock(new HashSet<int>()), rebuildIfEnabled: true); // 设置随机目标并重建路径
            }
        }

        /// <summary>
        /// 选中指定机器人
        /// </summary>
        /// <param name="id"></param>
        /// <returns></returns>
        public bool SelectRobot(int id)
        {
            lock (_robotLock) // 加锁：写选中 Id
            {
                if (id < 0 || id >= _robots.Count) // 越界校验
                {
                    return false; // 无效 Id
                }

                _selectedRobotId = id; // 更新选中机器人 Id
                return true; // 选中成功
            }
        }

        /// <summary>
        /// 获取所有机器人状态快照（用于渲染/显示）
        /// </summary>
        /// <returns></returns>
        public List<RobotStateSnapshot> GetRobotStatesSnapshot()
        {
            lock (_robotLock) // 加锁读取机器人状态
            {
                var list = new List<RobotStateSnapshot>(_robots.Count); // 初始化快照列表
                for (int i = 0; i < _robots.Count; i++) // 遍历所有机器人
                {
                    list.Add(_robots[i].GetSnapshot()); // 采集每个机器人的快照
                }
                return list; // 返回快照列表
            }
        }

        /// <summary>
        /// 获取“选中机器人”的状态快照
        /// </summary>
        /// <returns></returns>
        public RobotStateSnapshot GetStateSnapshot()
        {
            lock (_robotLock) // 加锁保证一致性
            {
                // 未选中：返回默认值，避免 UI 崩溃
                if (_selectedRobotId < 0 || _selectedRobotId >= _robots.Count)
                {
                    return default(RobotStateSnapshot);
                }

                return _robots[_selectedRobotId].GetSnapshot();
            }
        }

        /// <summary>
        /// 获取选中机器人的路径点（世界坐标）快照
        /// </summary>
        /// <returns></returns>
        public List<(double X, double Y)> GetPathWorldPointsSnapshot()
        {
            lock (_robotLock) // 加锁读取路径
            {
                // 未选中：不画路径（返回 null/空都行；DrawPath 一般对 null 更友好）
                if (_selectedRobotId < 0 || _selectedRobotId >= _robots.Count)
                {
                    return null;
                }

                return _robots[_selectedRobotId].AutoNavigator.GetPathWorldPointsSnapshot();
            }
        }

        /// <summary>
        /// 获取抢占格子快照（robotId -> claimed grid cells）。
        /// 用于渲染层显示每台机器人的抢占路径。
        /// </summary>
        public Dictionary<int, List<GridPos>> GetClaimedCellsSnapshot()
        {
            lock (_robotLock)
            {
                return _claimBoard.GetClaimedCellsSnapshot();
            }
        }

        /// <summary>
        /// 获取所有机器人的目标点（世界坐标）快照（用于渲染）。
        /// - key: robotId
        /// - value: (X,Y) 世界坐标（格子中心）
        /// - 没有目标的机器人不会出现在列表里
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
        /// 设置“前向加速度”配置（同步到全部机器人）
        /// </summary>
        /// <param name="acc"></param>
        public void SetForwardAcc(double acc)
        {
            lock (_robotLock) // 加锁写入所有机器人
            {
                // 对所有机器人同步（否则只有选中机器人 Acc != 0）
                for (int i = 0; i < _robots.Count; i++) // 遍历所有机器人
                {
                    _robots[i].Acc = acc; // 更新机器人加速度

                    // 自动模式下需要刷新命令，让下一帧 MoveDistance 读取到新的 forwardAcc
                    _robots[i].Manager.ResetAutoCommands(); // 重置自动命令队列
                }
            }
        }

        /// <summary>
        /// 设置最大速度（同步到全部机器人）
        /// </summary>
        /// <param name="vmax"></param>
        public void SetMaxSpeed(double vmax)
        {
            lock (_robotLock) // 加锁更新
            {
                for (int i = 0; i < _robots.Count; i++) // 遍历所有机器人
                {
                    _robots[i].Manager.MaxSpeed = vmax; // 设置机器人管理器的最大速度
                }
            }
        }

        /// <summary>
        /// 切换为自动导航模式（仅选中机器人对齐方向并重置命令）
        /// </summary>
        public void EnableAuto()
        {
            lock (_robotLock) // 加锁切换模式
            {
                ChangeProcessState_NoLock(EnumRobotProcessState.AutoNavigating); // 设置引擎状态为自动巡航

                // 未选中：只切引擎状态，不对某一台机器人做“对齐/清命令”
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
        /// 切换为手动控制模式（仅选中机器人启用手动）
        /// </summary>
        public void EnableManual()
        {
            lock (_robotLock) // 加锁切换模式
            {
                ChangeProcessState_NoLock(EnumRobotProcessState.ManualControl); // 设置引擎状态为手动控制

                // 未选中：不启用任何机器人的手动
                if (_selectedRobotId < 0 || _selectedRobotId >= _robots.Count)
                {
                    return;
                }

                RobotInstance r = _robots[_selectedRobotId];
                r.Manual.Enable();
            }
        }

        /// <summary>
        /// 触发选中机器人重建路径
        /// </summary>
        public void RebuildPath()
        {
            lock (_robotLock) // 加锁操作导航器
            {
                if (_selectedRobotId < 0 || _selectedRobotId >= _robots.Count)
                {
                    return;
                }

                _robots[_selectedRobotId].AutoNavigator.RebuildPath();
            }
        }

        /// <summary>
        /// 手动：前进键按下/抬起
        /// </summary>
        public void ManualForwardKey(bool down)
        {
            lock (_robotLock) // 加锁更新输入状态
            {
                if (_selectedRobotId < 0 || _selectedRobotId >= _robots.Count)
                {
                    return;
                }

                RobotInstance r = _robots[_selectedRobotId];
                r.Manual.InputForwardKey(down); // 写入手动输入：前进键状态
                r.Manager.ResetManualCommands(); // 刷新手动命令队列
            }
        }

        /// <summary>
        /// 手动：左转键按下/抬起
        /// </summary>
        public void ManualTurnLeftKey(bool down)
        {
            lock (_robotLock) // 加锁更新输入状态
            {
                if (_selectedRobotId < 0 || _selectedRobotId >= _robots.Count)
                {
                    return;
                }

                RobotInstance r = _robots[_selectedRobotId];
                r.Manual.InputTurnLeftKey(down); // 写入手动输入：左转键状态
                r.Manager.ResetManualCommands(); // 刷新手动命令队列
            }
        }

        /// <summary>
        /// 手动：右转键按下/抬起
        /// </summary>
        public void ManualTurnRightKey(bool down)
        {
            lock (_robotLock) // 加锁更新输入状态
            {
                if (_selectedRobotId < 0 || _selectedRobotId >= _robots.Count)
                {
                    return;
                }

                RobotInstance r = _robots[_selectedRobotId];
                r.Manual.InputTurnRightKey(down); // 写入手动输入：右转键状态
                r.Manager.ResetManualCommands(); // 刷新手动命令队列
            }
        }

        /// <summary>
        /// 为选中机器人设置目标格（自动导航）
        /// </summary>
        public bool TrySetSelectedRobotGoal(GridPos goal)
        {
            lock (_robotLock) // 加锁设置目标
            {
                if (_selectedRobotId < 0 || _selectedRobotId >= _robots.Count)
                {
                    return false;
                }

                RobotInstance r = _robots[_selectedRobotId]; // 取选中机器人
                r.AutoNavigator.SetGoal(goal, rebuildIfEnabled: true); // 设置目标并在启用自动时重建路径
                return true; // 当前实现总是成功
            }
        }

        /// <summary>
        /// 切换障碍编辑模式（暂停/恢复机器人运动）
        /// </summary>
        /// <param name="enabled"></param>
        public void SetObstacleEditMode(bool enabled)
        {
            lock (_robotLock) // 加锁切换模式与批量更新机器人状态
            {
                if (enabled) // 开启障碍编辑模式
                {
                    ChangeProcessState_NoLock(EnumRobotProcessState.ObstacleEditing); // 设置引擎状态为障碍编辑

                    for (int i = 0; i < _robots.Count; i++) // 遍历所有机器人并强制停止
                    {
                        _robots[i].Speed = 0.0; // 将速度置零（状态层面）
                        _robots[i].Manager.Acc = 0.0; // 将管理器加速度置零（避免继续加速）
                        _robots[i].Move.StopImmediately_NoLock(); // 立即停止运动（运动学层面）
                        if (_robots[i].AutoNavigator.IsEnabled) // 若自动导航启用
                        {
                            _robots[i].AutoNavigator.Disable(); // 禁用自动导航，避免编辑期间重规划/移动
                        }
                        _robots[i].Manual.Disable(); // 禁用手动控制
                        _robots[i].Manager.ResetAutoCommands(); // 清空自动命令队列
                    }
                }
                else // 关闭障碍编辑模式，恢复自动巡航
                {
                    ChangeProcessState_NoLock(EnumRobotProcessState.AutoNavigating); // 设置引擎状态为自动巡航

                    for (int i = 0; i < _robots.Count; i++) // 遍历所有机器人
                    {
                        _robots[i].AutoNavigator.Enable(); // 启用自动导航
                        _robots[i].Manager.ResetAutoCommands(); // 重置自动命令队列（确保恢复后命令一致）
                    }
                }
            }
        }

        /// <summary>
        /// 引擎主循环：每帧更新（调度导航、处理碰撞、推进运动学）
        /// </summary>
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
                // 0) 冷却计数递减（保留）
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

                // 1) 动态障碍（保留）
                RebindDynamicWalkable_NoLock();

                // 2) 先确保路径是“当前固定结果”，再按距离优先抢占整段路径
                ApplyPathClaiming_NoLock();

                // 4) 每帧：允许则下发下一条自动指令；若下一格被抢占 -> 原地等待
                for (int i = 0; i < _robots.Count; i++)
                {
                    RobotInstance r = _robots[i];
                    bool isSelected = r.Id == _selectedRobotId;

                    if (!isSelected)
                    {
                        r.Manual.Disable();
                        if (!r.AutoNavigator.IsEnabled)
                        {
                            r.AutoNavigator.Enable();
                        }
                    }

                    if (r.AutoNavigator.IsEnabled)
                    {
                        if (r.AutoNavigator.GetPathWorldPointsSnapshot() == null
                            || r.AutoNavigator.GetPathWorldPointsSnapshot().Count == 0)
                        {
                            r.AutoNavigator.SetGoal(PickRandomFreeCell_NoLock(used: null), rebuildIfEnabled: true);
                            r.Manager.ResetAutoCommands();
                        }
                    }

                    // Auto 的 Manager.Tick 现在不拉 provider（保留调用避免影响手动/其它状态）
                    r.Manager.Tick(_dt, () => r.Acc);

                    if (r.AutoNavigator.IsEnabled)
                    {
                        if (TryDispatchAutoCommandWithClaim_NoLock(r))
                        {
                            // 已下发
                        }
                        else
                        {
                            // 被抢占阻塞：原地等待（保证彻底停）
                            r.Move.StopImmediately_NoLock();
                        }
                    }

                    r.Move.Update();
                }

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

        private void ApplyPathClaiming_NoLock()
        {
            _claimBoard.Reset();

            var items = new List<(RobotInstance R, int Dist)>(_robots.Count);

            for (int i = 0; i < _robots.Count; i++)
            {
                RobotInstance r = _robots[i];

                GridPos? goal = r.AutoNavigator.GetGoalGridSnapshot();
                if (!goal.HasValue)
                {
                    continue;
                }

                GridPos cur = r.GetGridPos_NoLock();
                int dist = Math.Abs(cur.X - goal.Value.X) + Math.Abs(cur.Y - goal.Value.Y);

                items.Add((r, dist));
            }

            items.Sort((a, b) =>
            {
                int c = a.Dist.CompareTo(b.Dist);
                if (c != 0) return c;
                return a.R.Id.CompareTo(b.R.Id);
            });

            // 注意：不要每帧 RebuildPath，否则会不断重建 _alignQueue 导致抖动
            for (int i = 0; i < items.Count; i++)
            {
                RobotInstance r = items[i].R;

                List<GridPos> path = r.AutoNavigator.GetPathGridSnapshot();

                // 仅当路径为空时才尝试重建一次（兼容刚切目标/刚启用的场景）
                if ((path == null || path.Count == 0) && r.AutoNavigator.IsEnabled)
                {
                    r.AutoNavigator.RebuildPath();
                    path = r.AutoNavigator.GetPathGridSnapshot();
                }

                _claimBoard.ClaimPath(r.Id, path);
            }
        }

        private bool TryDispatchAutoCommandWithClaim_NoLock(RobotInstance r)
        {
            GridPos cur;
            GridPos next;
            EnumMoveDirection desiredDir;

            // 先看下一格意图（用于等待判定）
            if (r.AutoNavigator.TryGetNextStepSnapshot(out cur, out next, out desiredDir))
            {
                // 下一格被其它机器人抢占：等待
                if (_claimBoard.IsClaimedByOther(r.Id, next))
                {
                    return false;
                }
            }

            // 允许走：直接从 AutoNavigator 生成“下一条命令”并直接下发（不走队列/provider）
            RobotCommand cmd = r.AutoNavigator.TryBuildNextCommand();
            if (cmd == null)
            {
                return false;
            }

            r.Manager.DispatchDirect_NoLock(cmd, () => r.Acc);
            return true;
        }

        /// <summary>
        /// 修改引擎状态（要求调用方已持有锁）
        /// </summary>
        private void ChangeProcessState_NoLock(EnumRobotProcessState newState)
        {
            _processState = newState; // 直接赋值状态
        }

        /// <summary>
        /// 获取选中机器人实例（要求调用方已持有锁）
        /// </summary>
        /// <returns></returns>
        private RobotInstance GetSelectedRobot_NoLock()
        {
            if (_selectedRobotId < 0) // 若选中 Id 小于 0
            {
                _selectedRobotId = 0; // 修正到 0
            }
            if (_selectedRobotId >= _robots.Count) // 若选中 Id 超出数量上限
            {
                _selectedRobotId = _robots.Count - 1; // 修正到最后一个
            }

            return _robots[_selectedRobotId]; // 返回选中机器人实例
        }

        /// <summary>
        /// 随机选择一个“非障碍且未被 used 占用”的格子（要求调用方已持有锁）
        /// </summary>
        /// <param name="used"></param>
        /// <returns></returns>
        private GridPos PickRandomFreeCell_NoLock(HashSet<int> used)
        {
            for (int tries = 0; tries < 5000; tries++) // 最多尝试 5000 次避免死循环
            {
                int x = _rng.Next(0, _gridCount); // 随机格 X
                int y = _rng.Next(0, _gridCount); // 随机格 Y
                int key = y * _gridCount + x; // 映射为一维 key

                if (used != null && used.Contains(key)) // 若 used 非空且该格已被占用
                {
                    continue; // 继续下一次尝试
                }

                var p = new GridPos(x, y); // 构造格坐标
                if (_obstacleMap.IsObstacle(p)) // 若该格是障碍
                {
                    continue; // 继续下一次尝试
                }

                return p; // 找到可用格则返回
            }

            return new GridPos(0, 0); // 尝试失败时回退到 (0,0)（可能是障碍/占用，调用方需容错）
        }

        /// <summary>
        /// 重新绑定“动态可行走判断”：将其它机器人占用格视为不可走（要求调用方已持有锁）
        /// </summary>
        private void RebindDynamicWalkable_NoLock()
        {
            // occupied：所有机器人当前占用格
            var occupied = new HashSet<int>(_robots.Count); // 占用格 key 集合（用于快速查找）
            var cellKeys = new int[_robots.Count]; // 缓存每个机器人当前格的 key（避免重复计算）

            for (int i = 0; i < _robots.Count; i++) // 先统计所有机器人占用格
            {
                GridPos c = _robots[i].GetGridPos_NoLock(); // 读取机器人当前格
                int key = c.Y * _gridCount + c.X; // 映射 key
                cellKeys[i] = key; // 记录到数组
                occupied.Add(key); // 加入占用集合
            }

            // 收集：goalKey -> ownerRobotId（注意：允许自己走进自己的 goal）
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
                        // 同一格多个目标时，保留第一个即可（都视为“有人占用的终点”）
                        if (!goalOwnerByKey.ContainsKey(k))
                        {
                            goalOwnerByKey.Add(k, _robots[i].Id);
                        }
                    }
                }
            }

            for (int i = 0; i < _robots.Count; i++) // 为每个机器人设置 WalkableProvider（闭包捕获 myKey/occupied）
            {
                RobotInstance me = _robots[i]; // 当前机器人（要设置 provider 的对象）
                int myKey = cellKeys[i]; // 当前机器人自己的占用格 key
                int myId = me.Id;

                me.AutoNavigator.SetIsWalkableProvider(p => // 设置“某格是否可走”的回调
                {
                    if (_obstacleMap.IsObstacle(p)) // 静态障碍优先判定
                    {
                        return false; // 障碍格不可走
                    }

                    int key = p.Y * _gridCount + p.X; // 将格坐标映射为 key

                    // 自己所在格允许，否则会把自己当障碍卡死
                    if (key == myKey) // 若查询的是自己当前格
                    {
                        return true; // 允许（否则寻路会认为起点不可走）
                    }

                    // 仅将“其它机器人”的终点视为障碍；自己的终点必须可走，否则永远到不了终点
                    int ownerId;
                    if (goalOwnerByKey.TryGetValue(key, out ownerId) && ownerId != myId)
                    {
                        return false;
                    }

                    return !occupied.Contains(key); // 其它机器人占用格不可走，否则可走
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
                    if (key == myKey)  // 自己当前位置始终可走
                        return true;

                    // 仅禁止进入“其它机器人”的终点格；允许进入自己的终点格
                    int ownerId;
                    if (goalOwnerByKey.TryGetValue(key, out ownerId) && ownerId != myId)
                        return false;

                    return !occupied.Contains(key);  // 其它机器人所在格视为不可走
                });
            }
        }

        /// <summary>
        /// 获取当前各机器人目标格的拥有者映射：cellKey -> ownerRobotId。
        /// 要求调用方已持有 _robotLock。
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
                        // 同一格多个目标时，保留第一个即可（都视为“有人占用的终点”）
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
    /// 获取机器人状态快照的结构体，用于跨线程/渲染层安全读取
    /// </summary>
    internal readonly struct RobotStateSnapshot
    {
        public RobotStateSnapshot(double X, double Y, double Speed, double Acc, double OrientationAngle) // 构造快照
        {
            this.X = X; // 保存位置 X
            this.Y = Y; // 保存位置 Y
            this.Speed = Speed; // 保存速度
            this.Acc = Acc; // 保存加速度
            this.OrientationAngle = OrientationAngle; // 保存朝向角（弧度/度取决于上层约定）
        }

        public double X { get; } // 位置 X（世界坐标）
        public double Y { get; } // 位置 Y（世界坐标）
        public double Speed { get; } // 速度
        public double Acc { get; } // 加速度
        public double OrientationAngle { get; } // 朝向角
    }
}