using Graphic.Maps;
using GridDemo.RobotModels.Pathfinding;
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

        public RobotEngine(int gridCount, double cellSizeM, double dt, double initialMaxSpeed, EnumMoveDirection initialDirection)
        {
            _gridCount = gridCount; // 保存网格数量
            _cellSizeM = cellSizeM; // 保存网格尺寸
            _dt = dt; // 保存时间步长
            _worldWidthM = gridCount * cellSizeM; // 计算世界宽度
            _worldHeightM = gridCount * cellSizeM; // 计算世界高度

            _obstacleMap = new ObstacleMap(gridCount, gridCount); // 创建障碍物地图（gridCount x gridCount）

            SetRobotCount(1, initialMaxSpeed, initialDirection); // 默认先创建 1 个机器人

            _processState = EnumRobotProcessState.AutoNavigating; // 默认进入自动巡航状态
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
                            id: id, // 设置机器人 Id
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
                            initialY: y); // 初始位置 Y（世界坐标）

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
            switch (_processState) // 根据引擎状态决定是否更新
            {
                case EnumRobotProcessState.ObstacleEditing: // 障碍编辑：不更新
                case EnumRobotProcessState.Idle: // 空闲：不更新
                case EnumRobotProcessState.Error: // 错误：不更新
                    return; // 直接返回
            }

            lock (_robotLock) // Tick 内部会读写大量共享状态，因此整段加锁
            {
                // 0) 冷却计数递减
                if (_yieldCooldownTicks.Count > 0) // 若存在冷却中的机器人
                {
                    var keys = new List<int>(_yieldCooldownTicks.Keys); // 拷贝 key 列表（避免遍历时修改字典）
                    for (int i = 0; i < keys.Count; i++) // 遍历所有处于冷却的 robotId
                    {
                        int id = keys[i]; // 当前 robotId
                        int t = _yieldCooldownTicks[id] - 1; // 冷却计数减 1
                        if (t <= 0) // 冷却结束
                        {
                            _yieldCooldownTicks.Remove(id); // 从字典移除
                        }
                        else // 仍在冷却
                        {
                            _yieldCooldownTicks[id] = t; // 写回剩余冷却帧数
                        }
                    }
                }

                // 1) 动态障碍：把其它机器人占用的格子注入 WalkableProvider
                RebindDynamicWalkable_NoLock(); // 更新每个机器人“可行走判断”（将其它机器人当作动态障碍）

                // 2) 碰撞让步：以网格为单位检测“前后左右/同格”
                HandleRobotCollisions_NoLock(); // 检测并处理机器人之间的潜在冲突（让步/刹停/重规划）

                // 3) 未选中机器人继续 Auto（不响应手动）
                for (int i = 0; i < _robots.Count; i++) // 遍历所有机器人逐个 Tick
                {
                    RobotInstance r = _robots[i]; // 当前机器人
                    bool isSelected = r.Id == _selectedRobotId; // 判断是否为选中机器人

                    if (!isSelected) // 未选中机器人
                    {
                        // 强制未选中机器人保持自动巡航
                        r.Manual.Disable(); // 禁止手动输入影响未选中机器人
                        if (!r.AutoNavigator.IsEnabled) // 若自动未开启
                        {
                            r.AutoNavigator.Enable(); // 强制开启自动导航
                        }
                    }

                    if (r.AutoNavigator.IsEnabled) // 自动导航启用时
                    {
                        // 没目标时给一个随机目标，确保“自动巡航”一定会走
                        // （避免 EnableAuto() 里 ClearGoal 后一直不动）
                        if (r.AutoNavigator.GetPathWorldPointsSnapshot() == null // 路径为空（未规划）
                            || r.AutoNavigator.GetPathWorldPointsSnapshot().Count == 0) // 或路径点数量为 0
                        {
                            // 注意：这里不使用 used，目标允许和别的机器人当前位置冲突由动态障碍避让+重规划解决
                            r.AutoNavigator.SetGoal(PickRandomFreeCell_NoLock(used: null), rebuildIfEnabled: true); // 设置随机目标并重建路径
                            r.Manager.ResetAutoCommands(); // 目标变化后重置命令队列
                        }
                    }

                    // 调度命令 + 运动学
                    r.Manager.Tick(_dt, () => r.Acc); // 让管理器按 dt 调度下一步命令（从委托获取当前加速度）
                    r.Move.Update(); // 运动学更新：推进位置/速度/朝向等
                }

                // 4) 单机器人随机巡航：无路径/到达后重置随机目标
                if (_singleRandomRoamEnabled && _robots.Count == 1) // 仅在单机器人随机巡航启用且确实只有 1 个机器人时执行
                {
                    RobotInstance r0 = _robots[0]; // 取第一个机器人
                    var p = r0.AutoNavigator.GetPathWorldPointsSnapshot(); // 获取路径点快照
                    if (p == null || p.Count < 2) // 若路径不足（无路径/到达终点）
                    {
                        r0.AutoNavigator.SetGoal(PickRandomFreeCell_NoLock(new HashSet<int>()), rebuildIfEnabled: true); // 重新设置随机目标并重建路径
                    }
                }
            }
        }

        // -------------------------- 交通规则 + 碰撞控制 -------------------------- //

        /// <summary>
        /// 当前机器人是否“在动”（只判断速度）
        /// </summary>
        private bool IsMoving_NoLock(RobotInstance r)
        {
            return r.Speed > 1e-3;
        }

        /// <summary>
        /// 判断两个机器人是否同向/相向
        /// </summary>
        private bool IsSameDirection(EnumMoveDirection a, EnumMoveDirection b)
        {
            return a == b;
        }

        private bool IsOppositeDirection(EnumMoveDirection a, EnumMoveDirection b)
        {
            return (a == EnumMoveDirection.Left && b == EnumMoveDirection.Right) ||
                   (a == EnumMoveDirection.Right && b == EnumMoveDirection.Left) ||
                   (a == EnumMoveDirection.Up && b == EnumMoveDirection.Down) ||
                   (a == EnumMoveDirection.Down && b == EnumMoveDirection.Up);
        }

        /// <summary>
        /// 简单判断“是否在转弯”
        /// </summary>
        private bool IsTurning_NoLock(RobotInstance r)
        {
            return r.Manager.IsTurning;
        }

        /// <summary>
        /// 碰撞处理：通过预测“下一格意图”进行让步（要求调用方已持有锁）
        /// </summary>
        private void HandleRobotCollisions_NoLock()
        {
            if (_robots.Count <= 1) // 少于等于 1 个机器人不需要碰撞检测
            {
                return; // 直接返回
            }

            var cur = new GridPos[_robots.Count]; // 当前格数组：cur[i] = 机器人 i 的当前格
            var next = new GridPos[_robots.Count]; // 预测格数组：next[i] = 机器人 i 的“下一步意图格”

            for (int i = 0; i < _robots.Count; i++) // 计算每个机器人当前格与意图下一格
            {
                RobotInstance r = _robots[i]; // 当前机器人
                cur[i] = r.GetGridPos_NoLock(); // 读取当前所在格
                next[i] = GetIntendedNextCell_NoLock(r, cur[i]); // 基于速度/方向/是否转向预测下一格
            }

            for (int i = 0; i < _robots.Count; i++) // 双重循环比较任意两机器人是否存在冲突
            {
                for (int j = i + 1; j < _robots.Count; j++) // j 从 i+1 开始避免重复/自比
                {
                    // 三类冲突：
                    bool iStationary = next[i].Equals(cur[i]);
                    bool jStationary = next[j].Equals(cur[j]);
                    if (iStationary && jStationary)
                    {
                        continue;
                    }
                    // 1) 同目标格：两者都想进同一格
                    bool sameTarget = next[i].Equals(next[j]); // 预测下一格完全相同

                    // 2) 交换格：对向互换
                    bool swap = next[i].Equals(cur[j]) && next[j].Equals(cur[i]); // 互相进入对方当前格

                    // 3) 穿入对方格：任一方下一步将进入对方当前格（防穿模核心）
                    bool enterOther =
                        next[i].Equals(cur[j]) || // i 进入 j 当前格
                        next[j].Equals(cur[i]);   // j 进入 i 当前格

                    if (!sameTarget && !swap && !enterOther) // 没有冲突则跳过
                    {
                        continue; // 继续下一对
                    }

                    RobotInstance a = _robots[i]; // 冲突对：机器人 a
                    RobotInstance b = _robots[j]; // 冲突对：机器人 b

                    if (IsInYieldCooldown_NoLock(a.Id) && IsInYieldCooldown_NoLock(b.Id)) // 若双方都在冷却期
                    {
                        continue; // 跳过，避免双方持续停车重规划造成抖动/饥饿
                    }

                    RobotInstance yield; // 让步方
                    RobotInstance go;    // 通行方

                    // 冷却期：双方都在冷却就跳过，不再额外处理
                    if (IsInYieldCooldown_NoLock(a.Id) && IsInYieldCooldown_NoLock(b.Id))
                    {
                        continue;
                    }

                    // 一方在冷却：优先让冷却中的那一方通行
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
                        // ---------- 交通规则开始 ----------

                        bool aTurning = IsTurning_NoLock(a);
                        bool bTurning = IsTurning_NoLock(b);
                        bool aMoving = IsMoving_NoLock(a);
                        bool bMoving = IsMoving_NoLock(b);

                        // 1) 转弯让直行：只要一方在转弯、另一方是直行，就让转弯那一方停车
                        if (aTurning && !bTurning && bMoving)
                        {
                            yield = a;
                            go = b;
                        }
                        else if (bTurning && !aTurning && aMoving)
                        {
                            yield = b;
                            go = a;
                        }
                        else
                        {
                            // 2) 都是直行或都在转弯：再看方向关系 + Id

                            EnumMoveDirection da = a.Manager.Direction;
                            EnumMoveDirection db = b.Manager.Direction;

                            if (IsOppositeDirection(da, db))
                            {
                                // 相向对冲：Id 大的那一方让路
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
                            else if (IsSameDirection(da, db))
                            {
                                // 同方向（追尾/并排行驶）：
                                // - 若其中一辆速度几乎为 0，则优先让“静止/慢车”让路
                                // - 否则仍然按照 Id 大者让路（保证确定性）
                                if (!aMoving && bMoving)
                                {
                                    yield = a;
                                    go = b;
                                }
                                else if (!bMoving && aMoving)
                                {
                                    yield = b;
                                    go = a;
                                }
                                else
                                {
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
                            }
                            else
                            {
                                // 3) 垂直交叉（十字路口）：简单版本——Id 大的让路
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
                        }
                        // ---------- 交通规则结束 ----------
                    }
                }
            }
        }

        /// <summary>
        /// 判断指定机器人是否处于“让步冷却期”
        /// </summary>
        private bool IsInYieldCooldown_NoLock(int robotId)
        {
            int t; // 冷却剩余帧数
            return _yieldCooldownTicks.TryGetValue(robotId, out t) && t > 0; // 存在且大于 0 则表示在冷却
        }

        /// <summary>
        /// 预测机器人“下一步意图进入的格子”（用于碰撞检测）
        /// </summary>
        private GridPos GetIntendedNextCell_NoLock(RobotInstance r, GridPos curCell)
        {
            // 默认意图为“不动”
            GridPos target = curCell; // 默认返回当前格（表示不移动）

            // 只对自动巡航做预测（手动连续角移动更复杂，先不在这里做阻拦）
            if (!r.AutoNavigator.IsEnabled) // 自动未启用则不预测
            {
                return target; // 返回当前格
            }

            // 转向中不预测（避免误判）
            //if (r.Manager.IsTurning) // 若正在转向
            //{
            //    return target; // 认为下一格不变
            //}

            // 关键：按“下一帧预测位置”推算下一格，避免速度较大时跨格穿模
            // 预测距离：max(speed * dt, 一个很小的最小值)，保证低速也能预测到相邻格意图
            double v = r.Speed; // 当前速度
            double move = v * _dt; // 预测本帧位移
            if (move < _cellSizeM * 0.15) // 经验值：低速时也至少预测 0.15 格
            {
                move = _cellSizeM * 0.15; // 提升最小预测位移，便于判定“意图进入相邻格”
            }

            int dx = 0; // 预测位移的格方向 X（-1/0/+1）
            int dy = 0; // 预测位移的格方向 Y（-1/0/+1）

            switch (r.Manager.Direction) // 根据离散方向决定 dx/dy
            {
                case EnumMoveDirection.Right: // 向右移动
                    dx = +1; // X+
                    break; // 结束分支
                case EnumMoveDirection.Left: // 向左移动
                    dx = -1; // X-
                    break; // 结束分支
                case EnumMoveDirection.Down: // 向下移动
                    dy = +1; // Y+
                    break; // 结束分支
                case EnumMoveDirection.Up: // 向上移动
                    dy = -1; // Y-
                    break; // 结束分支
            }

            // 用当前格中心 + 预测位移，映射到将要进入的格子
            double curCenterX = curCell.X * _cellSizeM + _cellSizeM / 2.0; // 当前格中心 X（世界坐标）
            double curCenterY = curCell.Y * _cellSizeM + _cellSizeM / 2.0; // 当前格中心 Y（世界坐标）

            double predX = curCenterX + dx * move; // 预测位置 X（世界坐标）
            double predY = curCenterY + dy * move; // 预测位置 Y（世界坐标）

            int gx = (int)Math.Floor(predX / _cellSizeM); // 将预测 X 转为格坐标 X（向下取整）
            int gy = (int)Math.Floor(predY / _cellSizeM); // 将预测 Y 转为格坐标 Y（向下取整）

            if (gx < 0)
            {
                gx = 0; // 左边界裁剪

            }
            if (gy < 0)
            {
                gy = 0; // 上边界裁剪
            }
            if (gx >= _gridCount)
            {
                gx = _gridCount - 1; // 右边界裁剪
            }
            if (gy >= _gridCount)
            {
                gy = _gridCount - 1; // 下边界裁剪
            }

            target = new GridPos(gx, gy); // 预测意图格
            return target; // 返回预测格
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

            for (int i = 0; i < _robots.Count; i++) // 为每个机器人设置 WalkableProvider（闭包捕获 myKey/occupied）
            {
                RobotInstance me = _robots[i]; // 当前机器人（要设置 provider 的对象）
                int myKey = cellKeys[i]; // 当前机器人自己的占用格 key

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

                    return !occupied.Contains(key); // 其它机器人占用格不可走，否则可走
                });

                // 需要在 RobotMove 中提供 SetIsWorldWalkableProvider(wx, wy) 之类的接口
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

                    return !occupied.Contains(key);  // 其它机器人所在格视为不可走
                });
            }
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