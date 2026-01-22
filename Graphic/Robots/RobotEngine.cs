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
            public int Id; // 机器人 ID（从 1 开始，稳定用于比较/让路策略）

            public EnumRobotProcessState State; // 单机器人当前流程状态

            public double X; // 世界坐标 X（米）
            public double Y; // 世界坐标 Y（米）
            public double Speed; // 线速度（米/秒）
            public double Acc; // 前向加速度（米/秒^2）

            public RobotManager Manager; // 机器人控制/指令调度器（Auto/Manual 模式切换、朝向等）
            public RobotMove Move; // 机器人运动/碰撞约束执行器（含转向控制）
            public RobotAutoNavigator AutoNavigator; // 自动导航器（从目标生成路径并产生命令）
            public RobotManual Manual; // 手动控制器（按键输入 -> 命令）

            // 随机目标（网格）
            public bool HasGoal; // 是否存在有效目标
            public GridPos Goal; // 目标网格坐标

            public GridPos? TempAvoidCell;
            public double AvoidTtlSec;
            public double ReplanCooldownSec;

            // 新增：随机路径段（命令队列）
            public readonly Queue<RobotCommand> AutoRandomCommandQueue = new Queue<RobotCommand>();
            public GridPos RandomPathLastCell;

            // 碰撞等待：一方原地停住一小段时间，另一方继续走
            public double WaitTtlSec; // 等待持续时间（秒）
        }

        private readonly object _robotLock = new object(); // 全局互斥锁：保护所有机器人状态读写一致性

        private readonly ObstacleMap _obstacleMap; // 网格障碍物地图（可编辑/快照）

        private readonly double _cellSizeM; // 单元格边长（米）
        private readonly double _dt; // Tick 步长（秒）
        private readonly int _gridCount; // 网格边长（N*N）

        private readonly double _worldWidthM; // 世界宽度（米）
        private readonly double _worldHeightM; // 世界高度（米）

        private readonly List<RobotContext> _robots = new List<RobotContext>(); // 机器人上下文列表
        private int _selectedIndex; // UI 选中机器人在列表中的索引

        private EnumRobotProcessState _globalState = EnumRobotProcessState.Idle; // 引擎全局状态（用于统一禁用 Tick）
        private bool _isObstacleEditMode; // 是否处于障碍编辑模式（编辑时暂停自动重规划）

        private readonly Random _rng = new Random(); // 随机数：用于出生点/目标生成

        // 与 RobotMove 内部一致：cell/3
        private readonly double _robotRadiusM; // 机器人半径（米），用于点击拾取/碰撞策略

        // 策略参数
        private const int DefaultRobotCount = 5; // 默认机器人数量
        private const double DefaultRobotAcc = 0.8; // 默认前向加速度
        private const double ArriveGoalEpsilonM = 0.25; // 到达目标的距离阈值（米）
        private const double AvoidCellTtlSec = 1.0; // 临时避让格持续时间（秒）
        private const double CollisionReplanCooldownSec = 0.35; // 碰撞后重规划冷却（秒）
        private const double CollisionWaitTtlSec = 0.5; // 碰撞让步等待时间（秒）

        // 随机路径段参数
        private const int RandomPathStepCount = 16;      // 一段路径多少步（格）
        private const int RandomPathBuildMaxTry = 40;    // 生成失败时重试次数
        private const int RandomPathPickNextMaxTry = 20; // 每一步找下一格的尝试次数（避免卡死）

        public RobotEngine( // 构造：初始化网格/地图/机器人，并默认进入自动导航
            int gridCount, // 网格尺寸 N
            double cellSizeM, // 单元格边长（米）
            double dt, // Tick 时间步长（秒）
            double initialMaxSpeed, // 初始最大速度（米/秒）
            EnumMoveDirection initialDirection) // 初始运动方向
        {
            _gridCount = gridCount; // 保存网格尺寸
            _cellSizeM = cellSizeM; // 保存单元格大小
            _dt = dt; // 保存 Tick 步长
            _worldWidthM = gridCount * cellSizeM; // 计算世界宽度
            _worldHeightM = gridCount * cellSizeM; // 计算世界高度

            _robotRadiusM = _cellSizeM / 3.0; // 机器人半径与运动模块保持一致

            _obstacleMap = new ObstacleMap(_gridCount, _gridCount); // 创建障碍地图（N*N）

            CreateRobots(DefaultRobotCount, initialMaxSpeed, initialDirection); // 创建默认数量机器人

            _selectedIndex = _robots.Count > 0 ? 0 : -1; // 默认选中第 1 台（若存在）

            // 默认全自动 + 每台随机目标
            _globalState = EnumRobotProcessState.AutoNavigating; // 全局进入自动导航
            for (int i = 0; i < _robots.Count; i++)
            {
                SetRobotState_NoLock(_robots[i], EnumRobotProcessState.AutoNavigating);
                EnsureRandomPathCommands_NoLock(_robots[i], force: true);
            }
        }

        #region 公共属性/方法（供 UI 调用）

        public double WorldWidthM => _worldWidthM; // 世界宽度（米）
        public double WorldHeightM => _worldHeightM; // 世界高度（米）
        public double CellSizeM => _cellSizeM; // 单元格大小（米）
        public int GridCount => _gridCount; // 网格边长 N

        public int SelectedRobotId // UI 读取当前选中机器人的 ID
        {
            get
            {
                lock (_robotLock) // 加锁保护索引与列表一致性
                {
                    if (_selectedIndex < 0 || _selectedIndex >= _robots.Count) // 防御：无选中或越界
                    {
                        return 0; // 约定：无选中返回 0
                    }

                    return _robots[_selectedIndex].Id; // 返回选中机器人的 ID
                }
            }
        }

        // 兼容旧 Form1：返回“选中机器人”的 AutoNavigator（DestinationPicker 仍可存在，但不再作为目标来源）
        public RobotAutoNavigator AutoNavigator // UI 获取选中机器人的导航状态/路径用于绘制
        {
            get
            {
                lock (_robotLock) // 加锁：防止并发读写
                {
                    if (_selectedIndex < 0 || _selectedIndex >= _robots.Count) // 防御：无选中或越界
                    {
                        return null; // 无有效对象则返回 null
                    }

                    return _robots[_selectedIndex].AutoNavigator; // 返回当前选中机器人的导航器
                }
            }
        }

        public EnumPathfindingAlgorithm Algorithm // 全局寻路算法（对所有机器人同步设置）
        {
            get
            {
                lock (_robotLock) // 加锁读取
                {
                    if (_selectedIndex < 0 || _selectedIndex >= _robots.Count) // 防御：无选中/越界
                    {
                        return EnumPathfindingAlgorithm.AStar; // 默认回退 A*
                    }

                    return _robots[_selectedIndex].AutoNavigator.Algorithm; // 读取选中机器人的算法设置
                }
            }
            set
            {
                lock (_robotLock) // 加锁写入
                {
                    // 仍保留 UI 操作入口，但随机路径模式不依赖该算法
                    for (int i = 0; i < _robots.Count; i++)
                    {
                        _robots[i].AutoNavigator.Algorithm = value;
                    }
                }
            }
        }

        public bool AutoEnabled // UI 判断选中机器人是否处于自动导航状态
        {
            get
            {
                lock (_robotLock) // 加锁读取
                {
                    if (_selectedIndex < 0 || _selectedIndex >= _robots.Count) // 防御：无选中/越界
                    {
                        return false; // 不可用则视为未启用
                    }

                    return _robots[_selectedIndex].State == EnumRobotProcessState.AutoNavigating; // 选中机器人是否自动
                }
            }
        }

        public MultiRobotStateSnapshot GetMultiStateSnapshot() // 生成所有机器人状态快照（供 UI 绘制）
        {
            lock (_robotLock) // 加锁：保证同一帧数据一致
            {
                var list = new List<RobotItemSnapshot>(_robots.Count); // 预分配容量减少分配

                for (int i = 0; i < _robots.Count; i++)
                {
                    var r = _robots[i];

                    // 随机路径模式：不再暴露目标点，UI 目标十字不画
                    r.HasGoal = false;

                    list.Add(new RobotItemSnapshot(
                        id: r.Id,
                        x: r.X,
                        y: r.Y,
                        speed: r.Speed,
                        acc: r.Acc,
                        orientationAngle: r.Manager.OrientationAngle,
                        direction: r.Manager.Direction,
                        isSelected: i == _selectedIndex,
                        hasGoal: false,
                        goalGridX: 0,
                        goalGridY: 0,
                        goalWorldX: 0,
                        goalWorldY: 0));
                }

                return new MultiRobotStateSnapshot(list);
            }
        }

        public bool TrySelectRobotByWorld(double worldX, double worldY) // 按世界坐标点击选择一台机器人
        {
            lock (_robotLock) // 加锁：避免与 Tick 并发
            {
                if (_robots.Count == 0) // 无机器人则无法选择
                {
                    return false; // 选择失败
                }

                double pickR = _robotRadiusM * 1.5; // 拾取半径（略大于机器人半径）
                double pickR2 = pickR * pickR; // 拾取半径平方（避免开方）

                int bestIndex = -1; // 最近命中的机器人索引
                double bestD2 = double.MaxValue; // 最近距离平方

                for (int i = 0; i < _robots.Count; i++) // 遍历所有机器人做命中测试
                {
                    double dx = _robots[i].X - worldX; // X 方向偏差
                    double dy = _robots[i].Y - worldY; // Y 方向偏差
                    double d2 = dx * dx + dy * dy; // 距离平方

                    if (d2 <= pickR2 && d2 < bestD2) // 在半径内且更近
                    {
                        bestD2 = d2; // 更新最佳距离
                        bestIndex = i; // 更新命中索引
                    }
                }

                if (bestIndex < 0) // 没有任何机器人被点中
                {
                    return false; // 选择失败
                }

                _selectedIndex = bestIndex; // 更新选中索引
                return true; // 选择成功
            }
        }

        public List<(double X, double Y)> GetPathWorldPointsSnapshot() // 获取选中机器人当前路径点（世界坐标）
        {
            lock (_robotLock) // 加锁：保证与导航器状态一致
            {
                if (_selectedIndex < 0 || _selectedIndex >= _robots.Count) // 防御：无选中/越界
                {
                    return new List<(double X, double Y)>(); // 返回空路径
                }

                // 仅画选中机器人的路径
                return _robots[_selectedIndex].AutoNavigator.GetPathWorldPointsSnapshot(); // 从导航器获取快照点
            }
        }

        public bool[,] GetObstacleSnapshot() // UI 获取障碍图快照
        {
            return _obstacleMap.GetSnapshot(); // 直接返回地图快照（ObstacleMap 内部自行保护）
        }

        public void ToggleObstacle(GridPos p) // 切换某格是否为障碍（编辑/点击）
        {
            _obstacleMap.Toggle(p); // 先更新地图数据

            if (_isObstacleEditMode) // 若处于编辑模式
            {
                return; // 不触发自动重规划（避免编辑过程中频繁重算）
            }

            lock (_robotLock) // 不在编辑模式：需要通知自动机器人重规划
            {
                for (int i = 0; i < _robots.Count; i++) // 遍历所有机器人
                {
                    if (_robots[i].State == EnumRobotProcessState.AutoNavigating) // 仅自动模式需要重规划
                    {
                        _robots[i].AutoRandomCommandQueue.Clear(); // 重建路径
                        _robots[i].Manager.ResetAutoCommands(); // 清空队列，下一 Tick 重新拉取
                    }
                }
            }
        }

        public void ClearObstacles() // 清空全部障碍
        {
            _obstacleMap.Clear(); // 清空地图

            if (_isObstacleEditMode) // 编辑模式下不触发重规划
            {
                return; // 直接返回
            }

            lock (_robotLock) // 非编辑模式：统一重规划
            {
                for (int i = 0; i < _robots.Count; i++) // 遍历所有机器人
                {
                    if (_robots[i].State == EnumRobotProcessState.AutoNavigating) // 仅自动模式
                    {
                        _robots[i].AutoRandomCommandQueue.Clear(); // 重建路径
                        _robots[i].Manager.ResetAutoCommands(); // 重置自动命令
                    }
                }
            }
        }

        public void SetObstacleEditMode(bool enabled) // 打开/关闭障碍编辑模式
        {
            lock (_robotLock) // 加锁：全局状态切换需要一致
            {
                _isObstacleEditMode = enabled; // 保存编辑模式标记

                if (enabled) // 进入编辑模式
                {
                    _globalState = EnumRobotProcessState.ObstacleEditing; // 全局进入编辑态（Tick 直接 return）

                    for (int i = 0; i < _robots.Count; i++) // 遍历所有机器人
                    {
                        SetRobotState_NoLock(_robots[i], EnumRobotProcessState.ObstacleEditing); // 每台立即停车并禁用控制器
                    }
                }
                else // 退出编辑模式
                {
                    _globalState = EnumRobotProcessState.AutoNavigating; // 回到自动导航

                    for (int i = 0; i < _robots.Count; i++) // 遍历每台
                    {
                        SetRobotState_NoLock(_robots[i], EnumRobotProcessState.AutoNavigating); // 切到自动
                        EnsureRandomPathCommands_NoLock(_robots[i], force: true); // 若没有路径则生成路径
                                                                                  // 重建路径
                        _robots[i].Manager.ResetAutoCommands(); // 清空命令队列
                    }
                }
            }
        }

        public void EnableAuto() // 全部机器人启用自动导航
        {
            lock (_robotLock) // 加锁：全局切换
            {
                _globalState = EnumRobotProcessState.AutoNavigating; // 设置全局状态自动

                for (int i = 0; i < _robots.Count; i++) // 遍历全部机器人
                {
                    SetRobotState_NoLock(_robots[i], EnumRobotProcessState.AutoNavigating); // 切换到自动状态
                    EnsureRandomPathCommands_NoLock(_robots[i], force: true); // 若没有路径则生成路径
                    _robots[i].AutoRandomCommandQueue.Clear(); // 重建路径
                    _robots[i].Manager.ResetAutoCommands(); // 重置自动命令
                    _robots[i].Manager.AlignOrientationToDirectionWithTurn(); // 让朝向与方向一致（通过转向控制器）
                }
            }
        }

        public void EnableManual() // 仅选中机器人手动，其余继续自动
        {
            lock (_robotLock) // 加锁：一致切换
            {
                _globalState = EnumRobotProcessState.AutoNavigating; // Tick 继续运行（手动也需要 Tick）

                for (int i = 0; i < _robots.Count; i++) // 遍历所有机器人
                {
                    if (i == _selectedIndex) // 选中机器人
                    {
                        SetRobotState_NoLock(_robots[i], EnumRobotProcessState.ManualControl); // 切换为手动控制
                    }
                    else // 非选中机器人
                    {
                        SetRobotState_NoLock(_robots[i], EnumRobotProcessState.AutoNavigating);
                        _robots[i].AutoRandomCommandQueue.Clear();
                        EnsureRandomPathCommands_NoLock(_robots[i], force: true);
                    }
                }
            }
        }

        public void RebuildPath() // 对所有自动机器人强制重建路径
        {
            lock (_robotLock) // 加锁：导航器/指令队列一致
            {
                // 随机路径模式：RebuildPath 表现为“重新采样一段路径”
                for (int i = 0; i < _robots.Count; i++)
                {
                    if (_robots[i].State == EnumRobotProcessState.AutoNavigating)
                    {
                        _robots[i].AutoRandomCommandQueue.Clear();
                        EnsureRandomPathCommands_NoLock(_robots[i], force: true);
                        _robots[i].Manager.ResetAutoCommands();
                    }
                }
            }
        }

        public void SetForwardAcc(double acc) // 设置选中机器人的前向加速度
        {
            lock (_robotLock) // 加锁：与 Tick 并发安全
            {
                if (_selectedIndex < 0 || _selectedIndex >= _robots.Count) // 防御：无选中/越界
                {
                    return; // 直接忽略
                }

                _robots[_selectedIndex].Acc = acc; // 更新加速度参数（Manager.Tick 中通过回调读取）
            }
        }

        public void SetMaxSpeed(double vmax) // 设置所有机器人的最大速度
        {
            lock (_robotLock) // 加锁：与 Tick 并发安全
            {
                for (int i = 0; i < _robots.Count; i++) // 遍历所有机器人
                {
                    _robots[i].Manager.MaxSpeed = vmax; // 写入管理器最大速度限制
                }
            }
        }

        public void ManualForwardKey(bool down) // 手动：前进键按下/松开
        {
            lock (_robotLock) // 加锁：与 Tick 并发安全
            {
                if (_selectedIndex < 0 || _selectedIndex >= _robots.Count) // 防御：无选中/越界
                {
                    return; // 忽略输入
                }

                var r = _robots[_selectedIndex]; // 取选中机器人
                r.Manual.InputForwardKey(down); // 写入手动输入状态
                r.Manager.ResetManualCommands(); // 清空旧的手动命令以便下一 Tick 生成新命令
            }
        }

        public void ManualTurnLeftKey(bool down) // 手动：左转键按下/松开
        {
            lock (_robotLock) // 加锁
            {
                if (_selectedIndex < 0 || _selectedIndex >= _robots.Count) // 防御
                {
                    return; // 忽略
                }

                var r = _robots[_selectedIndex]; // 取选中机器人
                r.Manual.InputTurnLeftKey(down); // 写入左转输入
                r.Manager.ResetManualCommands(); // 重置手动命令队列
            }
        }

        public void ManualTurnRightKey(bool down) // 手动：右转键按下/松开
        {
            lock (_robotLock) // 加锁
            {
                if (_selectedIndex < 0 || _selectedIndex >= _robots.Count) // 防御
                {
                    return; // 忽略
                }

                var r = _robots[_selectedIndex]; // 取选中机器人
                r.Manual.InputTurnRightKey(down); // 写入右转输入
                r.Manager.ResetManualCommands(); // 重置手动命令队列
            }
        }

        public void Tick() // 主循环：调度指令、推进运动、处理碰撞
        {
            lock (_robotLock) // 单锁串行化所有机器人更新
            {
                if (_globalState == EnumRobotProcessState.ObstacleEditing // 编辑模式：暂停
                    || _globalState == EnumRobotProcessState.Idle // 待命：暂停
                    || _globalState == EnumRobotProcessState.Error) // 错误：暂停
                {
                    return; // 不推进
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

                // 2) 自动机器人：若随机段命令为空，则补一段
                for (int i = 0; i < _robots.Count; i++)
                {
                    var r = _robots[i];
                    if (r.State != EnumRobotProcessState.AutoNavigating)
                    {
                        continue;
                    }

                    r.HasGoal = false;

                    if (r.AutoRandomCommandQueue.Count == 0)
                    {
                        EnsureRandomPathCommands_NoLock(r, force: true);
                        r.Manager.ResetAutoCommands();
                    }
                }

                // 3) 推进每台机器人
                for (int i = 0; i < _robots.Count; i++)
                {
                    var r = _robots[i];

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

                // 4) 碰撞处理（保留原逻辑：等待 + “前进者临时避让格”）
                ResolveRobotRobotCollisionAndReplan_NoLock();
            }
        }

        public RobotStateSnapshot GetStateSnapshot() // 获取选中机器人的状态快照（兼容旧接口）
        {
            lock (_robotLock) // 加锁读取
            {
                if (_selectedIndex < 0 || _selectedIndex >= _robots.Count) // 防御：无选中/越界
                {
                    return default(RobotStateSnapshot); // 返回默认值
                }

                var r = _robots[_selectedIndex]; // 取选中机器人

                return new RobotStateSnapshot( // 构造快照返回
                    X: r.X, // 世界 X
                    Y: r.Y, // 世界 Y
                    Speed: r.Speed, // 速度
                    Acc: r.Acc, // 加速度
                    OrientationAngle: r.Manager.OrientationAngle); // 朝向角
            }
        }

        #endregion

        #region Private

        private void CreateRobots(int robotCount, double initialMaxSpeed, EnumMoveDirection initialDirection) // 创建并初始化机器人列表
        {
            _robots.Clear(); // 清空旧机器人列表

            // 避免出生点重叠：记录已占用格子
            var occupied = new HashSet<GridPos>(); // 已被分配的起点格集合

            // 若障碍很多，允许多尝试
            int maxTryPerRobot = 500; // 每台机器人最多尝试次数

            for (int i = 0; i < robotCount; i++) // 按数量创建机器人
            {
                int id = i + 1; // 人类友好：ID 从 1 开始

                // 1) 随机挑一个可用起点格
                GridPos start = default(GridPos); // 起点格（默认值占位）
                bool found = false; // 是否找到有效起点

                for (int t = 0; t < maxTryPerRobot; t++) // 多次尝试找空位
                {
                    int gx = _rng.Next(0, _gridCount); // 随机格子 X
                    int gy = _rng.Next(0, _gridCount); // 随机格子 Y
                    var p = new GridPos(gx, gy); // 组合成 GridPos

                    if (_obstacleMap.IsObstacle(p)) // 该格是障碍则跳过
                    {
                        continue; // 继续尝试
                    }

                    if (occupied.Contains(p)) // 该格已被其他机器人占用
                    {
                        continue; // 继续尝试
                    }

                    start = p; // 记录起点
                    found = true; // 标记找到
                    break; // 退出尝试循环
                }

                if (!found) // 尝试耗尽仍未找到
                {
                    // 找不到空位：允许重复，但继续运行（避免直接崩）
                    start = new GridPos(_rng.Next(0, _gridCount), _rng.Next(0, _gridCount)); // 放宽约束随机一个
                }

                occupied.Add(start); // 将起点登记为占用

                double x = start.X * _cellSizeM + _cellSizeM / 2.0; // 起点世界 X（格子中心）
                double y = start.Y * _cellSizeM + _cellSizeM / 2.0; // 起点世界 Y（格子中心）

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
                ctx.RandomPathLastCell = start;

                ctx.Manager = new RobotManager( // 创建管理器（控制/指令调度中心）
                    acc: ctx.Acc, // 初始加速度
                    maxSpeed: initialMaxSpeed, // 最大速度
                    direction: initialDirection); // 初始方向

                ctx.Move = new RobotMove( // 创建运动执行器（包含边界/障碍可行走判断）
                    robotLock: _robotLock, // 传入共享锁（保持与引擎一致）
                    getRobotX: () => ctx.X, // 读取 X 委托
                    setRobotX: v => ctx.X = v, // 写入 X 委托
                    getRobotY: () => ctx.Y, // 读取 Y 委托
                    setRobotY: v => ctx.Y = v, // 写入 Y 委托
                    getRobotSpeed: () => ctx.Speed, // 读取速度委托
                    setRobotSpeed: v => ctx.Speed = v, // 写入速度委托
                    robotManager: ctx.Manager, // 关联管理器（用于获取方向/转向等）
                    getWorldWidthM: () => _worldWidthM, // 世界宽度提供者
                    getWorldHeightM: () => _worldHeightM, // 世界高度提供者
                    cellSizeM: _cellSizeM, // 单元格大小
                    dt: _dt, // 时间步长
                    isWorldWalkable: (wx, wy) => // 世界坐标是否可行走判定（障碍+临时避让+边界）
                    {
                        int cgx = (int)Math.Floor(wx / _cellSizeM); // 将世界 X 转换为格子 X
                        int cgy = (int)Math.Floor(wy / _cellSizeM); // 将世界 Y 转换为格子 Y

                        if (cgx < 0 || cgy < 0 || cgx >= _gridCount || cgy >= _gridCount) // 越界则不可走
                        {
                            return false; // 不可行走
                        }

                        if (_obstacleMap.IsObstacle(new GridPos(cgx, cgy))) // 静态障碍不可走
                        {
                            return false; // 不可行走
                        }

                        if (ctx.TempAvoidCell.HasValue && ctx.TempAvoidCell.Value.Equals(new GridPos(cgx, cgy))) // 临时避让格不可走
                        {
                            return false; // 不可行走
                        }

                        return true; // 默认可行走
                    });

                ctx.AutoNavigator = new RobotAutoNavigator( // 创建自动导航器（寻路与命令生成）
                    robotLock: _robotLock, // 共享锁
                    getRobotX: () => ctx.X, // 获取机器人世界 X
                    getRobotY: () => ctx.Y, // 获取机器人世界 Y
                    setRobotSpeed: v => ctx.Speed = v, // 允许导航器直接设置速度（必要时）
                    getWorldWidthM: () => _worldWidthM, // 世界宽度
                    getWorldHeightM: () => _worldHeightM, // 世界高度
                    cellSizeM: _cellSizeM, // 单元格大小
                    robotManager: ctx.Manager); // 关联管理器

                ctx.AutoNavigator.SetIsWalkableProvider(p => // 设置网格可行走提供者（用于寻路）
                {
                    if (_obstacleMap.IsObstacle(p)) // 静态障碍不可走
                    {
                        return false; // 返回不可走
                    }

                    if (ctx.TempAvoidCell.HasValue && ctx.TempAvoidCell.Value.Equals(p)) // 临时避让格不可走
                    {
                        return false; // 返回不可走
                    }

                    return true; // 其它情况可走
                });

                ctx.Manual = new RobotManual( // 创建手动控制器
                    robotLock: _robotLock, // 共享锁
                    robotManager: ctx.Manager); // 绑定管理器

                ctx.Manager.BindRuntime( // 将运行时组件绑定到管理器（自动命令来源/运动与转向控制）
                    robotLock: _robotLock, // 共享锁
                    move: ctx.Move, // 运动执行器
                    turn: ctx.Move.TurnController, // 转向控制器（由 Move 提供）
                    autoCommandProvider: () => ctx.AutoNavigator.TryBuildNextCommand(), // 自动命令提供者：按需生成下一条
                    getForwardAcc: () => ctx.Acc); // 动态获取当前加速度（允许 UI 调整）

                ctx.Manager.BindManualCommandProvider(() => ctx.Manual.TryBuildNextCommand()); // 绑定手动命令提供者

                _robots.Add(ctx); // 加入机器人列表
            }
        }

        private RobotCommand TryBuildRandomAutoCommand_NoLock(RobotContext r)
        {
            // 注意：RobotManager.Tick 内部会在锁里调用该 provider，因此这里不再加锁
            if (r.State != EnumRobotProcessState.AutoNavigating)
            {
                return null;
            }

            if (r.WaitTtlSec > 0)
            {
                return RobotCommand.MoveDistance(0.0);
            }

            if (r.AutoRandomCommandQueue.Count == 0)
            {
                EnsureRandomPathCommands_NoLock(r, force: true);
            }

            if (r.AutoRandomCommandQueue.Count == 0)
            {
                return null;
            }

            return r.AutoRandomCommandQueue.Dequeue();
        }

        private void EnsureRandomPathCommands_NoLock(RobotContext r, bool force)
        {
            if (!force && r.AutoRandomCommandQueue.Count > 0)
            {
                return;
            }

            r.AutoRandomCommandQueue.Clear();

            GridPos start = WorldToGrid_NoLock(r.X, r.Y);
            r.RandomPathLastCell = start;

            for (int t = 0; t < RandomPathBuildMaxTry; t++)
            {
                List<GridPos> cells = TryBuildRandomWalkCells_NoLock(start, RandomPathStepCount);
                if (cells == null || cells.Count < 2)
                {
                    continue;
                }

                BuildCommandsFromCells_NoLock(r, cells);
                if (r.AutoRandomCommandQueue.Count > 0)
                {
                    return;
                }
            }
        }

        private List<GridPos> TryBuildRandomWalkCells_NoLock(GridPos start, int stepCount)
        {
            if (_obstacleMap.IsObstacle(start))
            {
                return null;
            }

            var path = new List<GridPos>(stepCount + 1);
            path.Add(start);

            GridPos cur = start;

            for (int i = 0; i < stepCount; i++)
            {
                GridPos next;
                if (!TryPickNextNeighborCell_NoLock(cur, out next))
                {
                    break;
                }

                path.Add(next);
                cur = next;
            }

            return path;
        }

        private bool TryPickNextNeighborCell_NoLock(GridPos cur, out GridPos next)
        {
            // 固定四邻域候选，随机打乱尝试
            var candidates = new GridPos[4];
            candidates[0] = new GridPos(cur.X + 1, cur.Y);
            candidates[1] = new GridPos(cur.X - 1, cur.Y);
            candidates[2] = new GridPos(cur.X, cur.Y + 1);
            candidates[3] = new GridPos(cur.X, cur.Y - 1);

            for (int i = 0; i < candidates.Length; i++)
            {
                int j = _rng.Next(i, candidates.Length);
                var tmp = candidates[i];
                candidates[i] = candidates[j];
                candidates[j] = tmp;
            }

            for (int k = 0; k < candidates.Length && k < RandomPathPickNextMaxTry; k++)
            {
                var p = candidates[k];

                if (p.X < 0 || p.Y < 0 || p.X >= _gridCount || p.Y >= _gridCount)
                {
                    continue;
                }

                if (_obstacleMap.IsObstacle(p))
                {
                    continue;
                }

                next = p;
                return true;
            }

            next = default(GridPos);
            return false;
        }

        private void BuildCommandsFromCells_NoLock(RobotContext r, List<GridPos> cells)
        {
            // 将格子序列转为 TurnAngle + MoveDistance（每步一格）
            // 说明：不做“同方向合并”，避免命令计算复杂化；需要合并可后续加
            for (int i = 1; i < cells.Count; i++)
            {
                GridPos prev = cells[i - 1];
                GridPos cur = cells[i];

                int dx = cur.X - prev.X;
                int dy = cur.Y - prev.Y;

                EnumMoveDirection desiredDir;
                if (dx == 1 && dy == 0) desiredDir = EnumMoveDirection.Right;
                else if (dx == -1 && dy == 0) desiredDir = EnumMoveDirection.Left;
                else if (dx == 0 && dy == 1) desiredDir = EnumMoveDirection.Down;
                else if (dx == 0 && dy == -1) desiredDir = EnumMoveDirection.Up;
                else
                {
                    continue;
                }

                double? turnAngle = TryGetTurnAngleRad(r.Manager.Direction, desiredDir);
                if (turnAngle.HasValue)
                {
                    r.AutoRandomCommandQueue.Enqueue(RobotCommand.TurnAngle(turnAngle.Value));
                }

                r.AutoRandomCommandQueue.Enqueue(RobotCommand.MoveDistance(_cellSizeM));

                // 关键：这里要“同步”方向，否则下一步的转角计算会始终基于旧方向
                r.Manager.Direction = desiredDir;
            }
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
                    r.AutoRandomCommandQueue.Clear();
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
                    r.AutoNavigator.Disable(); // 明确：随机路径模式不启用 AutoNavigator 出命令
                    r.AutoRandomCommandQueue.Clear();
                    EnsureRandomPathCommands_NoLock(r, force: true);

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
                    r.AutoRandomCommandQueue.Clear();
                    break;

                case EnumRobotProcessState.Idle:
                    r.Manager.SetMode(EnumRobotControlMode.Auto);
                    r.AutoNavigator.Disable();
                    r.Move.StopImmediately_NoLock();
                    r.AutoRandomCommandQueue.Clear();
                    break;

                case EnumRobotProcessState.Error:
                    r.Move.StopImmediately_NoLock();
                    r.AutoNavigator.Disable();
                    r.Manual.Disable();
                    r.Manager.ResetAutoCommands();
                    r.AutoRandomCommandQueue.Clear();
                    break;
            }
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

                    bool isSameCell = dx == 0 && dy == 0;
                    bool is4Neighbor = (dx + dy) == 1;

                    if (!isSameCell && !is4Neighbor)
                    {
                        continue;
                    }

                    if (a.WaitTtlSec > 0 || b.WaitTtlSec > 0)
                    {
                        continue;
                    }

                    bool aAuto = a.State == EnumRobotProcessState.AutoNavigating;
                    bool bAuto = b.State == EnumRobotProcessState.AutoNavigating;

                    if (!aAuto && !bAuto)
                    {
                        continue;
                    }

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

                    yield.WaitTtlSec = CollisionWaitTtlSec;
                    yield.Move.StopImmediately_NoLock();
                    yield.Manager.ResetAutoCommands();

                    if (go.State == EnumRobotProcessState.AutoNavigating && go.ReplanCooldownSec <= 0)
                    {
                        go.TempAvoidCell = WorldToGrid_NoLock(yield.X, yield.Y);
                        go.AvoidTtlSec = AvoidCellTtlSec;

                        // 随机路径模式：不 RebuildPath，直接清空队列并重采样一段
                        go.AutoRandomCommandQueue.Clear();
                        EnsureRandomPathCommands_NoLock(go, force: true);
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
    internal readonly struct RobotStateSnapshot // 单机器人显示所需状态（不可变）
    {
        /// <summary>
        /// 构造快照：一次性拷贝当前帧需要展示的状态。
        /// </summary>
        public RobotStateSnapshot(double X, double Y, double Speed, double Acc, double OrientationAngle) // 构造函数
        {
            this.X = X; // 拷贝 X
            this.Y = Y; // 拷贝 Y
            this.Speed = Speed; // 拷贝速度
            this.Acc = Acc; // 拷贝加速度
            this.OrientationAngle = OrientationAngle; // 拷贝朝向角
        }

        public double X { get; } // 世界坐标 X（米）
        public double Y { get; } // 世界坐标 Y（米）
        public double Speed { get; } // 速度（米/秒）
        public double Acc { get; } // 加速度（米/秒^2）
        public double OrientationAngle { get; } // 朝向角（弧度/角度：由 Manager 定义）
    }
}