using Graphic.Maps;
using GridDemo.RobotModels.Pathfinding;
using GridDemo.Robots;
using System;
using System.Collections.Generic;

namespace GridDemo.MultiRobots
{
    /// <summary>
    /// RobotWorld：承载“世界级别”的基础数据与操作。
    /// 职责：
    /// - 保存所有机器人列表与障碍地图；
    /// - 管理世界尺寸、网格参数；
    /// - 提供线程安全的 SelectedRobotId 管理接口；
    /// - 提供通用工具：随机空闲格生成、已占用格集合、终点拥有者映射等。
    ///
    /// 注意：
    /// - 不包含仿真 Tick 逻辑；
    /// - 不包含路径抢占 / 动态 walkable / 控制模式等高级策略。
    /// </summary>
    internal sealed class RobotWorld
    {
        private readonly object _robotLock;
        private readonly Random _rng = new Random();

        private readonly ObstacleMap _obstacleMap;
        private readonly List<RobotInstance> _robots = new List<RobotInstance>();

        private int _selectedRobotId = -1;

        private readonly int _gridCount;
        private readonly double _cellSizeM;
        private readonly double _dt;
        private readonly double _worldWidthM;
        private readonly double _worldHeightM;

        public RobotWorld(
            object robotLock,
            int gridCount,
            double cellSizeM,
            double dt)
        {
            _robotLock = robotLock;
            _gridCount = gridCount;
            _cellSizeM = cellSizeM;
            _dt = dt;

            _worldWidthM = gridCount * cellSizeM;
            _worldHeightM = gridCount * cellSizeM;

            _obstacleMap = new ObstacleMap(gridCount, gridCount);
        }

        /// <summary> 引擎使用的全局锁（由 RobotEngine 传入）。 </summary>
        public object RobotLock => _robotLock;

        /// <summary> 静态障碍物地图。 </summary>
        public ObstacleMap ObstacleMap => _obstacleMap;

        /// <summary> 所有机器人集合（注意：需要在持有 RobotLock 的前提下访问/迭代）。 </summary>
        public IList<RobotInstance> Robots => _robots;

        public int GridCount => _gridCount;
        public double CellSizeM => _cellSizeM;
        public double Dt => _dt;
        public double WorldWidthM => _worldWidthM;
        public double WorldHeightM => _worldHeightM;

        /// <summary>
        /// 当前选中机器人 Id（-1 表示未选中）。
        /// 对外通过属性封装，内部访问仍需注意锁。
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
            set
            {
                lock (_robotLock)
                {
                    if (value < 0 || value >= _robots.Count)
                    {
                        _selectedRobotId = -1;
                    }
                    else
                    {
                        _selectedRobotId = value;
                    }
                }
            }
        }

        /// <summary>
        /// 在已持有 lock 的前提下获取一个“有效”的选中机器人实例：
        /// - 若当前未选中，则选中 0；
        /// - 若越界，则修正到最后一个；
        /// - 若当前没有机器人，则返回 null。
        /// </summary>
        public RobotInstance GetSelectedRobot_NoLock()
        {
            if (_robots.Count == 0)
                return null;

            if (_selectedRobotId < 0)
                _selectedRobotId = 0;
            if (_selectedRobotId >= _robots.Count)
                _selectedRobotId = _robots.Count - 1;

            return _robots[_selectedRobotId];
        }

        /// <summary> 在锁保护下往世界中添加一个机器人。 </summary>
        public void AddRobot(RobotInstance robot)
        {
            lock (_robotLock)
            {
                _robots.Add(robot);
            }
        }

        /// <summary> 在锁保护下按索引删除一个机器人实例。 </summary>
        public void RemoveRobotAt(int index)
        {
            lock (_robotLock)
            {
                _robots.RemoveAt(index);
            }
        }

        /// <summary>
        /// 随机选择一个非障碍且不在 used 集合中的格子。
        /// 要求：调用方已持有 RobotLock。
        /// 用途：为新机器人生成初始格、为自动巡航生成随机目标等。
        /// </summary>
        public GridPos PickRandomFreeCell_NoLock(HashSet<int> used)
        {
            for (int tries = 0; tries < 5000; tries++)
            {
                int x = _rng.Next(0, _gridCount);
                int y = _rng.Next(0, _gridCount);
                int key = y * _gridCount + x;

                if (used != null && used.Contains(key))
                    continue;

                var p = new GridPos(x, y);
                if (_obstacleMap.IsObstacle(p))
                    continue;

                return p;
            }

            // 兜底返回：调用方需自己容错（(0,0) 可能是障碍或已占用）
            return new GridPos(0, 0);
        }

        /// <summary>
        /// 构建所有机器人当前占用格 key（y*W + x）集合。
        /// 要求：调用方已持有锁。
        /// 用途：新增机器人时避免出生点重叠。
        /// </summary>
        public HashSet<int> BuildUsedCellKeySet_NoLock()
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
        /// 构建“终点拥有者”映射：cellKey -> robotId。
        /// 要求：调用方已持有锁。
        /// 用途：给 AutoNavigator.RebuildPath 用于把其它机器人的目标格视为不可走。
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
}