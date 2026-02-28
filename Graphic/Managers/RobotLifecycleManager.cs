using GridDemo.MultiRobots;
using GridDemo.RobotModels.Pathfinding;
using GridDemo.Robots;
using System;
using System.Collections.Generic;

namespace GridDemo.Managers
{
    /// <summary>
    /// 机器人生命周期管理器：
    /// 职责：
    /// - 管理机器人数量的动态增减（新增/删除/修正选中项）；
    /// - 重置为单机器人模式；
    /// - 新增机器人时随机放置到空闲格，并分配随机目标。
    ///
    /// 设计原则：
    /// - 所有 _NoLock 方法要求调用方已持有 RobotLock；
    /// - 不持有引擎状态（processState/isRunning），由 RobotEngine 管理。
    /// </summary>
    internal sealed class RobotLifecycleManager
    {
        private readonly RobotWorld _world;
        private readonly PathClaimManager _pathClaimManager;
        private readonly DynamicWalkableBinder _walkableBinder;
        private readonly RobotControlManager _control;

        private readonly int _gridCount;
        private readonly double _cellSizeM;
        private readonly double _dt;
        private readonly double _worldWidthM;
        private readonly double _worldHeightM;

        public RobotLifecycleManager(
            RobotWorld world,
            PathClaimManager pathClaimManager,
            DynamicWalkableBinder walkableBinder,
            RobotControlManager control,
            int gridCount,
            double cellSizeM,
            double dt,
            double worldWidthM,
            double worldHeightM)
        {
            _world = world ?? throw new ArgumentNullException(nameof(world));
            _pathClaimManager = pathClaimManager ?? throw new ArgumentNullException(nameof(pathClaimManager));
            _walkableBinder = walkableBinder ?? throw new ArgumentNullException(nameof(walkableBinder));
            _control = control ?? throw new ArgumentNullException(nameof(control));
            _gridCount = gridCount;
            _cellSizeM = cellSizeM;
            _dt = dt;
            _worldWidthM = worldWidthM;
            _worldHeightM = worldHeightM;
        }

        /// <summary>
        /// 设置机器人数量（不加锁版本，要求调用方已持有 RobotLock）：
        /// - 多 -> 少：删除尾部机器人（先清理其所有抢占/状态）；
        /// - 少 -> 多：新增机器人，随机放到空闲格，并给随机目标；
        /// - 最后重绑动态 walkable。
        /// </summary>
        public void SetRobotCount_NoLock(int count, double initialMaxSpeed, EnumMoveDirection initialDirection)
        {
            if (count < 1)
            {
                count = 1;
            }

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
            {
                _world.SelectedRobotId = robots.Count > 0 ? robots.Count - 1 : -1;
            }

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
                        robotLock: _world.RobotLock,
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
                    {
                        r.Acc = robots[0].Acc;
                    }

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

        /// <summary>
        /// 将引擎重置成单机器人（不加锁版本，要求调用方已持有 RobotLock）：
        /// - 重置为 1 台机器人；
        /// - 清理引擎侧残留状态（路径/抢占/暂停等）；
        /// - 清理机器人侧状态（目标/路径/命令/到达停顿）；
        /// - 重绑 walkable；
        /// - 维持"自动已启用但无目标"的等待态。
        /// </summary>
        public void ResetToSingleRobot_NoLock(double initialMaxSpeed, EnumMoveDirection initialDirection)
        {
            // 1) 重置为单机器人
            SetRobotCount_NoLock(1, initialMaxSpeed, initialDirection);

            // 2) 清理引擎侧残留
            _control.ClearAllPause();
            _pathClaimManager.ClearRobotState(0);

            // 3) 清理机器人侧状态
            var r0 = _world.Robots[0];

            r0.Speed = 0.0;
            r0.Manager.Acc = 0.0;
            r0.Move.StopImmediately_NoLock();
            r0.Manager.ResetAutoCommands();

            r0.AutoNavigator.SetClaimedPathPrefix(null);
            r0.AutoNavigator.ClearGoal();

            // 清理到达停顿计时器
            r0.ResetArrivalPause();

            // 4) 重绑 walkable
            _walkableBinder.RebindDynamicWalkable_NoLock();

            // 5) 维持"自动已启用但无目标"的等待态
            if (!r0.AutoNavigator.IsEnabled)
            {
                r0.AutoNavigator.Enable();
            }
        }
    }
}