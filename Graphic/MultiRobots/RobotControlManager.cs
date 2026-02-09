using GridDemo.Robots;
using GridDemo.RobotModels.Pathfinding;
using System.Collections.Generic;

namespace GridDemo.MultiRobots
{
    /// <summary>
    /// RobotControlManager：
    /// 职责：
    /// - 管理自动/手动控制模式的切换；
    /// - 统一管理“单机暂停”状态；
    /// - 处理手动输入（W/A/D 键）并转化为 Manual 控制模块的输入；
    /// - 提供全局设置：寻路算法、加速度、最大速度等。
    ///
    /// 注意：
    /// - RobotWorld.RobotLock 由外部（RobotEngine）负责加锁；
    /// - 公开方法内部再自行加锁，保持使用习惯与 RobotEngine 一致。
    /// </summary>
    internal sealed class RobotControlManager
    {
        private readonly RobotWorld _world;

        // 单机暂停集合：在 Tick 中若包含该 Id，则不派发任何指令，强制停车。
        private readonly HashSet<int> _pausedRobotIds = new HashSet<int>();

        public RobotControlManager(RobotWorld world)
        {
            _world = world;
        }

        public HashSet<int> PausedRobotIds => _pausedRobotIds;

        /// <summary>
        /// 当前全局寻路算法：
        /// - get：读取选中机器人的算法作为“当前配置”；
        /// - set：同步到所有机器人。
        /// </summary>
        public EnumPathfindingAlgorithm Algorithm
        {
            get
            {
                lock (_world.RobotLock)
                {
                    RobotInstance r = _world.GetSelectedRobot_NoLock();
                    return r?.AutoNavigator.Algorithm ?? EnumPathfindingAlgorithm.AStar;
                }
            }
            set
            {
                lock (_world.RobotLock)
                {
                    foreach (var r in _world.Robots)
                        r.AutoNavigator.Algorithm = value;
                }
            }
        }

        /// <summary> 选中机器人是否启用自动导航。 </summary>
        public bool AutoEnabled
        {
            get
            {
                lock (_world.RobotLock)
                {
                    int id = _world.SelectedRobotId;
                    if (id < 0 || id >= _world.Robots.Count)
                        return false;
                    return _world.Robots[id].AutoNavigator.IsEnabled;
                }
            }
        }

        /// <summary>
        /// 将“选中机器人”切换到自动模式：
        /// - 禁用手动；
        /// - 设置 RobotManager 为 Auto 模式；
        /// - 启用 AutoNavigator 并重置自动命令；
        /// - 将朝向对齐到当前运动方向。
        /// </summary>
        public void EnableAutoForSelected()
        {
            lock (_world.RobotLock)
            {
                int id = _world.SelectedRobotId;
                if (id < 0 || id >= _world.Robots.Count)
                    return;

                RobotInstance r = _world.Robots[id];
                r.Manual.Disable();
                r.Manager.SetMode(EnumRobotControlMode.Auto);

                r.AutoNavigator.Enable();
                r.Manager.ResetAutoCommands();
                r.Manager.AlignOrientationToDirectionWithTurn();
            }
        }

        /// <summary>
        /// 将“选中机器人”切换到手动模式。
        /// 注意：需要调用方传入 PathClaimManager，用于释放格子锁。
        /// 步骤：
        /// - 禁用自动导航；
        /// - 释放该机器人的所有格子锁/路径前缀；
        /// - 硬停并清理自动命令；
        /// - 启用 Manual 控制，并移除单机暂停状态。
        /// </summary>
        public void EnableManualForSelected(PathClaimManager claimMgr)
        {
            lock (_world.RobotLock)
            {
                int id = _world.SelectedRobotId;
                if (id < 0 || id >= _world.Robots.Count)
                    return;

                RobotInstance r = _world.Robots[id];

                if (r.AutoNavigator.IsEnabled)
                    r.AutoNavigator.Disable();

                claimMgr.ClaimBoard.ReleaseAllByRobot(r.Id);
                claimMgr.ClearRobotState(r.Id);
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
        /// 设置所有机器人的前进加速度。
        /// 设置后重置自动命令，使下一条 MoveDistance 使用新加速度。
        /// </summary>
        public void SetForwardAcc(double acc)
        {
            lock (_world.RobotLock)
            {
                foreach (var r in _world.Robots)
                {
                    r.Acc = acc;
                    r.Manager.ResetAutoCommands();
                }
            }
        }

        /// <summary> 设置所有机器人的最大速度。 </summary>
        public void SetMaxSpeed(double vmax)
        {
            lock (_world.RobotLock)
            {
                foreach (var r in _world.Robots)
                    r.Manager.MaxSpeed = vmax;
            }
        }

        /// <summary> 手动控制：前进键（W）按下/抬起。 </summary>
        public void ManualForwardKey(bool down)
        {
            lock (_world.RobotLock)
            {
                int id = _world.SelectedRobotId;
                if (id < 0 || id >= _world.Robots.Count)
                    return;

                var r = _world.Robots[id];
                if (!r.Manual.IsEnabled)
                    return;

                r.Manual.InputForwardKey(down);
                _pausedRobotIds.Remove(r.Id);
            }
        }

        /// <summary> 手动控制：左转键（A）按下/抬起。 </summary>
        public void ManualTurnLeftKey(bool down)
        {
            lock (_world.RobotLock)
            {
                int id = _world.SelectedRobotId;
                if (id < 0 || id >= _world.Robots.Count)
                    return;

                var r = _world.Robots[id];
                if (!r.Manual.IsEnabled)
                    return;

                r.Manual.InputTurnLeftKey(down);
                _pausedRobotIds.Remove(r.Id);
            }
        }

        /// <summary> 手动控制：右转键（D）按下/抬起。 </summary>
        public void ManualTurnRightKey(bool down)
        {
            lock (_world.RobotLock)
            {
                int id = _world.SelectedRobotId;
                if (id < 0 || id >= _world.Robots.Count)
                    return;

                var r = _world.Robots[id];
                if (!r.Manual.IsEnabled)
                    return;

                r.Manual.InputTurnRightKey(down);
                _pausedRobotIds.Remove(r.Id);
            }
        }

        /// <summary> 将指定机器人加入“单机暂停”集合。 </summary>
        public void PauseRobot(int robotId) => _pausedRobotIds.Add(robotId);

        /// <summary> 将指定机器人从“单机暂停”集合移除。 </summary>
        public void ResumeRobot(int robotId) => _pausedRobotIds.Remove(robotId);

        /// <summary> 判断机器人是否处于“单机暂停”。 </summary>
        public bool IsPaused(int robotId) => _pausedRobotIds.Contains(robotId);

        /// <summary> 清空所有单机暂停状态。 </summary>
        public void ClearAllPause() => _pausedRobotIds.Clear();
    }
}