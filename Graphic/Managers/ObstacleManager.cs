using GridDemo.Maps;
using GridDemo.Models.Pathfinding;
using System;

namespace GridDemo.Models
{
    /// <summary>
    /// 障碍物管理器：
    /// 职责：
    /// - 封装对 <see cref="ObstacleMap"/> 的所有业务操作（切换、清空、快照）；
    /// - 管理障碍物编辑模式的进入/退出（暂停机器人、恢复自动导航）；
    /// - 提供地图文件的保存与载入（委托 <see cref="MapFileService"/>）。
    ///
    /// 设计原则：
    /// - 不持有锁：所有需要 lock 的方法由调用方（RobotEngine）在持锁后调用 _NoLock 版本，
    ///   或方法内部自行加锁（与 RobotEngine 使用同一把 _robotLock）。
    /// - 操作完成后自动触发路径重建，保证导航层与障碍物状态一致。
    /// </summary>
    internal sealed class ObstacleManager
    {
        private readonly RobotWorld _world;
        private readonly DynamicWalkableBinder _walkableBinder;

        /// <summary>
        /// 当前是否处于障碍物编辑模式。
        /// </summary>
        private bool _isEditing;

        public ObstacleManager(RobotWorld world, DynamicWalkableBinder walkableBinder)
        {
            _world = world ?? throw new ArgumentNullException(nameof(world));
            _walkableBinder = walkableBinder ?? throw new ArgumentNullException(nameof(walkableBinder));
        }

        /// <summary>
        /// 当前是否处于障碍物编辑模式。
        /// </summary>
        public bool IsEditing => _isEditing;

        /// <summary>
        /// 切换指定格子的障碍状态。
        /// 非编辑模式下会触发所有启用自动导航的机器人重建路径。
        /// </summary>
        /// <param name="p">网格坐标。</param>
        public void ToggleObstacle(GridPos p)
        {
            _world.ObstacleMap.Toggle(p);

            if (_isEditing)
            {
                return;
            }

            lock (_world.RobotLock)
            {
                RebuildAllAutoPath_NoLock();
            }
        }

        /// <summary>
        /// 清空所有障碍物。
        /// 非编辑模式下会触发所有启用自动导航的机器人重建路径。
        /// </summary>
        public void ClearObstacles()
        {
            _world.ObstacleMap.Clear();

            if (_isEditing)
            {
                return;
            }

            lock (_world.RobotLock)
            {
                RebuildAllAutoPath_NoLock();
            }
        }

        /// <summary>
        /// 进入障碍物编辑模式（不加锁版本，要求调用方已持有 RobotLock）：
        /// - 暂停所有机器人运动（禁用自动/手动，停止命令）。
        /// </summary>
        public void EnterEditMode_NoLock()
        {
            _isEditing = true;

            foreach (var r in _world.Robots)
            {
                r.Speed = 0.0;
                r.Manager.Acc = 0.0;
                r.Move.StopImmediately_NoLock();

                if (r.AutoNavigator.IsEnabled)
                {
                    r.AutoNavigator.Disable();
                }

                r.Manual.Disable();
                r.Manager.ResetAutoCommands();
            }
        }

        /// <summary>
        /// 退出障碍物编辑模式（不加锁版本，要求调用方已持有 RobotLock）：
        /// - 重绑动态 walkable；
        /// - 重新启用所有机器人的自动导航。
        /// </summary>
        public void ExitEditMode_NoLock()
        {
            _isEditing = false;

            // 先重绑 walkable（包含静态障碍），再 Enable()
            _walkableBinder.RebindDynamicWalkable_NoLock();

            foreach (var r in _world.Robots)
            {
                r.AutoNavigator.Enable();
                r.Manager.ResetAutoCommands();
            }
        }

        /// <summary>
        /// 触发所有已启用自动导航的机器人重建路径（不加锁版本）。
        /// </summary>
        private void RebuildAllAutoPath_NoLock()
        {
            foreach (var r in _world.Robots)
            {
                if (r.AutoNavigator.IsEnabled)
                {
                    r.AutoNavigator.RebuildPath();
                }
            }
        }
    }
}