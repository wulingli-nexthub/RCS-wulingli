using Graphic.Maps;
using GridDemo.RobotModels.Pathfinding;
using System;
using System.Collections.Generic;

namespace GridDemo.Robots
{
    /// <summary>
    /// 网格“格子锁/抢占板”（Claim Board）：用于多机器人在同一网格世界中做互斥占用。
    /// 
    /// 设计目的：
    /// - 防止多机器人同时进入同一格子导致“穿模/重叠/互堵不可控”。
    /// - 引擎层先按每台机器人的完整路径尝试抢占一段“可用前缀”（claim prefix），
    ///   然后导航器只允许沿已抢占前缀下发移动指令（抢不到就停/等待）。
    /// 
    /// 数据结构：
    /// - <see cref="_claimedBy"/>：cellKey -> robotId，快速判断某个格子当前归谁占用。
    /// - <see cref="_claimedCellsByRobotId"/>：robotId -> claimed cells（按路径顺序），用于释放/渲染/回溯。
    /// 
    /// 注意：
    /// - 本类本身不加锁，假设由上层（<see cref="RobotEngine"/>）在 <c>_robotLock</c> 下调用，
    ///   以保证与导航器、运动学、UI 快照之间的一致性。
    /// - 障碍格不允许抢占（由 <see cref="ObstacleMap"/> 判定）。
    /// </summary>
    internal sealed class GridCellClaimBoard
    {
        // 网格尺寸（宽高一致）：用于把 (x,y) 映射为 cellKey = y*W + x，以及做越界判断
        private readonly int _gridCount;

        // 静态障碍地图：障碍格不能抢占
        private readonly ObstacleMap _obstacleMap;

        // cellKey -> robotId：某个格子当前被哪个机器人锁住（占用）
        private readonly Dictionary<int, int> _claimedBy = new Dictionary<int, int>();

        // robotId -> claimed cells：该机器人当前持有（锁住）的格子序列（按路径从近到远记录）
        private readonly Dictionary<int, List<GridPos>> _claimedCellsByRobotId = new Dictionary<int, List<GridPos>>();

        public GridCellClaimBoard(int gridCount, ObstacleMap obstacleMap)
        {
            _gridCount = gridCount;
            _obstacleMap = obstacleMap ?? throw new ArgumentNullException(nameof(obstacleMap));
        }

        /// <summary>
        /// 释放某个机器人当前占用的全部格子锁。
        /// 常见触发场景：
        /// - 机器人切换目标（旧路径锁不能残留）
        /// - 禁用自动导航（手动模式不再遵循路径）
        /// - 删除机器人/重置仿真
        /// </summary>
        public void ReleaseAllByRobot(int robotId)
        {
            // 从 robotId -> list 找到该机器人曾经主张的所有格子
            List<GridPos> list;
            if (!_claimedCellsByRobotId.TryGetValue(robotId, out list) || list.Count == 0)
            {
                // 容错：没有 list 也确保移除映射
                _claimedCellsByRobotId.Remove(robotId);
                return;
            }

            // 遍历该机器人持有的格子，把 _claimedBy 中归属为 robotId 的条目删掉
            for (int i = 0; i < list.Count; i++)
            {
                GridPos cell = list[i];
                int key = cell.Y * _gridCount + cell.X;

                int owner;
                if (_claimedBy.TryGetValue(key, out owner) && owner == robotId)
                {
                    _claimedBy.Remove(key);
                }
            }

            // 最后移除 robotId -> list
            _claimedCellsByRobotId.Remove(robotId);
        }

        /// <summary>
        /// 整段锁定到终点（或尽可能长的前缀）：
        /// - 从 path 的 startIndex 开始逐格尝试抢占；
        /// - 一旦遇到“被别人占用”的格子，立即停止；
        /// - 返回“本次成功抢占的路径前缀”，并写入内部状态。
        /// 
        /// 说明：
        /// - 本方法会先释放该 robotId 的旧锁（避免旧路径残留影响新路径）。
        /// - 障碍格/越界格会被跳过（不抢占）。
        /// </summary>
        public List<GridPos> ClaimPathToGoalOrPrefix(int robotId, List<GridPos> path, int startIndex)
        {
            // 先清掉该机器人旧锁，避免“旧路径残留锁”影响新路径
            ReleaseAllByRobot(robotId);

            // 无路径则不抢占
            if (path == null || path.Count == 0)
            {
                return null;
            }

            // 规范化 startIndex，避免越界
            if (startIndex < 0)
            {
                startIndex = 0;
            }
            if (startIndex >= path.Count)
            {
                startIndex = Math.Max(0, path.Count - 1);
            }

            // claimed：本次成功抢到的格子（按路径顺序）
            var claimed = new List<GridPos>(Math.Max(4, path.Count - startIndex));

            // 逐格抢占：能抢到就写入 _claimedBy，抢不到（被别人占）立刻停止并返回前缀
            for (int i = startIndex; i < path.Count; i++)
            {
                GridPos p = path[i];

                // 越界格：跳过（不抢占、不计入 claimed）
                if (p.X < 0 || p.Y < 0 || p.X >= _gridCount || p.Y >= _gridCount)
                {
                    continue;
                }

                // 障碍格：跳过（不允许抢占）
                if (_obstacleMap.IsObstacle(p))
                {
                    continue;
                }

                int key = p.Y * _gridCount + p.X;

                int owner;
                if (_claimedBy.TryGetValue(key, out owner))
                {
                    if (owner != robotId)
                    {
                        // 被其它机器人占用：只能拿到前缀，立即停止
                        break;
                    }

                    // 已经是自己占用（理论上不应该，因为上面 ReleaseAllByRobot 了），做容错
                    claimed.Add(p);
                    continue;
                }

                // 未被占用：写入占用关系，并记录到 claimed
                _claimedBy[key] = robotId;
                claimed.Add(p);
            }

            // 若抢到任何格子，则保存 robotId -> claimed 列表，并返回副本（避免外部修改内部状态）
            if (claimed.Count > 0)
            {
                _claimedCellsByRobotId[robotId] = claimed;
                return new List<GridPos>(claimed);
            }

            // 没抢到任何格子：确保移除 robotId 的记录，并返回 null（上层应等待/停车）
            _claimedCellsByRobotId.Remove(robotId);
            return null;
        }

        /// <summary>
        /// 释放“单个格子锁”：
        /// - 若该格在 _claimedBy 中归属为 robotId，则移除；
        /// - 同时从 robotId 的 claimed 列表中删除该格（保持渲染/快照一致）。
        /// 
        /// 典型用途：
        /// - 机器人“边走边释放”：完全走出某格后释放该格，让后车能更快抢到。
        /// </summary>
        public void ReleaseCell(int robotId, GridPos cell)
        {
            int key = cell.Y * _gridCount + cell.X;

            // 从全局占用表移除（仅当 owner 是自己）
            int owner;
            if (_claimedBy.TryGetValue(key, out owner) && owner == robotId)
            {
                _claimedBy.Remove(key);
            }

            // 从 robotId 的路径占用列表中移除该格
            List<GridPos> list;
            if (_claimedCellsByRobotId.TryGetValue(robotId, out list))
            {
                for (int i = list.Count - 1; i >= 0; i--)
                {
                    if (list[i].Equals(cell))
                    {
                        list.RemoveAt(i);
                        break;
                    }
                }

                // 列表清空则移除该机器人记录
                if (list.Count == 0)
                {
                    _claimedCellsByRobotId.Remove(robotId);
                }
            }
        }

        /// <summary>
        /// 获取“抢占格子”快照（robotId -> claimed cells）。
        /// 供渲染层使用（画出每台机器人的抢占前缀/锁定格）。
        /// 
        /// 注意：返回的是深拷贝副本，避免外部修改内部集合。
        /// </summary>
        public Dictionary<int, List<GridPos>> GetClaimedCellsSnapshot()
        {
            var snapshot = new Dictionary<int, List<GridPos>>(_claimedCellsByRobotId.Count);

            foreach (var kv in _claimedCellsByRobotId)
            {
                snapshot[kv.Key] = new List<GridPos>(kv.Value);
            }

            return snapshot;
        }
    }
}