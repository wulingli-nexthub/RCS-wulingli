using GridDemo.Models.Pathfinding;
using GridDemo.Models;
using System;
using System.Collections.Generic;
using GridDemo.Maps;

namespace GridDemo.Models
{
    /// <summary>
    /// DynamicWalkableBinder：
    /// 职责：
    /// - 每帧按当前机器人位置/目标，重绑定每个机器人的“可通行（walkable）”判定。
    /// - 对 AutoNavigator 提供基于 GridPos 的可行走判断；
    /// - 对 Move 模块提供基于世界坐标 (x,y) 的碰撞/穿越判定。
    ///
    /// 规则：
    /// - 静态障碍格不可通行；
    /// - 其它机器人的当前占用格不可通行；
    /// - 其它机器人的目标格（终点）对自己不可通行（避免多机器人抢终点）；
    /// - 自己的起点格（当前占用格）始终视为可通行（避免把自己困死）。
    /// </summary>
    internal sealed class DynamicWalkableBinder
    {
        private readonly RobotWorld _world;

        public DynamicWalkableBinder(RobotWorld world)
        {
            _world = world;
        }

        /// <summary>
        /// 重绑所有机器人的动态可行走判定。
        /// 要求：调用方已持有 RobotLock。
        /// 通常在每帧 Tick 开始阶段调用。
        /// </summary>
        public void RebindDynamicWalkable_NoLock()
        {
            var robots = _world.Robots;
            int gridCount = _world.GridCount;
            double cellSizeM = _world.CellSizeM;
            var obstacleMap = _world.ObstacleMap;

            var occupied = new HashSet<int>(robots.Count);
            var cellKeys = new int[robots.Count];

            // 1. 统计所有机器人当前占用格
            for (int i = 0; i < robots.Count; i++)
            {
                GridPos c = robots[i].GetGridPos_NoLock();
                int key = c.Y * gridCount + c.X;
                cellKeys[i] = key;
                occupied.Add(key);
            }

            // 2. 统计所有机器人目标格的“拥有者集合”
            //    注意：用 HashSet<int> 支持多个机器人目标重合场景。
            var goalOwnersByKey = new Dictionary<int, HashSet<int>>();
            for (int i = 0; i < robots.Count; i++)
            {
                var g = robots[i].AutoNavigator.GetGoalGridSnapshot();
                if (g.HasValue)
                {
                    GridPos gp = g.Value;
                    if (gp.X >= 0 && gp.Y >= 0 && gp.X < gridCount && gp.Y < gridCount)
                    {
                        int k = gp.Y * gridCount + gp.X;
                        if (!goalOwnersByKey.TryGetValue(k, out var owners))
                        {
                            owners = new HashSet<int>();
                            goalOwnersByKey.Add(k, owners);
                        }
                        owners.Add(robots[i].Id);
                    }
                }
            }

            // 3. 为每台机器人设置 AutoNavigator 和 Move 的 walkable 回调
            for (int i = 0; i < robots.Count; i++)
            {
                RobotInstance me = robots[i];
                int myKey = cellKeys[i];
                int myId = me.Id;

                // A* 寻路用的 GridPos 判定
                me.AutoNavigator.SetIsWalkableProvider(p =>
                {
                    return !obstacleMap.IsObstacle(p);
                });

                // 运动学 / 碰撞层使用的世界坐标判定
                me.Move.SetIsWorldWalkableProvider((wx, wy) =>
                {
                    int gx = (int)Math.Floor(wx / cellSizeM);
                    int gy = (int)Math.Floor(wy / cellSizeM);

                    if (gx < 0 || gy < 0 || gx >= gridCount || gy >= gridCount)
                    {
                        return false;
                    }

                    var p = new GridPos(gx, gy);
                    if (obstacleMap.IsObstacle(p))
                    {
                        return false;
                    }

                    int key = p.Y * gridCount + p.X;

                    // 自己所在格始终可行走
                    if (key == myKey)
                    {
                        return true;
                    }

                    // 注意：Move 层不把“其它机器人目标格”当成障碍，只关心当前占用格
                    return !occupied.Contains(key);
                });
            }
        }
    }
}