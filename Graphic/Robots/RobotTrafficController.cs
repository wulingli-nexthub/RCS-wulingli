using Graphic.Maps;
using GridDemo.RobotModels.Pathfinding;
using System;
using System.Collections.Generic;

namespace GridDemo.Robots
{
    /// <summary>
    /// 交通规则 + 碰撞控制（从 RobotEngine 中拆分）
    /// - 仅负责：意图预测、冲突判定、让步决策、冷却计数
    /// - 不负责：机器人运动学更新、导航重建、Stop 等动作执行（由调用方决定如何应用决策）
    /// </summary>
    internal sealed class RobotTrafficController
    {
        // --- 碰撞让步冷却：避免每帧 Stop + Rebuild 导致抖动 ---
        private const int YieldCooldownFrames = 12; // 冷却帧数：12 帧 * 20ms ≈ 240ms（防止频繁让步抖动）
        private readonly Dictionary<int, int> _yieldCooldownTicks = new Dictionary<int, int>(); // robotId -> 冷却剩余帧数

        private readonly int _gridCount;
        private readonly double _cellSizeM;
        private readonly double _dt;
        private readonly ObstacleMap _obstacleMap;

        public RobotTrafficController(int gridCount, double cellSizeM, double dt, ObstacleMap obstacleMap)
        {
            _gridCount = gridCount;
            _cellSizeM = cellSizeM;
            _dt = dt;
            _obstacleMap = obstacleMap;
        }

        /// <summary>
        /// 每帧递减冷却计数（要求调用方已持有与机器人集合一致的锁）
        /// </summary>
        public void TickCooldown_NoLock()
        {
            if (_yieldCooldownTicks.Count <= 0)
            {
                return;
            }

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

        /// <summary>
        /// 基于“下一格意图”检测冲突并产出“让步/通行”决策列表（要求调用方已持有锁）
        /// </summary>
        /// <param name="robots">机器人列表（RobotEngine 内部的同一实例）</param>
        /// <returns>冲突对决策列表</returns>
        public List<YieldDecision> BuildYieldDecisions_NoLock(List<RobotInstance> robots)
        {
            if (robots == null || robots.Count <= 1)
            {
                return new List<YieldDecision>(0);
            }

            var decisions = new List<YieldDecision>();

            var cur = new GridPos[robots.Count];
            var next = new GridPos[robots.Count];

            for (int i = 0; i < robots.Count; i++)
            {
                RobotInstance r = robots[i];
                cur[i] = r.GetGridPos_NoLock();
                next[i] = GetIntendedNextCell_NoLock(r, cur[i]);
            }

            for (int i = 0; i < robots.Count; i++)
            {
                for (int j = i + 1; j < robots.Count; j++)
                {
                    bool iStationary = next[i].Equals(cur[i]);
                    bool jStationary = next[j].Equals(cur[j]);
                    if (iStationary && jStationary)
                    {
                        continue;
                    }

                    bool sameTarget = next[i].Equals(next[j]);
                    bool swap = next[i].Equals(cur[j]) && next[j].Equals(cur[i]);
                    bool enterOther = next[i].Equals(cur[j]) || next[j].Equals(cur[i]);

                    if (!sameTarget && !swap && !enterOther)
                    {
                        continue;
                    }

                    RobotInstance a = robots[i];
                    RobotInstance b = robots[j];

                    if (IsInYieldCooldown_NoLock(a.Id) && IsInYieldCooldown_NoLock(b.Id))
                    {
                        continue;
                    }

                    RobotInstance yield;
                    RobotInstance go;

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
                        bool aTurning = IsTurning_NoLock(a);
                        bool bTurning = IsTurning_NoLock(b);
                        bool aMoving = IsMoving_NoLock(a);
                        bool bMoving = IsMoving_NoLock(b);

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
                            EnumMoveDirection da = a.Manager.Direction;
                            EnumMoveDirection db = b.Manager.Direction;

                            if (IsOppositeDirection(da, db))
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
                            else if (IsSameDirection(da, db))
                            {
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
                    }

                    decisions.Add(new YieldDecision(yieldId: yield.Id, goId: go.Id));
                }
            }

            return decisions;
        }

        /// <summary>
        /// 标记机器人进入让步冷却期（要求调用方已持有锁）
        /// </summary>
        public void EnterYieldCooldown_NoLock(int robotId)
        {
            _yieldCooldownTicks[robotId] = YieldCooldownFrames;
        }

        private bool IsInYieldCooldown_NoLock(int robotId)
        {
            int t;
            return _yieldCooldownTicks.TryGetValue(robotId, out t) && t > 0;
        }

        private bool IsMoving_NoLock(RobotInstance r)
        {
            return r.Speed > 1e-3;
        }

        private bool IsTurning_NoLock(RobotInstance r)
        {
            return r.Manager.IsTurning;
        }

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

        private GridPos GetIntendedNextCell_NoLock(RobotInstance r, GridPos curCell)
        {
            GridPos target = curCell;

            if (!r.AutoNavigator.IsEnabled)
            {
                return target;
            }

            double v = r.Speed;
            double move = v * _dt;
            if (move < _cellSizeM * 0.15)
            {
                move = _cellSizeM * 0.15;
            }

            int dx = 0;
            int dy = 0;

            switch (r.Manager.Direction)
            {
                case EnumMoveDirection.Right:
                    dx = +1;
                    break;
                case EnumMoveDirection.Left:
                    dx = -1;
                    break;
                case EnumMoveDirection.Down:
                    dy = +1;
                    break;
                case EnumMoveDirection.Up:
                    dy = -1;
                    break;
            }

            double curCenterX = curCell.X * _cellSizeM + _cellSizeM / 2.0;
            double curCenterY = curCell.Y * _cellSizeM + _cellSizeM / 2.0;

            double predX = curCenterX + dx * move;
            double predY = curCenterY + dy * move;

            int gx = (int)Math.Floor(predX / _cellSizeM);
            int gy = (int)Math.Floor(predY / _cellSizeM);

            if (gx < 0)
            {
                gx = 0;
            }
            if (gy < 0)
            {
                gy = 0;
            }
            if (gx >= _gridCount)
            {
                gx = _gridCount - 1;
            }
            if (gy >= _gridCount)
            {
                gy = _gridCount - 1;
            }

            target = new GridPos(gx, gy);
            return target;
        }

        /// <summary>
        /// 让步/通行决策
        /// </summary>
        internal readonly struct YieldDecision
        {
            public YieldDecision(int yieldId, int goId)
            {
                YieldId = yieldId;
                GoId = goId;
            }

            public int YieldId { get; }
            public int GoId { get; }
        }
    }
}