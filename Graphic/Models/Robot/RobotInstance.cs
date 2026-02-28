using GridDemo.Maps;
using GridDemo.Models.Pathfinding;
using System;
using System.Collections.Generic;

namespace GridDemo.Models
{
    /// <summary>
    /// 单个机器人运行实例（多机器人引擎内部使用）。
    /// 将单机器人时期的 Manager/Move/Auto/Manual/状态聚合到一起，便于批量管理。
    /// 
    /// 职责：
    /// - 持有机器人的位置/速度/加速度等运动学状态；
    /// - 持有并协调四大子模块：
    ///   * <see cref="RobotManager"/>：调度指令（自动/手动统一入口）
    ///   * <see cref="RobotMove"/>：平移执行器（运动学积分 + 碰撞检测）
    ///   * <see cref="RobotAutoNavigator"/>：自动导航（寻路 + 路径指令生成）
    ///   * <see cref="RobotManual"/>：手动控制（键盘输入翻译为指令）
    /// - 提供线程安全的状态快照，供 UI 绘制层读取。
    /// </summary>
    internal sealed class RobotInstance
    {
        private readonly object _robotLock;
        private readonly ObstacleMap _obstacleMap;
        private readonly int _gridCount;
        private readonly double _cellSizeM;

        /// <summary>
        /// 创建一个机器人实例。
        /// </summary>
        public RobotInstance(
            int id,
            object robotLock,
            ObstacleMap obstacleMap,
            int gridCount,
            double cellSizeM,
            double dt,
            double worldWidthM,
            double worldHeightM,
            double initialMaxSpeed,
            EnumMoveDirection initialDirection,
            double initialX,
            double initialY,
            Func<Dictionary<int, int>> getGoalOwnerMap)
        {
            Id = id;
            _robotLock = robotLock ?? throw new ArgumentNullException(nameof(robotLock));
            _obstacleMap = obstacleMap ?? throw new ArgumentNullException(nameof(obstacleMap));
            _gridCount = gridCount;
            _cellSizeM = cellSizeM;

            X = initialX;
            Y = initialY;
            Speed = 0.0;
            Acc = 0.0;

            if (getGoalOwnerMap == null) throw new ArgumentNullException(nameof(getGoalOwnerMap));

            Manager = new RobotManager(
                acc: Acc,
                maxSpeed: initialMaxSpeed,
                direction: initialDirection);

            Move = new RobotMove(
                robotLock: _robotLock,
                getRobotX: () => X,
                setRobotX: v => X = v,
                getRobotY: () => Y,
                setRobotY: v => Y = v,
                getRobotSpeed: () => Speed,
                setRobotSpeed: v => Speed = v,
                robotManager: Manager,
                getWorldWidthM: () => worldWidthM,
                getWorldHeightM: () => worldHeightM,
                cellSizeM: cellSizeM,
                dt: dt,
                isWorldWalkable: (wx, wy) =>
                {
                    int gx = (int)Math.Floor(wx / _cellSizeM);
                    int gy = (int)Math.Floor(wy / _cellSizeM);

                    if (gx < 0 || gy < 0 || gx >= _gridCount || gy >= _gridCount)
                    {
                        return false;
                    }

                    return !_obstacleMap.IsObstacle(new GridPos(gx, gy));
                });

            AutoNavigator = new RobotAutoNavigator(
                robotLock: _robotLock,
                getRobotX: () => X,
                getRobotY: () => Y,
                setRobotSpeed: v => Speed = v,
                getWorldWidthM: () => worldWidthM,
                getWorldHeightM: () => worldHeightM,
                cellSizeM: cellSizeM,
                robotManager: Manager,
                robotId: id,
                gridCount: gridCount,
                getGoalOwnerMap: getGoalOwnerMap);

            Manual = new RobotManual(
                robotLock: _robotLock,
                robotManager: Manager);

            // 绑定运行时依赖：让 Manager 能调度 Move/Turn 执行器
            Manager.BindRuntime(
                robotLock: _robotLock,
                move: Move,
                turn: Move.TurnController,
                getForwardAcc: () => Acc);
        }

        // ---------------------------对外暴露的属性和方法--------------------------- //

        /// <summary> 机器人唯一标识（在引擎中按列表索引递增）。 </summary>
        public int Id { get; }

        /// <summary> 当前世界 X 坐标（米）。 </summary>
        public double X { get; set; }

        /// <summary> 当前世界 Y 坐标（米）。 </summary>
        public double Y { get; set; }

        /// <summary> 当前速度（米/秒）。 </summary>
        public double Speed { get; set; }

        /// <summary> 当前加速度（米/秒²），由控制模块写入，Move 执行器读取。 </summary>
        public double Acc { get; set; }

        /// <summary> 指令调度器（管理自动/手动指令的生成与派发）。 </summary>
        public RobotManager Manager { get; }

        /// <summary> 平移执行器（运动学积分 + 碰撞检测）。 </summary>
        public RobotMove Move { get; }

        /// <summary> 自动导航器（寻路 + 路径指令生成 + 格子锁抢占前缀管理）。 </summary>
        public RobotAutoNavigator AutoNavigator { get; }

        /// <summary> 手动控制器（键盘输入翻译为机器人指令）。 </summary>
        public RobotManual Manual { get; }

        /// <summary>
        /// 到达终点后的停顿剩余时间（秒）。
        /// 大于 0 表示正在停顿倒计时中；小于等于 0 表示无停顿状态。
        /// -1 表示尚未进入停顿（初始/非到达状态）。
        /// </summary>
        public double ArrivalPauseRemainS { get; set; } = -1.0;

        /// <summary>
        /// 重置到达停顿计时器（用于引擎重置/手动设目标/删除机器人等场景）。
        /// </summary>
        public void ResetArrivalPause()
        {
            ArrivalPauseRemainS = -1.0;
        }

        /// <summary>
        /// 生成状态快照，将当前状态封装为不可变结构体，供 UI 线程安全读取。
        /// </summary>
        public RobotStateSnapshot GetSnapshot()
        {
            return new RobotStateSnapshot(
                X: X,
                Y: Y,
                Speed: Speed,
                Acc: Acc,
                OrientationAngle: Manager.OrientationAngle,
                IsAutoMode: AutoNavigator.IsEnabled);
        }

        /// <summary>
        /// 根据当前世界坐标计算所在网格位置（已做越界夹紧）。
        /// 要求：调用方已持有 robotLock。
        /// </summary>
        public GridPos GetGridPos_NoLock()
        {
            int gx = (int)Math.Floor(X / _cellSizeM);
            int gy = (int)Math.Floor(Y / _cellSizeM);

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
                
            return new GridPos(gx, gy);
        }

        /// <summary>
        /// 将网格列索引转换为该格中心的世界 X 坐标（米）。
        /// </summary>
        public double GridToCenterX(int gx) => gx * _cellSizeM + _cellSizeM / 2.0;

        /// <summary>
        /// 将网格行索引转换为该格中心的世界 Y 坐标（米）。
        /// </summary>
        public double GridToCenterY(int gy) => gy * _cellSizeM + _cellSizeM / 2.0;
    }
}