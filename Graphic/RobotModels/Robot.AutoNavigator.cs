using Graphic.RobotModels.Pathfinding;
using Graphic.RobotRuns;
using System;
using System.Collections.Generic;

namespace Graphic.RobotModels
{
    /// <summary>
    /// 机器人自动导航器：
    /// - 将世界坐标系离散为网格（cell），使用寻路算法生成从当前位置到目标格子的路径；
    /// - 每帧根据当前路径点计算“应朝向的移动方向”，必要时请求机器人原地转向；
    /// - 转向完成后继续按加速度参数前进，直到到达最后一个路径点。
    /// </summary>
    internal sealed class RobotAutoNavigator
    {
        private readonly object _robotLock;
        private readonly Func<double> _getRobotX;     // 获取机器人当前 X 坐标（世界坐标系）
        private readonly Func<double> _getRobotY;    // 获取机器人当前 Y 坐标（世界坐标系）
        private readonly Action<double> _setRobotSpeed;        // 直接设置机器人速度（通常用于到达/禁用时强制刹停）

        private readonly Func<double> _getWorldWidthM;          // 获取世界宽度（米）
        private readonly Func<double> _getWorldHeightM;         // 获取世界高度（米）

        private readonly double _cellSizeM;           // 网格单元大小（米），用于将世界坐标系离散化为网格
        private readonly Robot _robot;             // 机器人实例

        private const double ArriveEpsilonM = 0.05;           // 到达路径点的容忍距离（米），用于判断是否到达某个格子中心

        private readonly List<GridPos> _path = new List<GridPos>();           // 当前路径点列表（网格坐标系），使用时需转换为世界坐标（取格子中心点）。
        private int _pathIndex;           // 当前路径点索引，指向 _path 中的下一个目标点。

        private EnumPathfindingAlgorithm _algorithm = EnumPathfindingAlgorithm.AStar;          // 寻路算法类型，默认为 A* 算法。
        private GridPos? _goal;

        public RobotAutoNavigator(
            object robotLock,
            Func<double> getRobotX,
            Func<double> getRobotY,
            Action<double> setRobotSpeed,
            Func<double> getWorldWidthM,
            Func<double> getWorldHeightM,
            double cellSizeM,
            Robot robot)
        {
            _robotLock = robotLock ?? throw new ArgumentNullException(nameof(robotLock));
            _getRobotX = getRobotX ?? throw new ArgumentNullException(nameof(getRobotX));
            _getRobotY = getRobotY ?? throw new ArgumentNullException(nameof(getRobotY));
            _setRobotSpeed = setRobotSpeed ?? throw new ArgumentNullException(nameof(setRobotSpeed));
            _getWorldWidthM = getWorldWidthM ?? throw new ArgumentNullException(nameof(getWorldWidthM));
            _getWorldHeightM = getWorldHeightM ?? throw new ArgumentNullException(nameof(getWorldHeightM));
            _cellSizeM = cellSizeM;
            _robot = robot ?? throw new ArgumentNullException(nameof(robot));
        }

        public bool IsEnabled { get; private set; }           // 是否启用自动导航器

        /// <summary>
        /// 当前寻路算法。设置新算法会清空现有路径并重置索引，等待下一次重建路径。
        /// </summary>
        public EnumPathfindingAlgorithm Algorithm
        {
            get { return _algorithm; }
            set
            {
                lock (_robotLock)
                {
                    if (_algorithm == value)
                    {
                        return;
                    }

                    _algorithm = value;

                    // 算法变化后，清空路径，等待下一帧/或外部调用 RebuildPath 立即重算
                    _path.Clear();
                    _pathIndex = 0;
                }
            }
        }

        public void SetGoal(GridPos goal, bool rebuildIfEnabled)
        {
            lock (_robotLock)
            {
                _goal = goal;

                _path.Clear();
                _pathIndex = 0;

                if (rebuildIfEnabled && IsEnabled)
                {
                    RebuildPath_NoLock();
                }
            }
        }

        public (double X, double Y)? GetGoalWorldPointSnapshot()
        {
            lock (_robotLock)
            {
                if (!_goal.HasValue)
                {
                    return null;
                }

                GridPos g = _goal.Value;
                return (GridToCenterWorldX(g.X), GridToCenterWorldY(g.Y));
            }
        }

        /// <summary>
        /// 立即重建路径。
        /// </summary>
        public void RebuildPath()
        {
            lock (_robotLock)
            {
                RebuildPath_NoLock();
            }
        }

        /// <summary>
        /// 启用自动导航：
        /// - 设置启用标志；
        /// - 将“前进按键”置为按下（让运动系统进入可前进状态）；
        /// - 根据当前位置重建路径；
        /// - 同步当前朝向角（避免刚启用时朝向状态异常）。
        /// </summary>
        public void Enable()
        {
            lock (_robotLock)
            {
                IsEnabled = true;

                _robot.IsForwardKeyDown = true;

                RebuildPath_NoLock();

                double angle = Robot.DirectionToAngle(_robot.Direction);
                _robot.OrientationAngle = angle;
                _robot.TargetOrientationAngle = angle;
            }
        }

        /// <summary>
        /// 禁用自动导航：
        /// - 清空路径并复位索引；
        /// - 取消前进、清零加速度并强制速度为 0。
        /// </summary>
        public void Disable()
        {
            lock (_robotLock)
            {
                IsEnabled = false;

                _path.Clear();
                _pathIndex = 0;

                _robot.IsForwardKeyDown = false;
                _robot.Acc = 0.0;
                _setRobotSpeed(0.0);
            }
        }

        /// <summary>
        /// 获取路径点的世界坐标快照（用于绘制/调试）。
        /// 返回的是“格子中心点”的世界坐标序列（单位：米）。
        /// </summary>
        public List<(double X, double Y)> GetPathWorldPointsSnapshot()
        {
            lock (_robotLock)
            {
                var points = new List<(double X, double Y)>(_path.Count);
                for (int i = 0; i < _path.Count; i++)
                {
                    GridPos p = _path[i];
                    points.Add((GridToCenterWorldX(p.X), GridToCenterWorldY(p.Y)));
                }

                return points;
            }
        }

        /// <summary>
        /// 获取当前路径索引快照（用于绘制“当前追踪点”或调试）。
        /// </summary>
        public int GetPathIndexSnapshot()
        {
            lock (_robotLock)
            {
                return _pathIndex;
            }
        }

        /// <summary>
        /// 计算本帧自动运动状态：
        /// - 若未启用：返回 Disabled；
        /// - 若无路径：尝试重建路径；仍无路径则刹停；
        /// - 若到达当前路径点：推进索引到下一个点；
        /// - 若需要调整方向：请求转向（原地转），转完再走；
        /// - 否则：按传入的加速度函数返回前进状态。
        /// </summary>
        /// <param name="getForwardAcc">获取本帧前进加速度（由外部策略决定）。</param>
        public RobotAutoMotionState GetAutoMotionState(Func<double> getForwardAcc)
        {
            if (getForwardAcc == null) throw new ArgumentNullException(nameof(getForwardAcc));

            lock (_robotLock)
            {
                if (!IsEnabled)
                {
                    return RobotAutoMotionState.Disabled;
                }

                if (_path.Count == 0)            // 如果没有路径，则尝试重建路径（可能是第一次启用，或之前的路径已被清空）
                {
                    RebuildPath_NoLock();
                }

                if (_path.Count == 0)            // 仍然没有可用路径：停住并维持“启用中”的状态，防止外部逻辑误判
                {
                    _robot.IsForwardKeyDown = false;
                    _robot.Acc = 0.0;
                    _setRobotSpeed(0.0);

                    return new RobotAutoMotionState(
                        enabled: true,
                        direction: _robot.Direction,
                        acc: 0.0,
                        suppressEdgeTurning: true,
                        clampOnBounds: true,
                        requestTurnLeft: false,
                        requestTurnToDirection: null);
                }

                double x = _getRobotX();
                double y = _getRobotY();

                AdvanceWaypointIfArrived_NoLock(x, y);             // 推进路径索引到下一个点（如果已到达当前点）

                if (_pathIndex >= _path.Count)               // 如果已到达最后一个路径点（或超出范围），停止运动
                {
                    _robot.IsForwardKeyDown = false;
                    _robot.Acc = 0.0;
                    _setRobotSpeed(0.0);

                    return new RobotAutoMotionState(
                        enabled: true,
                        direction: _robot.Direction,
                        acc: 0.0,
                        suppressEdgeTurning: true,
                        clampOnBounds: true,
                        requestTurnLeft: false,
                        requestTurnToDirection: null);
                }

                GridPos next = _path[_pathIndex];                      // 获取下一个目标路径点（网格坐标）
                double targetX = GridToCenterWorldX(next.X);
                double targetY = GridToCenterWorldY(next.Y);

                EnumMoveDirection dir = ChooseDirectionToTarget(x, y, targetX, targetY);      // 计算应朝向的移动方向

                double acc = getForwardAcc();            // 获取本帧前进加速度（由外部策略决定）

                // 需要转向：触发转向动画（原地转），转完再继续前进
                if (!_robot.IsTurning && dir != _robot.Direction)
                {
                    _robot.IsForwardKeyDown = true;
                    _robot.Acc = 0.0;

                    return new RobotAutoMotionState(
                        enabled: true,
                        direction: _robot.Direction,
                        acc: 0.0,
                        suppressEdgeTurning: true,
                        clampOnBounds: true,
                        requestTurnLeft: false,
                        requestTurnToDirection: dir);
                }

                // 不需要转向：正常前进（朝向由 RobotTurn 保持/或已对齐）
                _robot.IsForwardKeyDown = true;
                _robot.Acc = acc;

                return new RobotAutoMotionState(
                    enabled: true,
                    direction: _robot.Direction,
                    acc: acc,
                    suppressEdgeTurning: true,
                    clampOnBounds: true,
                    requestTurnLeft: false,
                    requestTurnToDirection: null);
            }
        }

        /// <summary>
        /// 重建路径（不加锁版，调用方必须已持有 <see cref="_robotLock"/>）。
        /// 当前实现以“右下角（gridW-1, gridH-1）”作为目标点。
        /// </summary>
        private void RebuildPath_NoLock()
        {
            double worldWidth = _getWorldWidthM();
            double worldHeight = _getWorldHeightM();

            // 将世界尺寸换算为网格尺寸（至少 1x1）
            int gridW = Math.Max(1, (int)Math.Round(worldWidth / _cellSizeM));
            int gridH = Math.Max(1, (int)Math.Round(worldHeight / _cellSizeM));

            GridPos start = WorldToGrid(_getRobotX(), _getRobotY(), gridW, gridH);            // 获取当前机器人位置的网格坐标

            // 未设置目的地：不生成路径（保持停住/等待外部 SetGoal）
            if (!_goal.HasValue)
            {
                _path.Clear();
                _pathIndex = 0;
                return;
            }

            GridPos goal = _goal.Value;

            Func<GridPos, bool> isWalkable = p => true;                 // 假设所有格子都是可通行的（可根据实际情况修改）

            List<GridPos> path = GridPathfinder.FindPath(        // 使用寻路算法计算路径
                width: gridW,
                height: gridH,
                start: start,
                goal: goal,
                isWalkable: isWalkable,
                algorithm: _algorithm);

            _path.Clear();              // 清空现有路径
            _path.AddRange(path);

            _pathIndex = 0;           // 默认从第 0 个点开始追踪；若路径第 0 个点就是 start，则从下一个点开始
            if (_path.Count > 0 && _path[0].Equals(start))
            {
                _pathIndex = Math.Min(1, _path.Count);
            }
        }

        /// <summary>
        /// 如果机器人已经到达当前路径点，则推进到下一个路径点。
        /// 使用 while 的原因：机器人可能在一帧内跨过多个格子中心点（或初始就落在多个点容差内）。
        /// </summary>
        private void AdvanceWaypointIfArrived_NoLock(double robotX, double robotY)
        {
            while (_pathIndex < _path.Count)
            {
                GridPos p = _path[_pathIndex];
                double tx = GridToCenterWorldX(p.X);
                double ty = GridToCenterWorldY(p.Y);

                bool arriveX = Math.Abs(robotX - tx) <= ArriveEpsilonM;
                bool arriveY = Math.Abs(robotY - ty) <= ArriveEpsilonM;

                if (!(arriveX && arriveY))
                {
                    break;
                }

                _pathIndex++;
            }
        }

        /// <summary>
        /// 将世界坐标转换为网格坐标（格子中心点），并确保不超出边界。
        /// </summary>
        private GridPos WorldToGrid(double wx, double wy, int gridW, int gridH)
        {
            int gx = (int)Math.Floor(wx / _cellSizeM);
            int gy = (int)Math.Floor(wy / _cellSizeM);

            if (gx < 0) gx = 0;
            if (gy < 0) gy = 0;
            if (gx >= gridW) gx = gridW - 1;
            if (gy >= gridH) gy = gridH - 1;

            return new GridPos(gx, gy);
        }

        private double GridToCenterWorldX(int gx) => gx * _cellSizeM + _cellSizeM / 2.0;
        private double GridToCenterWorldY(int gy) => gy * _cellSizeM + _cellSizeM / 2.0;

        /// <summary>
        /// 根据目标点相对当前位置的偏移，选择一个离散移动方向：
        /// - |dx| >= |dy| 时优先水平移动（左/右）
        /// - 否则优先垂直移动（上/下）
        /// </summary>
        private static EnumMoveDirection ChooseDirectionToTarget(double x, double y, double tx, double ty)
        {
            double dx = tx - x;
            double dy = ty - y;

            if (Math.Abs(dx) >= Math.Abs(dy))
            {
                return dx >= 0 ? EnumMoveDirection.Right : EnumMoveDirection.Left;
            }

            return dy >= 0 ? EnumMoveDirection.Down : EnumMoveDirection.Up;
        }
    }
}