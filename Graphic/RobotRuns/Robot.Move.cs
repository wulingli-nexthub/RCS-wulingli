using System;

namespace GridDemo.RobotRuns
{
    internal class RobotMove
    {
        private readonly object _robotLock;
        private readonly Func<double> _getRobotX;
        private readonly Action<double> _setRobotX;
        private readonly Func<double> _getRobotY;
        private readonly Action<double> _setRobotY;
        private readonly Func<double> _getRobotSpeed;
        private readonly Action<double> _setRobotSpeed;
        private readonly RobotManager _robotManager;
        private readonly Func<double> _getWorldWidthM;
        private readonly Func<double> _getWorldHeightM;
        private readonly double _cellSizeM;
        private readonly double _dt;

        private readonly RobotTurn _turnController;

        // 新增：可通行判定（世界坐标 -> 是否可走）
        private readonly Func<double, double, bool> _isWorldWalkable;
        private readonly double _robotRadiusM;

        // 指令执行状态：MoveDistance
        private bool _moveDistanceActive;
        private double _moveDistanceRemainM;

        public RobotMove(
            object robotLock,
            Func<double> getRobotX,
            Action<double> setRobotX,
            Func<double> getRobotY,
            Action<double> setRobotY,
            Func<double> getRobotSpeed,
            Action<double> setRobotSpeed,
            RobotManager robotManager,
            Func<double> getWorldWidthM,
            Func<double> getWorldHeightM,
            double cellSizeM,
            double dt,
            Func<double, double, bool> isWorldWalkable = null
            )
        {
            _robotLock = robotLock ?? throw new ArgumentNullException(nameof(robotLock));
            _getRobotX = getRobotX ?? throw new ArgumentNullException(nameof(getRobotX));
            _setRobotX = setRobotX ?? throw new ArgumentNullException(nameof(setRobotX));
            _getRobotY = getRobotY ?? throw new ArgumentNullException(nameof(getRobotY));
            _setRobotY = setRobotY ?? throw new ArgumentNullException(nameof(setRobotY));
            _getRobotSpeed = getRobotSpeed ?? throw new ArgumentNullException(nameof(getRobotSpeed));
            _setRobotSpeed = setRobotSpeed ?? throw new ArgumentNullException(nameof(setRobotSpeed));
            _robotManager = robotManager ?? throw new ArgumentNullException(nameof(robotManager));
            _getWorldWidthM = getWorldWidthM ?? throw new ArgumentNullException(nameof(getWorldWidthM));
            _getWorldHeightM = getWorldHeightM ?? throw new ArgumentNullException(nameof(getWorldHeightM));
            _cellSizeM = cellSizeM;
            _dt = dt;
            // 默认全可走，保持兼容
            _isWorldWalkable = isWorldWalkable ?? ((x, y) => true);
            _robotRadiusM = _cellSizeM / 3.0; // 机器人半径
            _turnController = new RobotTurn(_robotLock, _robotManager, this, _dt);
        }

        public RobotTurn TurnController
        {
            get { return _turnController; }
        }

        /// <summary>
        /// 为转向做准备的“停车”：
        /// - 清零速度与加速度（避免继续积分位移）；
        /// - 转向将打断当前 MoveDistance（清理执行状态）；
        /// - 内部加锁，适合从外部线程/定时器在未知锁状态下调用。
        /// </summary>
        public void StopForTurn()
        {
            lock (_robotLock)
            {
                _setRobotSpeed(0.0);
                _robotManager.Acc = 0.0;

                // 转向会打断前进指令
                _moveDistanceActive = false;
                _moveDistanceRemainM = 0.0;
            }
        }

        /// <summary>
        /// 立即停止（不加锁版本）：
        /// 仅用于调用方已持有 `_robotLock` 的场景，避免重复 lock 或锁顺序问题。
        /// </summary>
        public void StopImmediately_NoLock()
        {
            _setRobotSpeed(0.0);
            _robotManager.Acc = 0.0;
            _moveDistanceActive = false;
            _moveDistanceRemainM = 0.0;
        }

        /// <summary>
        /// 开始执行“前进位移”指令（米）。
        /// 注意：该方法不加锁，要求调用方已经持有 `_robotLock`，用于上层批量更新时减少锁开销。
        /// </summary>
        /// <param name="distanceM">期望前进距离（米）。小于等于 0 会直接视为无指令。</param>
        /// <param name="forwardAcc">前进加速度（米/秒^2）。允许由上层决定加速策略。</param>
        public void StartMoveDistance_NoLock(double distanceM, double forwardAcc)
        {
            if (distanceM <= 0)
            { // 这是一个“0 距离指令”：立即视作完成，并在此处自然停车
                _moveDistanceActive = false;
                _moveDistanceRemainM = 0.0;

                _setRobotSpeed(0.0);
                _robotManager.Acc = 0.0;
                return;
            }

            if (_robotManager.IsTurning)
            { // 转向中不允许开始移动
                _moveDistanceActive = false;
                _moveDistanceRemainM = 0.0;
                return;
            }

            // 写入加速度并激活指令：Update() 将按 dt 逐步扣减 remain。
            _robotManager.Acc = forwardAcc;
            _moveDistanceActive = true;
            _moveDistanceRemainM = distanceM;
        }

        /// <summary>
        /// 查询 MoveDistance 是否完成（不加锁版本）。
        /// 说明：`_moveDistanceActive == false` 表示未在执行（可能完成，也可能被 Stop/Turn 打断）。
        /// </summary>
        public bool IsMoveDistanceDone_NoLock()
        {
            return !_moveDistanceActive;
        }

        /// <summary>
        /// 每帧更新运动学：
        /// - 转向期间不移动；
        /// - 若存在 MoveDistance 指令：按 v=a*dt 积分更新速度，并按 step=v*dt 扣减剩余距离；
        /// - 移动后执行边界夹紧，确保机器人始终位于世界范围内（以半格为边界）。
        /// </summary>
        public void Update()
        {
            lock (_robotLock)
            {
                // 先更新转向动画（转向期间不移动）
                // 注意：RobotTurn.Update 内部也 lock，同一把锁会死锁，所以这里不提前调用
                // 转向 Update 移到锁外执行
                if (_robotManager.IsTurning)
                {
                    // 转向期间不走位移
                }
                else if (_moveDistanceActive)
                {
                    // 1) 读取当前运动学状态
                    double v = _getRobotSpeed();     // 当前速度（米/秒）
                    double a = _robotManager.Acc;           // 当前加速度（米/秒^2）
                    double vmax = _robotManager.MaxSpeed;   // 最大速度（米/秒）
                    double x = _getRobotX();         // 当前 x（米）
                    double y = _getRobotY();         // 当前 y（米）
                    EnumMoveDirection dir = _robotManager.Direction;

                    // 2) 欧拉积分更新速度：v(t+dt) = v(t) + a*dt
                    v += a * _dt;
                    // 将速度限制在 [0, vmax]：避免负速度导致“倒退”或速度上溢。
                    if (v < 0)
                    {
                        v = 0;
                    }
                    if (v > vmax)
                    {
                        v = vmax;
                    }

                    // 3) 计算本帧位移步长，并确保不超过剩余距离
                    double step = v * _dt;
                    if (step > _moveDistanceRemainM)
                    {
                        step = _moveDistanceRemainM;
                    }

                    // 4) 按方向更新坐标：仅允许四向网格移动
                    double newX = x;
                    double newY = y;
                    switch (dir)
                    {
                        case EnumMoveDirection.Right:
                            newX += step;
                            break;
                        case EnumMoveDirection.Left:
                            newX -= step;
                            break;
                        case EnumMoveDirection.Down:
                            newY += step;
                            break;
                        case EnumMoveDirection.Up:
                            newY -= step;
                            break;
                    }

                    // 5) 边界夹紧，碰到障碍物
                    ClampToWorld_NoLock(ref newX, ref newY);

                    // 命中障碍物：停止（不允许穿过）
                    if (IsHitObstacle_NoLock(newX, newY))
                    {
                        StopImmediately_NoLock();
                        return;
                    }

                    // 6) 扣减剩余距离
                    _moveDistanceRemainM -= step;

                    // 7) 回写速度与位置
                    _setRobotSpeed(v);
                    _setRobotX(newX);
                    _setRobotY(newY);

                    // 8) 指令完成判定：用一个很小的阈值避免浮点误差导致“永远差一点”
                    if (_moveDistanceRemainM <= 0.000001)
                    {
                        // 清理指令并立即停车：保证完成后速度归零，便于上层下一步决策。
                        _moveDistanceActive = false;
                        _moveDistanceRemainM = 0.0;
                        _setRobotSpeed(0.0);
                        _robotManager.Acc = 0.0;
                    }
                }
                else if (_robotManager.Acc != 0.0 || _getRobotSpeed() != 0.0)
                {
                    double v = _getRobotSpeed();
                    double a = _robotManager.Acc;
                    double vmax = _robotManager.MaxSpeed;

                    // 1) 积分速度
                    v += a * _dt;
                    if (v < 0)
                    {
                        v = 0;
                    }
                    if (v > vmax)
                    {
                        v = vmax;
                    }

                    // 2) 根据 OrientationAngle 做连续方向移动
                    double x = _getRobotX();
                    double y = _getRobotY();

                    double step = v * _dt;

                    // 使用朝向角度，而不是离散方向
                    double angle = _robotManager.OrientationAngle;
                    double newX = x + Math.Cos(angle) * step;
                    double newY = y + Math.Sin(angle) * step;

                    ClampToWorld_NoLock(ref newX, ref newY);

                    // 手动模式核心：命中障碍物就刹停，玩家需要自己转向绕行
                    if (IsHitObstacle_NoLock(newX, newY))
                    {
                        StopImmediately_NoLock();
                        return;
                    }

                    _setRobotSpeed(v);
                    _setRobotX(newX);
                    _setRobotY(newY);
                }
            }

            _turnController.Update();
        }

        private bool IsHitObstacle_NoLock(double centerX, double centerY)
        {
            double r = _robotRadiusM;

            if (!_isWorldWalkable(centerX, centerY))
            {
                return true;
            }

            double worldW = _getWorldWidthM();
            double worldH = _getWorldHeightM();
            int gridW = (int)Math.Floor(worldW / _cellSizeM);
            int gridH = (int)Math.Floor(worldH / _cellSizeM);

            int minX = (int)Math.Floor((centerX - r) / _cellSizeM) - 1;
            int maxX = (int)Math.Floor((centerX + r) / _cellSizeM) + 1;
            int minY = (int)Math.Floor((centerY - r) / _cellSizeM) - 1;
            int maxY = (int)Math.Floor((centerY + r) / _cellSizeM) + 1;

            for (int gy = minY; gy <= maxY; gy++)
            {
                for (int gx = minX; gx <= maxX; gx++)
                {
                    // 越界格子：跳过（不要直接判碰撞）
                    if (gx < 0 || gy < 0 || gx >= gridW || gy >= gridH)
                    {
                        continue;
                    }

                    // 用格子中心点判断该格是否为障碍格
                    double cellCenterX = gx * _cellSizeM + _cellSizeM / 2.0;
                    double cellCenterY = gy * _cellSizeM + _cellSizeM / 2.0;

                    if (_isWorldWalkable(cellCenterX, cellCenterY))
                    {
                        continue;
                    }

                    // 圆-矩形相交检测
                    double rectLeft = gx * _cellSizeM;
                    double rectRight = rectLeft + _cellSizeM;
                    double rectTop = gy * _cellSizeM;
                    double rectBottom = rectTop + _cellSizeM;

                    double closestX = Clamp(centerX, rectLeft, rectRight);
                    double closestY = Clamp(centerY, rectTop, rectBottom);

                    double dx = centerX - closestX;
                    double dy = centerY - closestY;

                    if (dx * dx + dy * dy <= r * r)
                    {
                        return true;
                    }
                }
            }

            return false;
        }

        private static double Clamp(double v, double min, double max)
        {
            if (v < min) return min;
            if (v > max) return max;
            return v;
        }

        /// <summary>
        /// 边界夹紧
        /// </summary>
        private void ClampToWorld_NoLock(ref double x, ref double y)
        {
            double worldWidth = _getWorldWidthM();
            double worldHeight = _getWorldHeightM();
            double halfCell = _cellSizeM / 2.0;

            if (x < halfCell)
            {
                x = halfCell;
            }
            if (y < halfCell)
            {
                y = halfCell;
            }
            if (x > worldWidth - halfCell)
            {
                x = worldWidth - halfCell;
            }
            if (y > worldHeight - halfCell)
            {
                y = worldHeight - halfCell;
            }
        }
    }
}