using GridDemo.Robots;
using System;

namespace GridDemo.RobotRuns
{
    /// <summary>
    /// 机器人“平移/前进”执行器（运动学积分 + 边界/障碍碰撞）：
    /// - 执行 <see cref="RobotCommand"/> 中的 MoveDistance 指令（按 dt 逐帧推进）；
    /// - 与 <see cref="RobotTurn"/> 协作：转向期间不平移，开始转向会打断正在执行的 MoveDistance；
    /// - 同时兼容“手动模式的连续移动”：当 MoveDistance 的距离为 <see cref="double.MaxValue"/> 时，按 <see cref="RobotManager.OrientationAngle"/> 连续方向移动。
    ///
    /// 典型调用链（项目内）：
    /// - 自动模式：<see cref="RobotManager.Tick"/> 从 Provider 拉取 MoveDistance 指令 -> 调用 <see cref="StartMoveDistance_NoLock"/> -> 仿真线程每帧调用 <see cref="Update"/> 积分。
    /// - 手动模式：W 按下时 <see cref="RobotManager.InputManualForwardKey"/> 下发 “MoveDistance(MaxValue)” -> <see cref="Update"/> 以朝向角连续移动；
    ///            A/D 按下时由 <see cref="RobotTurn.Update"/> 改变朝向角，平移跟随朝向变化。
    /// </summary>
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

        // 可通行判定（世界坐标 -> 是否可走）
        private readonly Func<double, double, bool> _isWorldWalkable;
        private readonly double _robotRadiusM;

        // 指令执行状态：MoveDistance
        private bool _moveDistanceActive;
        private double _moveDistanceRemainM;

        // 新增：机器人-机器人碰撞判定（new center -> 是否撞到其它机器人）
        private readonly Func<double, double, bool> _isHitOtherRobot;

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
            Func<double, double, bool> isWorldWalkable = null,
            Func<double, double, bool> isHitOtherRobot = null
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
            _isHitOtherRobot = isHitOtherRobot;

            _robotRadiusM = _cellSizeM / 3.0; // 机器人半径
            _turnController = new RobotTurn(_robotLock, _robotManager, this, _dt);
        }

        /// <summary>
        /// 暴露转向控制器给外部（RobotManager.BindRuntime 会绑定 Turn 执行器）。
        /// </summary>
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
                // 如果当前正在转向，则本帧不执行平移逻辑
                // 注意：真正的转向更新在锁外调用 _turnController.Update()
                if (_robotManager.IsTurning)
                {
                    // 转向期间不走位移
                }
                else if (_moveDistanceActive)
                {
                    // 1) 读取当前运动学状态
                    double v = _getRobotSpeed();     // 当前速度（米/秒）
                    double vmax = _robotManager.MaxSpeed;   // 最大速度（米/秒）
                    double x = _getRobotX();         // 当前 x（米）
                    double y = _getRobotY();         // 当前 y（米）
                    EnumMoveDirection dir = _robotManager.Direction;

                    bool isManualInfiniteMove = _moveDistanceRemainM == double.MaxValue;

                    // 2) 计算本帧加速度：自动模式使用“加速-匀速-减速”，手动无限移动保持原逻辑
                    double configuredAcc = _robotManager.Acc;
                    double aMag = Math.Abs(configuredAcc);

                    double a;
                    if (isManualInfiniteMove || aMag <= 0.000001)
                    { // 手动无限前进：加速度按配置（允许为 0）即可
                        a = configuredAcc;
                    }
                    else
                    {
                        // 自动 MoveDistance：根据剩余距离决定加速/减速
                        // 刹车距离 d = v^2 / (2a)
                        double dStop = (v * v) / (2.0 * aMag);

                        if (dStop >= _moveDistanceRemainM)
                        {
                            // 需要开始刹车，确保能在剩余距离内停下
                            a = -aMag;
                        }
                        else
                        {
                            // 还在加速/匀速阶段
                            a = aMag;
                        }
                    }

                    // 3) 欧拉积分更新速度：v(t+dt) = v(t) + a*dt
                    v += a * _dt;
                    if (v < 0.0)
                    {
                        v = 0.0;
                    }
                    if (v > vmax)
                    {
                        v = vmax;
                    }

                    // 4) 计算步长，并确保不超过剩余距离（仅对固定距离移动有限制）
                    double step = v * _dt;
                    if (!isManualInfiniteMove && step > _moveDistanceRemainM)
                    {
                        step = _moveDistanceRemainM;
                    }

                    // 5) 更新坐标
                    double newX = x;
                    double newY = y;

                    if (isManualInfiniteMove)
                    { // 手动模式的无限前进：按朝向角移动
                        double angle = _robotManager.OrientationAngle;
                        newX += Math.Cos(angle) * step;
                        newY += Math.Sin(angle) * step;
                    }
                    else
                    { // 自动模式的 MoveDistance：按固定离散方向移动
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
                    }

                    // 6) 边界夹紧 + 障碍物处理
                    ClampToWorld_NoLock(ref newX, ref newY);

                    if (IsHitObstacle_NoLock(newX, newY))
                    {
                        StopImmediately_NoLock();
                        return;
                    }

                    // 新增：机器人-机器人碰撞
                    if (_isHitOtherRobot != null && _isHitOtherRobot(newX, newY))
                    {
                        StopImmediately_NoLock();
                        return;
                    }

                    // 7) 扣减剩余距离（固定距离移动）
                    if (!isManualInfiniteMove)
                    {
                        _moveDistanceRemainM -= step;
                    }

                    // 8) 回写速度与位置
                    _setRobotSpeed(v);
                    _setRobotX(newX);
                    _setRobotY(newY);

                    // 9) 指令完成判定：距离到 0，或速度已降为 0 且剩余距离很小（避免抖动/卡死）
                    if (!isManualInfiniteMove)
                    {
                        if (_moveDistanceRemainM <= 0.000001 || (v <= 0.000001 && _moveDistanceRemainM <= 0.01))
                        {
                            _moveDistanceActive = false;
                            _moveDistanceRemainM = 0.0;
                            _setRobotSpeed(0.0);
                            _robotManager.Acc = 0.0;
                        }
                    }
                }
            }
            // 转向更新（锁外调用以避免死锁）
            _turnController.Update();
        }

        /// <summary>
        /// 判断给定中心点位置是否与障碍物发生碰撞（不加锁版本）。
        /// </summary>
        /// <remarks>
        /// 实现策略：
        /// 1) 先用 _isWorldWalkable(center) 做快速失败；
        /// 2) 再枚举机器人圆形包围范围附近的格子；
        /// 3) 对“障碍格”使用圆-矩形相交检测（closest point）。
        /// </remarks>
        private bool IsHitObstacle_NoLock(double centerX, double centerY)
        {
            double r = _robotRadiusM;      // 机器人半径

            // 第一步：直接用传入的可行走函数判断机器人中心点是否在可行走区域上
            // 若中心点都不可走，则一定视为已经碰撞
            if (!_isWorldWalkable(centerX, centerY))
            {
                return true;
            }

            // 第二步：枚举机器人圆形包围范围附近的格子，检测碰撞
            double worldW = _getWorldWidthM();
            double worldH = _getWorldHeightM();
            int gridW = (int)Math.Floor(worldW / _cellSizeM);
            int gridH = (int)Math.Floor(worldH / _cellSizeM);

            // 以机器人中心为圆心、半径为 r 的圆，向外再多预留一圈格子，
            // 计算其在网格坐标系下覆盖到的格子索引范围 [minX, maxX], [minY, maxY]
            int minX = (int)Math.Floor((centerX - r) / _cellSizeM) - 1;
            int maxX = (int)Math.Floor((centerX + r) / _cellSizeM) + 1;
            int minY = (int)Math.Floor((centerY - r) / _cellSizeM) - 1;
            int maxY = (int)Math.Floor((centerY + r) / _cellSizeM) + 1;

            // 第三步：遍历所有候选格子，查找障碍格并做圆-矩形相交检测
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

                    // 圆-矩形相交检测，对机器人“圆形包围体”与该障碍格的矩形边界做精确相交检测
                    double rectLeft = gx * _cellSizeM;          // 障碍格的矩形边界
                    double rectRight = rectLeft + _cellSizeM;
                    double rectTop = gy * _cellSizeM;
                    double rectBottom = rectTop + _cellSizeM;

                    // 找矩形和圆心的最近点：坐标夹紧
                    double closestX = Clamp(centerX, rectLeft, rectRight);
                    double closestY = Clamp(centerY, rectTop, rectBottom);

                    double dx = centerX - closestX;
                    double dy = centerY - closestY;

                    if (dx * dx + dy * dy <= r * r)
                    { // 若最近点与圆心的距离 <= 半径，则碰撞成立
                        return true;
                    }
                }
            }
            // 遍历完所有相关格子都没撞上障碍，则认为当前位置是安全的
            return false;
        }

        /// <summary>
        /// 将 v 夹紧到 [min, max]。
        /// </summary>
        private static double Clamp(double v, double min, double max)
        {
            if (v < min)
            {
                return min;
            }
            if (v > max)
            {
                return max;
            }
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