using System;

namespace Graphic
{
    /// <summary>
    /// 负责机器人自动巡航逻辑（从左上 → 右上 → 右下）
    /// 不直接画图，只通过 Robot 提供的接口控制：目标角度、速度等。
    /// </summary>
    public class RobotAutoNavigator
    {
        private readonly Robot _robot;

        // 航路点：0=右上，1=右下
        private int _waypointIndex = 0;

        private double _targetX;
        private double _targetY;

        // 终点容差
        private readonly double _posReachedEps = 0.01;

        // world 信息
        private readonly double _worldWidth;
        private readonly double _worldHeight;
        private readonly double _cellSize;

        public RobotAutoNavigator(Robot robot)
        {
            _robot = robot;

            (_worldWidth, _worldHeight, _cellSize) = _robot.GetWorldInfo();

            // 初始化路线：默认从左上出发
            ResetRoute();
        }

        /// <summary>
        /// 重新从 “左上→右上→右下” 巡航
        /// </summary>
        public void ResetRoute()
        {
            _waypointIndex = 0;

            // 右上角格子中心
            _targetX = _worldWidth - _cellSize / 2.0;
            _targetY = _cellSize / 2.0;

            // 让机器人朝向右上
            UpdateTargetAngleToCurrentGoal();
            // 速度初始为 0，由外部 UpdateLoop 设置
            _robot.SetSpeedDirect(0);
        }

        /// <summary>
        /// 每帧调用，根据当前 waypoint 控制机器人：
        /// - 设置目标角度（_targetAngle）
        /// - 设置线速度（Speed）
        /// </summary>
        public void Update(double dt)
        {
            // 如果当前不是自动模式，则不做任何事
            if (_robot.Mode != EnumControlMode.AutoNavigate)
                return;

            // 获取当前位置
            var (x, y) = _robot.GetPosition();

            // 与当前目标点的距离
            double dx = _targetX - x;
            double dy = _targetY - y;
            double dist = Math.Sqrt(dx * dx + dy * dy);

            if (dist <= _posReachedEps)
            {
                // 已到达当前 waypoint，切换到下一个
                if (_waypointIndex == 0)
                {
                    // 右上 → 右下
                    _waypointIndex = 1;
                    _targetX = _worldWidth - _cellSize / 2.0;
                    _targetY = _worldHeight - _cellSize / 2.0;

                    // 停一下，修改目标角度，让 Robot 用自带的转向动画转过去
                    _robot.SetSpeedDirect(0);
                    UpdateTargetAngleToCurrentGoal();
                }
                else if (_waypointIndex == 1)
                {
                    // 已到右下，停止（也可以循环）
                    _robot.SetSpeedDirect(0);

                    // 如果想循环：
                    // ResetRoute();
                }
            }
            else
            {
                // 尚未到达目标点：保持目标角朝向终点
                UpdateTargetAngleToCurrentGoal();

                // 控制速度：简单起见，让它匀速接近 MaxSpeed
                double maxSpeed = _robot.GetMaxSpeed();
                _robot.SetSpeedDirect(maxSpeed);
            }
        }

        private void UpdateTargetAngleToCurrentGoal()
        {
            var (x, y) = _robot.GetPosition();
            double dx = _targetX - x;
            double dy = _targetY - y;
            double ang = Math.Atan2(dy, dx);
            _robot.SetTargetAngle(ang);
        }
    }
}
