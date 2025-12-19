using Graphic.RobotRuns;
using System;

namespace Graphic.RobotModels
{
    internal sealed class RobotAutoNavigator
    {
        private readonly object _robotLock;
        private readonly Func<double> _getRobotX;
        private readonly Func<double> _getRobotY;
        private readonly Action<double> _setRobotSpeed;

        private readonly Func<double> _getWorldWidthM;
        private readonly Func<double> _getWorldHeightM;

        private readonly double _cellSizeM;
        private readonly Robot _robot;

        private double _targetX;
        private double _targetY;
        private bool _hasTarget;

        // 到达阈值（米）：略大于 dt 带来的一个步长抖动
        private readonly double _arriveEpsilonM;

        public RobotAutoNavigator(
            object robotLock,
            Func<double> getRobotX,
            Func<double> getRobotY,
            Action<double> setRobotSpeed,
            Func<double> getWorldWidthM,
            Func<double> getWorldHeightM,
            double cellSizeM,
            Robot robot,
            double arriveEpsilonM = 0.02)
        {
            _robotLock = robotLock ?? throw new ArgumentNullException(nameof(robotLock));
            _getRobotX = getRobotX ?? throw new ArgumentNullException(nameof(getRobotX));
            _getRobotY = getRobotY ?? throw new ArgumentNullException(nameof(getRobotY));
            _setRobotSpeed = setRobotSpeed ?? throw new ArgumentNullException(nameof(setRobotSpeed));
            _getWorldWidthM = getWorldWidthM ?? throw new ArgumentNullException(nameof(getWorldWidthM));
            _getWorldHeightM = getWorldHeightM ?? throw new ArgumentNullException(nameof(getWorldHeightM));
            _cellSizeM = cellSizeM;
            _robot = robot ?? throw new ArgumentNullException(nameof(robot));

            _arriveEpsilonM = arriveEpsilonM;
        }

        public bool IsEnabled { get; private set; }

        public void Enable()
        {
            lock (_robotLock)
            {
                IsEnabled = true;

                // 初始化一次目标：右下角（cell 中心点）
                double halfCell = _cellSizeM / 2.0;
                _targetX = _getWorldWidthM() - halfCell;
                _targetY = _getWorldHeightM() - halfCell;
                _hasTarget = true;

                // 自动模式下视为持续“前进”
                _robot.IsForwardKeyDown = true;
            }
        }

        public void Disable()
        {
            lock (_robotLock)
            {
                IsEnabled = false;
                _hasTarget = false;

                _robot.IsForwardKeyDown = false;
                _robot.Acc = 0.0;
                _setRobotSpeed(0.0);
            }
        }

        /// <summary>
        /// 给 RobotMove 使用：自动模式下每帧提供“应该怎么走”
        /// </summary>
        public RobotAutoMotionState GetAutoMotionState(Func<double> getForwardAcc)
        {
            if (getForwardAcc == null) throw new ArgumentNullException(nameof(getForwardAcc));

            lock (_robotLock)
            {
                if (!IsEnabled || !_hasTarget)
                {
                    return RobotAutoMotionState.Disabled;
                }

                double x = _getRobotX();
                double y = _getRobotY();

                double dx = _targetX - x;
                double dy = _targetY - y;

                bool arriveX = Math.Abs(dx) <= _arriveEpsilonM;
                bool arriveY = Math.Abs(dy) <= _arriveEpsilonM;

                if (arriveX && arriveY)
                {
                    // 到达：停住（仍保持自动模式 enabled，防止被键盘抢控制）
                    _robot.Acc = 0.0;
                    _setRobotSpeed(0.0);
                    _robot.IsForwardKeyDown = false;

                    return new RobotAutoMotionState(
                        enabled: true,
                        direction: _robot.Direction,
                        acc: 0.0,
                        suppressEdgeTurning: true,
                        clampOnBounds: true
                    );
                }

                // 简化策略：先对齐 X，再对齐 Y
                EnumMoveDirection dir;
                if (!arriveX)
                {
                    dir = dx >= 0 ? EnumMoveDirection.Right : EnumMoveDirection.Left;
                }
                else
                {
                    dir = dy >= 0 ? EnumMoveDirection.Down : EnumMoveDirection.Up;
                }

                double acc = getForwardAcc();
                _robot.IsForwardKeyDown = true;
                _robot.Acc = acc;

                return new RobotAutoMotionState(
                    enabled: true,
                    direction: dir,
                    acc: acc,
                    suppressEdgeTurning: true,
                    clampOnBounds: true
                );
            }
        }
    }
}