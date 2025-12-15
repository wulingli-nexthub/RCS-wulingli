using System;
using System.Threading;
using System.Windows.Forms;

namespace Graphic
{
    public partial class Form1
    {
        /// <summary>
        /// 机器人状态更新
        /// </summary>
        /// <param name="dt"></param>
        private void _Thread_UpdateRobot()
        {
            lock (_robotLock)
            {
                // 速度更新
                _robotSpeed += _robotAcc * _dt;

                if (_robotSpeed < 0)
                {
                    _robotSpeed = 0;
                }

                if (_robotSpeed > _robotMaxSpeed)
                {
                    _robotSpeed = _robotMaxSpeed;
                }

                // 根据方向，用 robotSpeed 更新位置
                switch (_moveDirection)
                {
                    case EnumMoveDirection.Right:
                        _robotX += _robotSpeed * _dt;
                        break;

                    case EnumMoveDirection.Left:
                        _robotX -= _robotSpeed * _dt;
                        break;

                    case EnumMoveDirection.Down:
                        _robotY += _robotSpeed * _dt;
                        break;

                    case EnumMoveDirection.Up:
                        _robotY -= _robotSpeed * _dt;
                        break;
                }

                // 根据位置决定是否转向（右→下→左→上→右...）
                switch (_moveDirection)
                {
                    case EnumMoveDirection.Right:
                        if (_robotX >= _worldWidthM - CellSizeM / 2)
                        {
                            _robotX = _worldWidthM - CellSizeM / 2;
                            _moveDirection = EnumMoveDirection.Down;
                        }
                        break;

                    case EnumMoveDirection.Down:
                        if (_robotY >= _worldHeightM - CellSizeM / 2)
                        {
                            _robotY = _worldHeightM - CellSizeM / 2;
                            _moveDirection = EnumMoveDirection.Left;
                        }
                        break;

                    case EnumMoveDirection.Left:
                        if (_robotX <= 0.0 + CellSizeM / 2)
                        {
                            _robotX = CellSizeM / 2;
                            _moveDirection = EnumMoveDirection.Up;
                        }
                        break;

                    case EnumMoveDirection.Up:
                        if (_robotY <= 0.0 + CellSizeM / 2)
                        {
                            _robotY = CellSizeM / 2;
                            _moveDirection = EnumMoveDirection.Right;
                        }
                        break;
                }
            }
        }

        /// <summary>
        /// 线程主循环，相当于逻辑帧定时器
        /// </summary>
        private void _Thread_Loop()
        {
            while (_isRunning)
            {
                _Thread_UpdateRobot();

                // 通知 UI 线程重绘
                try
                {
                    if (!this.IsDisposed)
                    {
                        this.BeginInvoke(new Action(() =>
                        {
                            this.Invalidate();
                        }));
                    }
                }
                catch
                {
                    // 窗口关闭时可能抛异常，简单吞掉
                }

                // Thread.Sleep(_dt * 1000) 相当于一个简单“定时器”：每 20ms 触发一次逻辑更新
                int sleepMs = (int)(_dt * 1000);
                if (sleepMs < 1) sleepMs = 1;
                Thread.Sleep(sleepMs);
            }
        }

        //窗体关闭时，安全停止线程
        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            _isRunning = false;
            if (_workerThread != null && _workerThread.IsAlive)
            {
                // 等一会儿退出
                _workerThread.Join(200);
            }
        }
    }
}
