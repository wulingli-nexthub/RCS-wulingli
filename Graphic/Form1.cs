using System;
using System.Drawing;
using System.Windows.Forms;

namespace Graphic
{
    /// <summary>
    /// 主窗体：负责组合各个类，并处理 UI 事件
    /// 具体职责拆分在：
    /// - DrawGrid：只管网格坐标转换和绘制
    /// - Robot：只管机器人物理状态和绘制
    /// - WorldTransform：只管缩放、平移（视图变换）
    /// - RobotSimulation：后台线程定时调用 Robot.Update
    /// </summary>
    public partial class Form1 : Form
    {
        private readonly DrawGrid _grid;     // 调用 DrawGrid 画网格
        private readonly Robot _robot;     // 调用 Robot 画机器人
        private readonly WorldTransform _view;      // 视图变换：缩放、平移
        private readonly RobotSimulation _simulation;     // 后台模拟线程

        private bool _isPanning = false;           // 是否正在拖动画布
        private Point _lastMousePos;            // 获取鼠标位置

        private const double Dt = 0.02;            // 模拟时间步长，单位秒

        public Form1()
        {
            InitializeComponent();

            DoubleBuffered = true;      // 开启双缓冲，减少GDI+重绘闪烁
            SetStyle(ControlStyles.AllPaintingInWmPaint |
                     ControlStyles.UserPaint |
                     ControlStyles.OptimizedDoubleBuffer, true);


            _grid = new DrawGrid();       // 组合各个功能类
            _robot = new Robot(_grid.CellSizeM, _grid.WorldWidthM, _grid.WorldHeightM);
            _view = new WorldTransform(_grid);

            // 后台模拟线程，
            // - 内部线程每 Dt 秒推进一次机器人状态
            // - 通过回调请求 UI 线程 Invalidate() 重绘
            _simulation = new RobotSimulation(_robot, Dt, () =>
            {
                if (!IsDisposed)
                {
                    BeginInvoke(new Action(Invalidate));
                }
            });

            KeyPreview = true; // 让窗体优先接收键盘事件

            // 事件注册（也可以在 Designer 中绑定）
            Load += Form1_Load;
            Paint += Form1_Paint;
            MouseWheel += Form1_MouseWheel;
            MouseDown += Form1_MouseDown;
            MouseMove += Form1_MouseMove;
            MouseUp += Form1_MouseUp;
            FormClosing += Form1_FormClosing;
            Resize += Form1_Resize;
            KeyDown += Form1_KeyDown;
            KeyUp += Form1_KeyUp;

            numericAcc.KeyDown += numeric_KeyDown;
            numericVmax.KeyDown += numeric_KeyDown;
        }

        private void numeric_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.KeyCode == Keys.Enter)
            {
                this.ActiveControl = null; // 焦点离开数值框
                e.Handled = true;
            }
        }

        private void Form1_KeyUp(object sender, KeyEventArgs e)
        {
            switch (e.KeyCode)
            {
                case Keys.W:
                case Keys.Up:
                    // 松开 W/↑：停止前进
                    _robot.StopMoving();
                    break;
            }
        }

        private void Form1_KeyDown(object sender, KeyEventArgs e)
        {
            switch (e.KeyCode)
            {
                case Keys.W:
                case Keys.Up:
                    // 按下 W/↑：开始按当前方向前
                    _robot.StartMoving();
                    break;

                case Keys.A:
                case Keys.Left:
                    // 左转（只改目标方向，转向由动画完成）
                    _robot.TurnLeft();
                    break;

                case Keys.D:
                case Keys.Right:
                    // 右转
                    _robot.TurnRight();
                    break;
            }
        }

        /// <summary>
        /// 首次加载时，根据当前 ClientSize 计算合适缩放并让网格居中。
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Form1_Load(object sender, EventArgs e)
        {
            _view.CentreGrid(ClientSize.Width, ClientSize.Height);
        }

        /// <summary>
        /// 窗体大小变化时，按当前缩放重新计算偏移，让网格始终居中显示。
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Form1_Resize(object sender, EventArgs e)
        {
            _view.RecenterOnResize(ClientSize.Width, ClientSize.Height);
            Invalidate();
        }

        /// <summary>
        /// 主绘制函数：网格 + 机器人 + 左上角状态文字。
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Form1_Paint(object sender, PaintEventArgs e)
        {
            Graphics g = e.Graphics;
            g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;
            g.Clear(Color.White);

            _grid.Draw(g);    // 1. 画网格

            _robot.Draw(g, _grid);            // 2. 画机器人

            using (var font = new Font("宋体", 10))               // 3. 显示信息
            using (var brush = new SolidBrush(Color.Black))
            {
                var state = _robot.GetState();
                string info = $"Scale: {_grid.Scale:F1} px/m   Offset: ({_grid.OffsetX:F0}, {_grid.OffsetY:F0})";
                string infoRobot = $"Robot: x={state.X:F2}m, y={state.Y:F2}m, v={state.V:F2}m/s, a={state.A:F2}m/s2";

                g.DrawString(info, font, brush, new PointF(10, 10));
                g.DrawString(infoRobot, font, brush, new PointF(10, 25));
            }
        }

        /// <summary>
        /// 鼠标滚轮缩放：以鼠标为中心进行放大/缩小。
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Form1_MouseWheel(object sender, MouseEventArgs e)
        {
            _view.ZoomAt(e.Delta, e.X, e.Y);
            Invalidate();
        }

        /// <summary>
        /// 左键按下开始拖动画布
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Form1_MouseDown(object sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Left)
            {
                _isPanning = true;
                _lastMousePos = e.Location;
                Cursor = Cursors.Hand;
            }
        }

        /// <summary>
        /// 鼠标移动：在拖动模式下偏移视图。
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Form1_MouseMove(object sender, MouseEventArgs e)
        {
            if (_isPanning)
            {
                int dx = e.X - _lastMousePos.X;
                int dy = e.Y - _lastMousePos.Y;
                _lastMousePos = e.Location;

                _view.Pan(dx, dy);
                Invalidate();
            }
        }

        /// <summary>
        /// 左键抬起时结束拖动。
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Form1_MouseUp(object sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Left)
            {
                _isPanning = false;
                Cursor = Cursors.Default;
            }
        }

        /// <summary>
        /// 窗体关闭：请求模拟线程安全退出
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            _simulation.Stop();
        }

        /// <summary>
        /// 加速度数值框变更：更新机器人加速度（m/s²）。
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void numericAcc_ValueChanged(object sender, EventArgs e)
        {
            _robot.SetAcc((double)((NumericUpDown)sender).Value);
        }

        private void numericVmax_ValueChanged(object sender, EventArgs e)
        {
            _robot.SetSpeed((double)((NumericUpDown)sender).Value);
        }

        /// <summary>
        /// 重置按钮：重置网格到初始缩放/居中
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void btnReset_Click(object sender, EventArgs e)
        {
            _view.ResetAndCenter(ClientSize.Width, ClientSize.Height);

            Invalidate(); // 触发重绘
        }

    }
}
