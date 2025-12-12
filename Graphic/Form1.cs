using System;
using System.Drawing;
using System.Windows.Forms;

namespace Graphic
{
    public partial class Form1 : Form
    {
        private readonly DrawGrid _grid;
        private readonly Robot _robot;
        private readonly WorldTransform _view;
        private readonly RobotSimulation _simulation;

        // 拖动画布
        private bool _isPanning = false;
        private Point _lastMousePos;

        private const double Dt = 0.02;

        public Form1()
        {
            InitializeComponent();

            DoubleBuffered = true;
            SetStyle(ControlStyles.AllPaintingInWmPaint |
                     ControlStyles.UserPaint |
                     ControlStyles.OptimizedDoubleBuffer, true);

            // 组合各个类
            _grid = new DrawGrid();
            _robot = new Robot(_grid.CellSizeM, _grid.WorldWidthM, _grid.WorldHeightM);
            _view = new WorldTransform(_grid);

            // 后台模拟线程，回调中使用 BeginInvoke 触发重绘
            _simulation = new RobotSimulation(_robot, Dt, () =>
            {
                if (!IsDisposed)
                {
                    BeginInvoke(new Action(Invalidate));
                }
            });

            // 事件注册（也可以在 Designer 中绑定）
            Load += Form1_Load;
            Paint += Form1_Paint;
            MouseWheel += Form1_MouseWheel;
            MouseDown += Form1_MouseDown;
            MouseMove += Form1_MouseMove;
            MouseUp += Form1_MouseUp;
            FormClosing += Form1_FormClosing;
            Resize += Form1_Resize;
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            _view.CentreGrid(ClientSize.Width, ClientSize.Height);
        }

        private void Form1_Resize(object sender, EventArgs e)
        {
            _view.RecenterOnResize(ClientSize.Width, ClientSize.Height);
            Invalidate();
        }

        private void Form1_Paint(object sender, PaintEventArgs e)
        {
            Graphics g = e.Graphics;
            g.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias;
            g.Clear(Color.White);

            // 1. 画网格
            _grid.Draw(g);

            // 2. 画机器人
            _robot.Draw(g, _grid);

            // 3. 显示信息
            using (var font = new Font("宋体", 10))
            using (var brush = new SolidBrush(Color.Black))
            {
                var state = _robot.GetState();
                string info = $"Scale: {_grid.Scale:F1} px/m   Offset: ({_grid.OffsetX:F0}, {_grid.OffsetY:F0})";
                string infoRobot = $"Robot: x={state.X:F2}m, y={state.Y:F2}m, v={state.V:F2}m/s, a={state.A:F2}m/s2";

                g.DrawString(info, font, brush, new PointF(10, 10));
                g.DrawString(infoRobot, font, brush, new PointF(10, 25));
            }
        }

        private void Form1_MouseWheel(object sender, MouseEventArgs e)
        {
            _view.ZoomAt(e.Delta, e.X, e.Y);
            Invalidate();
        }

        private void Form1_MouseDown(object sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Left)
            {
                _isPanning = true;
                _lastMousePos = e.Location;
                Cursor = Cursors.Hand;
            }
        }

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

        private void Form1_MouseUp(object sender, MouseEventArgs e)
        {
            if (e.Button == MouseButtons.Left)
            {
                _isPanning = false;
                Cursor = Cursors.Default;
            }
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            _simulation.Stop();
        }


        private void numericAcc_ValueChanged(object sender, EventArgs e)
        {
            _robot.SetAcc((double)((NumericUpDown)sender).Value);
        }

        private void numericVinit_ValueChanged(object sender, EventArgs e)
        {
            _robot.SetSpeed((double)((NumericUpDown)sender).Value);
        }

        private void btnReset_Click(object sender, EventArgs e)
        {
            _view.ResetAndCenter(ClientSize.Width, ClientSize.Height);

            Invalidate(); // 触发重绘
        }
    }
}
