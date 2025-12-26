using SkiaSharp;
using SkiaSharp.Views.Desktop;
using System;
using System.Windows.Forms;

namespace GridDemo
{
    public partial class Form1 : Form
    {
        private const int GridCount = 30;               // 网格数30
        private const double CellSizeM = 0.55;          // 一格代表距离0.55米

        private double _worldWidthM = GridCount * CellSizeM;        // 网格在世界坐标中的总宽高（米）
        private double _worldHeightM = GridCount * CellSizeM;

        private double _scale;
        private double _offsetX;        //世界坐标对应屏幕坐标偏移量
        private double _offsetY;

        private double _robotX = CellSizeM / 2;        //机器人初始世界坐标
        private double _robotY = CellSizeM / 2;

        private double _robotSpeed = 0;        //机器人的初始速度和加速度
        private double _robotMaxSpeed = 1.5;
        private double _robotAcc = 0;

        private readonly object _robotLock = new object();   // 机器人共享状态锁：保护 _robotX/_robotY/_robotSpeed/_robotAcc/_robot 等多线程读写

        public Form1()
        {
            InitializeComponent();

            this.Load += Form1_Load;
            this.Resize += Form1_Resize;
            this.KeyPreview = true;     // 允许窗体截获按键，即使当前焦点在子控件

            this.skControl.MouseWheel += Form1_MouseWheel;
            this.skControl.MouseDown += Form1_MouseDown;
            this.skControl.MouseMove += Form1_MouseMove;
            this.skControl.MouseUp += Form1_MouseUp;

            this.skControl.PaintSurface += SkControl_PaintSurface;

            numericAcc.KeyDown += Numeric_KeyDown_OnEnter;
            numericVinit.KeyDown += Numeric_KeyDown_OnEnter;
        }

        /// <summary>
        /// 窗体加载：
        /// - 初始化偏移/初始视图数据；
        /// - 调用 Initialize() 创建各模块；
        /// - 执行加载居中；
        /// - 设置默认控制模式与默认寻路算法；
        /// - 将焦点切回窗体，确保键盘控制可用。
        /// </summary>
        private void Form1_Load(object sender, EventArgs e)
        {
            Initialize();  // 初始化
            _centerGrid.Center();    // 加载居中

            ActiveControl = null;      // 把焦点回到窗体（避免下拉框/数值框占用焦点导致按键无效）
            BeginInvoke(new Action(() => Focus()));
        }

        /// <summary>
        /// 窗口大小改变：保持当前缩放不变，重新计算 offset 实现居中。
        /// </summary>
        private void Form1_Resize(object sender, EventArgs e)
        {
            _centerGrid.Center();
        }

        /// <summary>
        /// 鼠标滚轮缩放（以鼠标所在点为缩放中心）
        /// </summary>
        private void Form1_MouseWheel(object sender, MouseEventArgs e)
        {
            _mouseWheel.Wheel(
                e,
                screen => _worldTransform.ScreenToWorld(screen.X, screen.Y));

            skControl.Invalidate();
        }

        /// <summary>
        /// 鼠标按下：
        /// - 进入拖拽平移模式（MousePan）。
        /// </summary>
        private void Form1_MouseDown(object sender, MouseEventArgs e)
        {
            _mousePan.MouseDown(e);
        }

        /// <summary>
        /// 鼠标移动：拖拽平移画布（若 MousePan 处于拖拽状态才会生效）。
        /// </summary>
        private void Form1_MouseMove(object sender, MouseEventArgs e)
        {
            _mousePan.MouseMove(e);
            skControl.Invalidate();
        }

        /// <summary>
        /// 鼠标松开：结束拖拽平移。
        /// </summary>
        private void Form1_MouseUp(object sender, MouseEventArgs e)
        {
            _mousePan.MouseUp(e);
        }

        /// <summary>
        /// Skia 绘制回调：绘制当前帧。
        /// 注意：绘制过程中会读取机器人位置/速度等共享状态，因此通过 _robotLock 保证一致性。
        /// </summary>
        private void SkControl_PaintSurface(object sender, SKPaintSurfaceEventArgs e)
        {
            SKCanvas canvas = e.Surface.Canvas;

            _drawGrid.Draw(canvas);
            _drawRobot.Draw(canvas);

            double robotX;
            double robotY;
            double robotV;
            double robotA;

            lock (_robotLock)
            {
                robotX = _robotX;
                robotY = _robotY;
                robotV = _robotSpeed;
                robotA = _robotAcc;
            }

            using (var textPaint = new SKPaint              // 绘制文本样式（机器人坐标、速度、加速度）
            {
                Color = SKColors.Black,
                IsAntialias = true,
            })
            {
                using (var font = new SKFont())
                {
                    font.Size = 15;
                    //string info = $"Scale: {_scale:F1} px/m   Offset: ({_offsetX:F0}, {_offsetY:F0})";
                    string infoRobot = $"Robot: ({robotX:F2}, {robotY:F2}), v={robotV:F2}m/s, a={robotA:F2}m/s2";
                    //canvas.DrawText(info, 10, 25, SKTextAlign.Left, font, textPaint);
                    canvas.DrawText(infoRobot, 10, 25, SKTextAlign.Left, font, textPaint);
                }
            }
        }

        /// <summary>
        /// 加速度数值变更：更新“前进时使用的加速度”。
        /// 若此时 W 正按着且不在转向中，则立即作用到当前 Acc。
        /// </summary>
        private void numericAcc_ValueChanged(object sender, EventArgs e)
        {
            lock (_robotLock)
            {
                // 数值框决定“前进时使用的加速度”
                _robotAcc = (double)((NumericUpDown)sender).Value;

                // 如果此时前进键按着且没有在转向，可以立即更新当前加速度
                if (_robot != null && _robot.IsForwardKeyDown && !_robot.IsTurning)
                {
                    _robot.Acc = _robotAcc;
                }
            }
        }

        /// <summary>
        /// 最大速度数值变更：实时更新 Robot 的 MaxSpeed（影响后续速度夹紧）。
        /// </summary>
        private void numericVmax_ValueChanged(object sender, EventArgs e)
        {
            lock (_robotLock)
            {
                _robot.MaxSpeed = (double)((NumericUpDown)sender).Value;
            }
        }

        /// <summary>
        /// NumericUpDown 在按 Enter 时主动失焦：
        /// - 保证 ValueChanged 能触发并应用参数；
        /// - 将焦点还给窗体，确保 W/A/D 键立即可用；
        /// - suppressKeyPress 防止系统“叮”的提示音。
        /// </summary>
        private void Numeric_KeyDown_OnEnter(object sender, KeyEventArgs e)
        {
            if (e.KeyCode == Keys.Enter)
            {
                this.ActiveControl = null;  // 让数值框失去焦点，触发 ValueChanged 事件
                this.Focus();                 // 焦点回到窗体，W/A/D 立刻可用
                e.Handled = true;
                e.SuppressKeyPress = true;    // 防止系统“叮”一声
            }
        }

        /// <summary>
        /// Reset 按钮：将视图恢复到初始缩放并居中显示。
        /// </summary>
        private void btnReset_Click(object sender, EventArgs e)
        {
            _centerGrid.Center();
        }
    }
}
