namespace GridDemo
{
    partial class Form1
    {
        /// <summary>
        /// 必需的设计器变量。
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// 清理所有正在使用的资源。
        /// </summary>
        /// <param name="disposing">如果应释放托管资源，为 true；否则为 false。</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows 窗体设计器生成的代码

        private void InitializeComponent()
        {
            this.splitMain = new System.Windows.Forms.SplitContainer();
            this.flowLeft = new System.Windows.Forms.FlowLayoutPanel();
            this.cmbPathAlgorithm = new System.Windows.Forms.ComboBox();
            this.panel1 = new System.Windows.Forms.Panel();
            this.btnStop = new System.Windows.Forms.Button();
            this.btnStart = new System.Windows.Forms.Button();
            this.btnResetRobot = new System.Windows.Forms.Button();
            this.numericAddRobot = new System.Windows.Forms.NumericUpDown();
            this.labelAddRobot = new System.Windows.Forms.Label();
            this.labelRobot = new System.Windows.Forms.Label();
            this.labelVinit = new System.Windows.Forms.Label();
            this.numericVinit = new System.Windows.Forms.NumericUpDown();
            this.labelAcc = new System.Windows.Forms.Label();
            this.numericAcc = new System.Windows.Forms.NumericUpDown();
            this.grpRobotStates = new System.Windows.Forms.GroupBox();
            this.lvRobotStates = new System.Windows.Forms.ListView();
            this.colId = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.colMode = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.colPos = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.colSpeed = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.colAcc = ((System.Windows.Forms.ColumnHeader)(new System.Windows.Forms.ColumnHeader()));
            this.skControl = new SkiaSharp.Views.Desktop.SKControl();
            this.panelRightTop = new System.Windows.Forms.Panel();
            this.panelObstacles = new System.Windows.Forms.Panel();
            this.labelObstacles = new System.Windows.Forms.Label();
            this.btnClearObstacle = new System.Windows.Forms.Button();
            this.btnObstacle = new System.Windows.Forms.Button();
            this.btnReset = new System.Windows.Forms.Button();
            ((System.ComponentModel.ISupportInitialize)(this.splitMain)).BeginInit();
            this.splitMain.Panel1.SuspendLayout();
            this.splitMain.Panel2.SuspendLayout();
            this.splitMain.SuspendLayout();
            this.flowLeft.SuspendLayout();
            this.panel1.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.numericAddRobot)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericVinit)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericAcc)).BeginInit();
            this.grpRobotStates.SuspendLayout();
            this.panelRightTop.SuspendLayout();
            this.panelObstacles.SuspendLayout();
            this.SuspendLayout();
            // 
            // splitMain
            // 
            this.splitMain.Dock = System.Windows.Forms.DockStyle.Fill;
            this.splitMain.FixedPanel = System.Windows.Forms.FixedPanel.Panel1;
            this.splitMain.Location = new System.Drawing.Point(0, 0);
            this.splitMain.Name = "splitMain";
            // 
            // splitMain.Panel1
            // 
            this.splitMain.Panel1.Controls.Add(this.flowLeft);
            // 
            // splitMain.Panel2
            // 
            this.splitMain.Panel2.Controls.Add(this.skControl);
            this.splitMain.Panel2.Controls.Add(this.panelRightTop);
            this.splitMain.Size = new System.Drawing.Size(1221, 774);
            this.splitMain.SplitterDistance = 230;
            this.splitMain.TabIndex = 0;
            // 
            // flowLeft
            // 
            this.flowLeft.AutoScroll = true;
            this.flowLeft.Controls.Add(this.cmbPathAlgorithm);
            this.flowLeft.Controls.Add(this.panel1);
            this.flowLeft.Controls.Add(this.grpRobotStates);
            this.flowLeft.Dock = System.Windows.Forms.DockStyle.Fill;
            this.flowLeft.FlowDirection = System.Windows.Forms.FlowDirection.TopDown;
            this.flowLeft.Location = new System.Drawing.Point(0, 0);
            this.flowLeft.Name = "flowLeft";
            this.flowLeft.Padding = new System.Windows.Forms.Padding(12);
            this.flowLeft.Size = new System.Drawing.Size(230, 774);
            this.flowLeft.TabIndex = 0;
            this.flowLeft.WrapContents = false;
            // 
            // cmbPathAlgorithm
            // 
            this.cmbPathAlgorithm.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.cmbPathAlgorithm.FormattingEnabled = true;
            this.cmbPathAlgorithm.Items.AddRange(new object[] {
            "Dijkstra",
            "A*",
            "蛇形"});
            this.cmbPathAlgorithm.Location = new System.Drawing.Point(15, 15);
            this.cmbPathAlgorithm.Name = "cmbPathAlgorithm";
            this.cmbPathAlgorithm.Size = new System.Drawing.Size(195, 26);
            this.cmbPathAlgorithm.TabIndex = 0;
            // 
            // panel1
            // 
            this.panel1.Controls.Add(this.btnStop);
            this.panel1.Controls.Add(this.btnStart);
            this.panel1.Controls.Add(this.btnResetRobot);
            this.panel1.Controls.Add(this.numericAddRobot);
            this.panel1.Controls.Add(this.labelAddRobot);
            this.panel1.Controls.Add(this.labelRobot);
            this.panel1.Controls.Add(this.labelVinit);
            this.panel1.Controls.Add(this.numericVinit);
            this.panel1.Controls.Add(this.labelAcc);
            this.panel1.Controls.Add(this.numericAcc);
            this.panel1.Location = new System.Drawing.Point(15, 47);
            this.panel1.Name = "panel1";
            this.panel1.Size = new System.Drawing.Size(195, 260);
            this.panel1.TabIndex = 1;
            // 
            // btnStop
            // 
            this.btnStop.Location = new System.Drawing.Point(108, 215);
            this.btnStop.Name = "btnStop";
            this.btnStop.Size = new System.Drawing.Size(75, 33);
            this.btnStop.TabIndex = 9;
            this.btnStop.Text = "暂停";
            this.btnStop.UseVisualStyleBackColor = true;
            this.btnStop.Click += new System.EventHandler(this.btnStop_Click);
            // 
            // btnStart
            // 
            this.btnStart.Location = new System.Drawing.Point(15, 215);
            this.btnStart.Name = "btnStart";
            this.btnStart.Size = new System.Drawing.Size(75, 33);
            this.btnStart.TabIndex = 8;
            this.btnStart.Text = "启动";
            this.btnStart.UseVisualStyleBackColor = true;
            this.btnStart.Click += new System.EventHandler(this.btnStart_Click);
            // 
            // btnResetRobot
            // 
            this.btnResetRobot.Location = new System.Drawing.Point(54, 168);
            this.btnResetRobot.Name = "btnResetRobot";
            this.btnResetRobot.Size = new System.Drawing.Size(86, 36);
            this.btnResetRobot.TabIndex = 7;
            this.btnResetRobot.Text = "重置";
            this.btnResetRobot.UseVisualStyleBackColor = true;
            this.btnResetRobot.Click += new System.EventHandler(this.btnResetRobot_Click);
            // 
            // numericAddRobot
            // 
            this.numericAddRobot.Location = new System.Drawing.Point(116, 124);
            this.numericAddRobot.Minimum = new decimal(new int[] {
            1,
            0,
            0,
            0});
            this.numericAddRobot.Name = "numericAddRobot";
            this.numericAddRobot.Size = new System.Drawing.Size(67, 28);
            this.numericAddRobot.TabIndex = 6;
            this.numericAddRobot.Value = new decimal(new int[] {
            1,
            0,
            0,
            0});
            this.numericAddRobot.ValueChanged += new System.EventHandler(this.numericAddRobot_ValueChanged);
            // 
            // labelAddRobot
            // 
            this.labelAddRobot.AutoSize = true;
            this.labelAddRobot.Location = new System.Drawing.Point(12, 128);
            this.labelAddRobot.Name = "labelAddRobot";
            this.labelAddRobot.Size = new System.Drawing.Size(98, 18);
            this.labelAddRobot.TabIndex = 0;
            this.labelAddRobot.Text = "机器人数量";
            // 
            // labelRobot
            // 
            this.labelRobot.AutoSize = true;
            this.labelRobot.Location = new System.Drawing.Point(12, 12);
            this.labelRobot.Name = "labelRobot";
            this.labelRobot.Size = new System.Drawing.Size(98, 18);
            this.labelRobot.TabIndex = 1;
            this.labelRobot.Text = "机器人设置";
            // 
            // labelVinit
            // 
            this.labelVinit.AutoSize = true;
            this.labelVinit.Location = new System.Drawing.Point(12, 50);
            this.labelVinit.Name = "labelVinit";
            this.labelVinit.Size = new System.Drawing.Size(80, 18);
            this.labelVinit.TabIndex = 2;
            this.labelVinit.Text = "最大速度";
            // 
            // numericVinit
            // 
            this.numericVinit.DecimalPlaces = 1;
            this.numericVinit.Increment = new decimal(new int[] {
            1,
            0,
            0,
            65536});
            this.numericVinit.Location = new System.Drawing.Point(116, 48);
            this.numericVinit.Name = "numericVinit";
            this.numericVinit.Size = new System.Drawing.Size(67, 28);
            this.numericVinit.TabIndex = 3;
            this.numericVinit.Value = new decimal(new int[] {
            15,
            0,
            0,
            65536});
            this.numericVinit.ValueChanged += new System.EventHandler(this.numericVmax_ValueChanged);
            // 
            // labelAcc
            // 
            this.labelAcc.AutoSize = true;
            this.labelAcc.Location = new System.Drawing.Point(12, 88);
            this.labelAcc.Name = "labelAcc";
            this.labelAcc.Size = new System.Drawing.Size(62, 18);
            this.labelAcc.TabIndex = 4;
            this.labelAcc.Text = "加速度";
            // 
            // numericAcc
            // 
            this.numericAcc.DecimalPlaces = 1;
            this.numericAcc.Increment = new decimal(new int[] {
            1,
            0,
            0,
            65536});
            this.numericAcc.Location = new System.Drawing.Point(116, 86);
            this.numericAcc.Minimum = new decimal(new int[] {
            100,
            0,
            0,
            -2147483648});
            this.numericAcc.Name = "numericAcc";
            this.numericAcc.Size = new System.Drawing.Size(67, 28);
            this.numericAcc.TabIndex = 5;
            this.numericAcc.ValueChanged += new System.EventHandler(this.numericAcc_ValueChanged);
            // 
            // grpRobotStates
            // 
            this.grpRobotStates.Controls.Add(this.lvRobotStates);
            this.grpRobotStates.Location = new System.Drawing.Point(15, 313);
            this.grpRobotStates.Name = "grpRobotStates";
            this.grpRobotStates.Size = new System.Drawing.Size(195, 430);
            this.grpRobotStates.TabIndex = 2;
            this.grpRobotStates.TabStop = false;
            this.grpRobotStates.Text = "机器人状态（双击切换模式）";
            // 
            // lvRobotStates
            // 
            this.lvRobotStates.Columns.AddRange(new System.Windows.Forms.ColumnHeader[] {
            this.colId,
            this.colMode,
            this.colPos,
            this.colSpeed,
            this.colAcc});
            this.lvRobotStates.Dock = System.Windows.Forms.DockStyle.Fill;
            this.lvRobotStates.FullRowSelect = true;
            this.lvRobotStates.GridLines = true;
            this.lvRobotStates.HideSelection = false;
            this.lvRobotStates.Location = new System.Drawing.Point(3, 24);
            this.lvRobotStates.MultiSelect = false;
            this.lvRobotStates.Name = "lvRobotStates";
            this.lvRobotStates.Size = new System.Drawing.Size(189, 403);
            this.lvRobotStates.TabIndex = 0;
            this.lvRobotStates.UseCompatibleStateImageBehavior = false;
            this.lvRobotStates.View = System.Windows.Forms.View.Details;
            // 
            // colId
            // 
            this.colId.Text = "Id";
            this.colId.Width = 25;
            // 
            // colMode
            // 
            this.colMode.Text = "模式";
            this.colMode.Width = 38;
            // 
            // colPos
            // 
            this.colPos.Text = "(X,Y)";
            this.colPos.Width = 62;
            // 
            // colSpeed
            // 
            this.colSpeed.Text = "V";
            this.colSpeed.Width = 28;
            // 
            // colAcc
            // 
            this.colAcc.Text = "A";
            this.colAcc.Width = 28;
            // 
            // skControl
            // 
            this.skControl.Dock = System.Windows.Forms.DockStyle.Fill;
            this.skControl.Location = new System.Drawing.Point(0, 70);
            this.skControl.Name = "skControl";
            this.skControl.Size = new System.Drawing.Size(987, 704);
            this.skControl.TabIndex = 8;
            // 
            // panelRightTop
            // 
            this.panelRightTop.BackColor = System.Drawing.Color.White;
            this.panelRightTop.Controls.Add(this.panelObstacles);
            this.panelRightTop.Controls.Add(this.btnReset);
            this.panelRightTop.Dock = System.Windows.Forms.DockStyle.Top;
            this.panelRightTop.Location = new System.Drawing.Point(0, 0);
            this.panelRightTop.Name = "panelRightTop";
            this.panelRightTop.Padding = new System.Windows.Forms.Padding(10);
            this.panelRightTop.Size = new System.Drawing.Size(987, 70);
            this.panelRightTop.TabIndex = 0;
            // 
            // panelObstacles
            // 
            this.panelObstacles.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right)));
            this.panelObstacles.Controls.Add(this.labelObstacles);
            this.panelObstacles.Controls.Add(this.btnClearObstacle);
            this.panelObstacles.Controls.Add(this.btnObstacle);
            this.panelObstacles.Location = new System.Drawing.Point(689, 10);
            this.panelObstacles.Name = "panelObstacles";
            this.panelObstacles.Size = new System.Drawing.Size(285, 54);
            this.panelObstacles.TabIndex = 1;
            // 
            // labelObstacles
            // 
            this.labelObstacles.AutoSize = true;
            this.labelObstacles.Location = new System.Drawing.Point(13, 20);
            this.labelObstacles.Name = "labelObstacles";
            this.labelObstacles.Size = new System.Drawing.Size(98, 18);
            this.labelObstacles.TabIndex = 0;
            this.labelObstacles.Text = "障碍物设置";
            // 
            // btnClearObstacle
            // 
            this.btnClearObstacle.Location = new System.Drawing.Point(211, 10);
            this.btnClearObstacle.Name = "btnClearObstacle";
            this.btnClearObstacle.Size = new System.Drawing.Size(61, 39);
            this.btnClearObstacle.TabIndex = 2;
            this.btnClearObstacle.Text = "清空";
            this.btnClearObstacle.UseVisualStyleBackColor = true;
            this.btnClearObstacle.Click += new System.EventHandler(this.btnClearObstacle_Click);
            // 
            // btnObstacle
            // 
            this.btnObstacle.Location = new System.Drawing.Point(141, 10);
            this.btnObstacle.Name = "btnObstacle";
            this.btnObstacle.Size = new System.Drawing.Size(54, 39);
            this.btnObstacle.TabIndex = 1;
            this.btnObstacle.Text = "关";
            this.btnObstacle.UseVisualStyleBackColor = true;
            this.btnObstacle.Click += new System.EventHandler(this.btnObstacle_Click);
            // 
            // btnReset
            // 
            this.btnReset.Location = new System.Drawing.Point(13, 10);
            this.btnReset.Name = "btnReset";
            this.btnReset.Size = new System.Drawing.Size(102, 51);
            this.btnReset.TabIndex = 0;
            this.btnReset.Text = "网格重置";
            this.btnReset.UseVisualStyleBackColor = true;
            this.btnReset.Click += new System.EventHandler(this.btnReset_Click);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 18F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1221, 774);
            this.Controls.Add(this.splitMain);
            this.Name = "Form1";
            this.Text = " RCS Ver3.4.3 吴灵丽（三期 28号）";
            this.splitMain.Panel1.ResumeLayout(false);
            this.splitMain.Panel2.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.splitMain)).EndInit();
            this.splitMain.ResumeLayout(false);
            this.flowLeft.ResumeLayout(false);
            this.panel1.ResumeLayout(false);
            this.panel1.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.numericAddRobot)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericVinit)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericAcc)).EndInit();
            this.grpRobotStates.ResumeLayout(false);
            this.panelRightTop.ResumeLayout(false);
            this.panelObstacles.ResumeLayout(false);
            this.panelObstacles.PerformLayout();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.SplitContainer splitMain;
        private System.Windows.Forms.FlowLayoutPanel flowLeft;
        private System.Windows.Forms.Panel panelRightTop;

        private SkiaSharp.Views.Desktop.SKControl skControl;
        private System.Windows.Forms.Button btnReset;
        private System.Windows.Forms.ComboBox cmbPathAlgorithm;

        private System.Windows.Forms.Panel panelObstacles;
        private System.Windows.Forms.Label labelObstacles;
        private System.Windows.Forms.Button btnObstacle;
        private System.Windows.Forms.Button btnClearObstacle;

        private System.Windows.Forms.Panel panel1;
        private System.Windows.Forms.Label labelRobot;
        private System.Windows.Forms.Label labelAddRobot;
        private System.Windows.Forms.NumericUpDown numericAddRobot;
        private System.Windows.Forms.Label labelVinit;
        private System.Windows.Forms.NumericUpDown numericVinit;
        private System.Windows.Forms.Label labelAcc;
        private System.Windows.Forms.NumericUpDown numericAcc;
        private System.Windows.Forms.Button btnResetRobot;
        private System.Windows.Forms.Button btnStart;
        private System.Windows.Forms.Button btnStop;

        private System.Windows.Forms.GroupBox grpRobotStates;
        private System.Windows.Forms.ListView lvRobotStates;
        private System.Windows.Forms.ColumnHeader colId;
        private System.Windows.Forms.ColumnHeader colMode;
        private System.Windows.Forms.ColumnHeader colPos;
        private System.Windows.Forms.ColumnHeader colSpeed;
        private System.Windows.Forms.ColumnHeader colAcc;
    }
}