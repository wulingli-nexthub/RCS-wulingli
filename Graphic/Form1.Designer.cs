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
            this.skControl = new SkiaSharp.Views.Desktop.SKControl();
            this.btnReset = new System.Windows.Forms.Button();
            this.labelAcc = new System.Windows.Forms.Label();
            this.numericAcc = new System.Windows.Forms.NumericUpDown();
            this.labelVinit = new System.Windows.Forms.Label();
            this.numericVinit = new System.Windows.Forms.NumericUpDown();
            this.cmbChooseModel = new System.Windows.Forms.ComboBox();
            this.cmbPathAlgorithm = new System.Windows.Forms.ComboBox();
            this.btnObstacle = new System.Windows.Forms.Button();
            this.btnClearObstacle = new System.Windows.Forms.Button();
            this.panelObstacles = new System.Windows.Forms.Panel();
            this.labelObstacles = new System.Windows.Forms.Label();
            this.panel1 = new System.Windows.Forms.Panel();
            this.labelRobot = new System.Windows.Forms.Label();
            this.labelAddRobot = new System.Windows.Forms.Label();
            this.numericAddRobot = new System.Windows.Forms.NumericUpDown();
            this.btnResetRobot = new System.Windows.Forms.Button();
            ((System.ComponentModel.ISupportInitialize)(this.numericAcc)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericVinit)).BeginInit();
            this.panelObstacles.SuspendLayout();
            this.panel1.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.numericAddRobot)).BeginInit();
            this.SuspendLayout();
            // 
            // skControl
            // 
            this.skControl.Dock = System.Windows.Forms.DockStyle.Fill;
            this.skControl.Location = new System.Drawing.Point(0, 0);
            this.skControl.Name = "skControl";
            this.skControl.Size = new System.Drawing.Size(1221, 774);
            this.skControl.TabIndex = 8;
            // 
            // btnReset
            // 
            this.btnReset.Location = new System.Drawing.Point(25, 69);
            this.btnReset.Name = "btnReset";
            this.btnReset.Size = new System.Drawing.Size(102, 51);
            this.btnReset.TabIndex = 0;
            this.btnReset.Text = "网格重置";
            this.btnReset.UseVisualStyleBackColor = true;
            this.btnReset.Click += new System.EventHandler(this.btnReset_Click);
            // 
            // labelAcc
            // 
            this.labelAcc.AutoSize = true;
            this.labelAcc.Location = new System.Drawing.Point(10, 82);
            this.labelAcc.Name = "labelAcc";
            this.labelAcc.Size = new System.Drawing.Size(62, 18);
            this.labelAcc.TabIndex = 2;
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
            this.numericAcc.Location = new System.Drawing.Point(108, 80);
            this.numericAcc.Minimum = new decimal(new int[] {
            100,
            0,
            0,
            -2147483648});
            this.numericAcc.Name = "numericAcc";
            this.numericAcc.Size = new System.Drawing.Size(64, 28);
            this.numericAcc.TabIndex = 4;
            this.numericAcc.ValueChanged += new System.EventHandler(this.numericAcc_ValueChanged);
            // 
            // labelVinit
            // 
            this.labelVinit.AutoSize = true;
            this.labelVinit.Location = new System.Drawing.Point(3, 41);
            this.labelVinit.Name = "labelVinit";
            this.labelVinit.Size = new System.Drawing.Size(80, 18);
            this.labelVinit.TabIndex = 5;
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
            this.numericVinit.Location = new System.Drawing.Point(108, 39);
            this.numericVinit.Name = "numericVinit";
            this.numericVinit.Size = new System.Drawing.Size(64, 28);
            this.numericVinit.TabIndex = 6;
            this.numericVinit.Value = new decimal(new int[] {
            15,
            0,
            0,
            65536});
            this.numericVinit.ValueChanged += new System.EventHandler(this.numericVmax_ValueChanged);
            // 
            // cmbChooseModel
            // 
            this.cmbChooseModel.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.cmbChooseModel.FormattingEnabled = true;
            this.cmbChooseModel.Items.AddRange(new object[] {
            "手动控制",
            "自动巡航"});
            this.cmbChooseModel.Location = new System.Drawing.Point(25, 159);
            this.cmbChooseModel.Name = "cmbChooseModel";
            this.cmbChooseModel.Size = new System.Drawing.Size(121, 26);
            this.cmbChooseModel.TabIndex = 7;
            // 
            // cmbPathAlgorithm
            // 
            this.cmbPathAlgorithm.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.cmbPathAlgorithm.FormattingEnabled = true;
            this.cmbPathAlgorithm.Items.AddRange(new object[] {
            "Dijkstra",
            "A*",
            "蛇形"});
            this.cmbPathAlgorithm.Location = new System.Drawing.Point(25, 210);
            this.cmbPathAlgorithm.Name = "cmbPathAlgorithm";
            this.cmbPathAlgorithm.Size = new System.Drawing.Size(121, 26);
            this.cmbPathAlgorithm.TabIndex = 9;
            // 
            // btnObstacle
            // 
            this.btnObstacle.Location = new System.Drawing.Point(141, 10);
            this.btnObstacle.Name = "btnObstacle";
            this.btnObstacle.Size = new System.Drawing.Size(54, 39);
            this.btnObstacle.TabIndex = 10;
            this.btnObstacle.Text = "关";
            this.btnObstacle.UseVisualStyleBackColor = true;
            this.btnObstacle.Click += new System.EventHandler(this.btnObstacle_Click);
            // 
            // btnClearObstacle
            // 
            this.btnClearObstacle.Location = new System.Drawing.Point(211, 10);
            this.btnClearObstacle.Name = "btnClearObstacle";
            this.btnClearObstacle.Size = new System.Drawing.Size(61, 39);
            this.btnClearObstacle.TabIndex = 11;
            this.btnClearObstacle.Text = "清空";
            this.btnClearObstacle.UseVisualStyleBackColor = true;
            this.btnClearObstacle.Click += new System.EventHandler(this.btnClearObstacle_Click);
            // 
            // panelObstacles
            // 
            this.panelObstacles.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right)));
            this.panelObstacles.Controls.Add(this.labelObstacles);
            this.panelObstacles.Controls.Add(this.btnClearObstacle);
            this.panelObstacles.Controls.Add(this.btnObstacle);
            this.panelObstacles.Location = new System.Drawing.Point(924, 12);
            this.panelObstacles.Name = "panelObstacles";
            this.panelObstacles.Size = new System.Drawing.Size(285, 54);
            this.panelObstacles.TabIndex = 12;
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
            // panel1
            // 
            this.panel1.Controls.Add(this.btnResetRobot);
            this.panel1.Controls.Add(this.numericAddRobot);
            this.panel1.Controls.Add(this.labelAddRobot);
            this.panel1.Controls.Add(this.labelRobot);
            this.panel1.Controls.Add(this.labelVinit);
            this.panel1.Controls.Add(this.numericVinit);
            this.panel1.Controls.Add(this.labelAcc);
            this.panel1.Controls.Add(this.numericAcc);
            this.panel1.Location = new System.Drawing.Point(12, 271);
            this.panel1.Name = "panel1";
            this.panel1.Size = new System.Drawing.Size(178, 210);
            this.panel1.TabIndex = 13;
            // 
            // labelRobot
            // 
            this.labelRobot.AutoSize = true;
            this.labelRobot.Location = new System.Drawing.Point(28, 11);
            this.labelRobot.Name = "labelRobot";
            this.labelRobot.Size = new System.Drawing.Size(98, 18);
            this.labelRobot.TabIndex = 0;
            this.labelRobot.Text = "机器人设置";
            // 
            // labelAddRobot
            // 
            this.labelAddRobot.AutoSize = true;
            this.labelAddRobot.Location = new System.Drawing.Point(3, 119);
            this.labelAddRobot.Name = "labelAddRobot";
            this.labelAddRobot.Size = new System.Drawing.Size(98, 18);
            this.labelAddRobot.TabIndex = 7;
            this.labelAddRobot.Text = "机器人数量";
            // 
            // numericAddRobot
            // 
            this.numericAddRobot.Location = new System.Drawing.Point(108, 114);
            this.numericAddRobot.Minimum = new decimal(new int[] {
            1,
            0,
            0,
            0});
            this.numericAddRobot.Name = "numericAddRobot";
            this.numericAddRobot.Size = new System.Drawing.Size(64, 28);
            this.numericAddRobot.TabIndex = 8;
            this.numericAddRobot.Value = new decimal(new int[] {
            1,
            0,
            0,
            0});
            this.numericAddRobot.ValueChanged += new System.EventHandler(this.numericAddRobot_ValueChanged);
            // 
            // btnResetRobot
            // 
            this.btnResetRobot.Location = new System.Drawing.Point(40, 157);
            this.btnResetRobot.Name = "btnResetRobot";
            this.btnResetRobot.Size = new System.Drawing.Size(86, 36);
            this.btnResetRobot.TabIndex = 9;
            this.btnResetRobot.Text = "重置";
            this.btnResetRobot.UseVisualStyleBackColor = true;
            this.btnResetRobot.Click += new System.EventHandler(this.btnResetRobot_Click);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 18F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1221, 774);
            this.Controls.Add(this.panel1);
            this.Controls.Add(this.panelObstacles);
            this.Controls.Add(this.cmbPathAlgorithm);
            this.Controls.Add(this.cmbChooseModel);
            this.Controls.Add(this.btnReset);
            this.Controls.Add(this.skControl);
            this.Name = "Form1";
            this.Text = " RCS Ver 2.3.8 吴灵丽（三期 28号）";
            ((System.ComponentModel.ISupportInitialize)(this.numericAcc)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericVinit)).EndInit();
            this.panelObstacles.ResumeLayout(false);
            this.panelObstacles.PerformLayout();
            this.panel1.ResumeLayout(false);
            this.panel1.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.numericAddRobot)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion

        private SkiaSharp.Views.Desktop.SKControl skControl;
        private System.Windows.Forms.Button btnReset;
        private System.Windows.Forms.Label labelAcc;
        private System.Windows.Forms.NumericUpDown numericAcc;
        private System.Windows.Forms.Label labelVinit;
        private System.Windows.Forms.NumericUpDown numericVinit;
        private System.Windows.Forms.ComboBox cmbChooseModel;
        private System.Windows.Forms.ComboBox cmbPathAlgorithm;
        private System.Windows.Forms.Button btnObstacle;
        private System.Windows.Forms.Button btnClearObstacle;
        private System.Windows.Forms.Panel panelObstacles;
        private System.Windows.Forms.Label labelObstacles;
        private System.Windows.Forms.Panel panel1;
        private System.Windows.Forms.Label labelRobot;
        private System.Windows.Forms.Label labelAddRobot;
        private System.Windows.Forms.Button btnResetRobot;
        private System.Windows.Forms.NumericUpDown numericAddRobot;
    }
}

