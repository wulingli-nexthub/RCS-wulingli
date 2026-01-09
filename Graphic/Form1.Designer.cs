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
            ((System.ComponentModel.ISupportInitialize)(this.numericAcc)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericVinit)).BeginInit();
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
            this.labelAcc.Location = new System.Drawing.Point(22, 298);
            this.labelAcc.Name = "labelAcc";
            this.labelAcc.Size = new System.Drawing.Size(98, 18);
            this.labelAcc.TabIndex = 2;
            this.labelAcc.Text = "调节加速度";
            // 
            // numericAcc
            // 
            this.numericAcc.DecimalPlaces = 1;
            this.numericAcc.Increment = new decimal(new int[] {
            1,
            0,
            0,
            65536});
            this.numericAcc.Location = new System.Drawing.Point(25, 333);
            this.numericAcc.Minimum = new decimal(new int[] {
            100,
            0,
            0,
            -2147483648});
            this.numericAcc.Name = "numericAcc";
            this.numericAcc.Size = new System.Drawing.Size(120, 28);
            this.numericAcc.TabIndex = 4;
            this.numericAcc.ValueChanged += new System.EventHandler(this.numericAcc_ValueChanged);
            // 
            // labelVinit
            // 
            this.labelVinit.AutoSize = true;
            this.labelVinit.Location = new System.Drawing.Point(22, 210);
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
            this.numericVinit.Location = new System.Drawing.Point(25, 252);
            this.numericVinit.Name = "numericVinit";
            this.numericVinit.Size = new System.Drawing.Size(120, 28);
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
            this.cmbPathAlgorithm.Location = new System.Drawing.Point(24, 401);
            this.cmbPathAlgorithm.Name = "cmbPathAlgorithm";
            this.cmbPathAlgorithm.Size = new System.Drawing.Size(121, 26);
            this.cmbPathAlgorithm.TabIndex = 9;
            // 
            // btnObstacle
            // 
            this.btnObstacle.Location = new System.Drawing.Point(493, 18);
            this.btnObstacle.Name = "btnObstacle";
            this.btnObstacle.Size = new System.Drawing.Size(150, 39);
            this.btnObstacle.TabIndex = 10;
            this.btnObstacle.Text = "设置障碍物：关";
            this.btnObstacle.UseVisualStyleBackColor = true;
            this.btnObstacle.Click += new System.EventHandler(this.btnObstacle_Click);
            // 
            // btnClearObstacle
            // 
            this.btnClearObstacle.Location = new System.Drawing.Point(700, 18);
            this.btnClearObstacle.Name = "btnClearObstacle";
            this.btnClearObstacle.Size = new System.Drawing.Size(118, 39);
            this.btnClearObstacle.TabIndex = 11;
            this.btnClearObstacle.Text = "清空障碍物";
            this.btnClearObstacle.UseVisualStyleBackColor = true;
            this.btnClearObstacle.Click += new System.EventHandler(this.btnClearObstacle_Click);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 18F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1221, 774);
            this.Controls.Add(this.btnClearObstacle);
            this.Controls.Add(this.btnObstacle);
            this.Controls.Add(this.cmbPathAlgorithm);
            this.Controls.Add(this.cmbChooseModel);
            this.Controls.Add(this.numericVinit);
            this.Controls.Add(this.labelVinit);
            this.Controls.Add(this.numericAcc);
            this.Controls.Add(this.labelAcc);
            this.Controls.Add(this.btnReset);
            this.Controls.Add(this.skControl);
            this.Name = "Form1";
            this.Text = " RCS Ver 2.3.2 吴灵丽（三期 28号）";
            ((System.ComponentModel.ISupportInitialize)(this.numericAcc)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericVinit)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

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
    }
}

