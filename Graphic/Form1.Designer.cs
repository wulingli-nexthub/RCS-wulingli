namespace Graphic
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

        /// <summary>
        /// 设计器支持所需的方法 - 不要修改
        /// 使用代码编辑器修改此方法的内容。
        /// </summary>
        private void InitializeComponent()
        {
            this.btnReset = new System.Windows.Forms.Button();
            this.labelAcc = new System.Windows.Forms.Label();
            this.numericAcc = new System.Windows.Forms.NumericUpDown();
            this.labelVinit = new System.Windows.Forms.Label();
            this.numericVmax = new System.Windows.Forms.NumericUpDown();
            this.cmbMode = new System.Windows.Forms.ComboBox();
            ((System.ComponentModel.ISupportInitialize)(this.numericAcc)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericVmax)).BeginInit();
            this.SuspendLayout();
            // 
            // btnReset
            // 
            this.btnReset.Location = new System.Drawing.Point(25, 69);
            this.btnReset.Name = "btnReset";
            this.btnReset.Size = new System.Drawing.Size(95, 51);
            this.btnReset.TabIndex = 0;
            this.btnReset.Text = "网格重置";
            this.btnReset.UseVisualStyleBackColor = true;
            this.btnReset.Click += new System.EventHandler(this.btnReset_Click);
            // 
            // labelAcc
            // 
            this.labelAcc.AutoSize = true;
            this.labelAcc.Location = new System.Drawing.Point(22, 279);
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
            this.numericAcc.Location = new System.Drawing.Point(25, 314);
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
            this.labelVinit.Location = new System.Drawing.Point(22, 191);
            this.labelVinit.Name = "labelVinit";
            this.labelVinit.Size = new System.Drawing.Size(80, 18);
            this.labelVinit.TabIndex = 5;
            this.labelVinit.Text = "最大速度";
            // 
            // numericVmax
            // 
            this.numericVmax.DecimalPlaces = 1;
            this.numericVmax.Increment = new decimal(new int[] {
            1,
            0,
            0,
            65536});
            this.numericVmax.Location = new System.Drawing.Point(25, 233);
            this.numericVmax.Name = "numericVmax";
            this.numericVmax.Size = new System.Drawing.Size(120, 28);
            this.numericVmax.TabIndex = 6;
            this.numericVmax.Value = new decimal(new int[] {
            15,
            0,
            0,
            65536});
            this.numericVmax.ValueChanged += new System.EventHandler(this.numericVmax_ValueChanged);
            // 
            // cmbMode
            // 
            this.cmbMode.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.cmbMode.FormattingEnabled = true;
            this.cmbMode.Items.AddRange(new object[] {
            "手动控制",
            "自动巡航"});
            this.cmbMode.Location = new System.Drawing.Point(25, 145);
            this.cmbMode.Name = "cmbMode";
            this.cmbMode.Size = new System.Drawing.Size(121, 26);
            this.cmbMode.TabIndex = 7;
            this.cmbMode.SelectedIndexChanged += new System.EventHandler(this.cmbMode_SelectedIndexChanged);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 18F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1221, 774);
            this.Controls.Add(this.cmbMode);
            this.Controls.Add(this.numericVmax);
            this.Controls.Add(this.labelVinit);
            this.Controls.Add(this.numericAcc);
            this.Controls.Add(this.labelAcc);
            this.Controls.Add(this.btnReset);
            this.Name = "Form1";
            this.Text = " RCS Ver3.2.2 吴灵丽";
            ((System.ComponentModel.ISupportInitialize)(this.numericAcc)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericVmax)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button btnReset;
        private System.Windows.Forms.Label labelAcc;
        private System.Windows.Forms.NumericUpDown numericAcc;
        private System.Windows.Forms.Label labelVinit;
        private System.Windows.Forms.NumericUpDown numericVmax;
        private System.Windows.Forms.ComboBox cmbMode;
    }
}

