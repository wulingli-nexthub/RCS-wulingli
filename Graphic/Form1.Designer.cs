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
            this.numericVinit = new System.Windows.Forms.NumericUpDown();
            ((System.ComponentModel.ISupportInitialize)(this.numericAcc)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericVinit)).BeginInit();
            this.SuspendLayout();
            // 
            // btnReset
            // 
            this.btnReset.Location = new System.Drawing.Point(25, 69);
            this.btnReset.Name = "btnReset";
            this.btnReset.Size = new System.Drawing.Size(77, 51);
            this.btnReset.TabIndex = 0;
            this.btnReset.Text = "重置";
            this.btnReset.UseVisualStyleBackColor = true;
            this.btnReset.Click += new System.EventHandler(this.btnReset_Click);
            // 
            // labelAcc
            // 
            this.labelAcc.AutoSize = true;
            this.labelAcc.Location = new System.Drawing.Point(22, 227);
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
            this.numericAcc.Location = new System.Drawing.Point(25, 262);
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
            this.labelVinit.Location = new System.Drawing.Point(22, 139);
            this.labelVinit.Name = "labelVinit";
            this.labelVinit.Size = new System.Drawing.Size(80, 18);
            this.labelVinit.TabIndex = 5;
            this.labelVinit.Text = "调节速度";
            // 
            // numericVinit
            // 
            this.numericVinit.DecimalPlaces = 1;
            this.numericVinit.Increment = new decimal(new int[] {
            1,
            0,
            0,
            65536});
            this.numericVinit.Location = new System.Drawing.Point(25, 181);
            this.numericVinit.Name = "numericVinit";
            this.numericVinit.Size = new System.Drawing.Size(120, 28);
            this.numericVinit.TabIndex = 6;
            this.numericVinit.Value = new decimal(new int[] {
            15,
            0,
            0,
            65536});
            this.numericVinit.ValueChanged += new System.EventHandler(this.numericVinit_ValueChanged);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(9F, 18F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1221, 774);
            this.Controls.Add(this.numericVinit);
            this.Controls.Add(this.labelVinit);
            this.Controls.Add(this.numericAcc);
            this.Controls.Add(this.labelAcc);
            this.Controls.Add(this.btnReset);
            this.Name = "Form1";
            this.Text = " RCS Ver2.5.0 吴灵丽";
            ((System.ComponentModel.ISupportInitialize)(this.numericAcc)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.numericVinit)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button btnReset;
        private System.Windows.Forms.Label labelAcc;
        private System.Windows.Forms.NumericUpDown numericAcc;
        private System.Windows.Forms.Label labelVinit;
        private System.Windows.Forms.NumericUpDown numericVinit;
    }
}

