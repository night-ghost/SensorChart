using System;
namespace OSD {
    partial class Compass {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing) {
            if (disposing && (components != null)) {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent() {
            this.components = new System.ComponentModel.Container();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(Compass));
            this.CMB_ComPort = new System.Windows.Forms.ComboBox();
            this.BUT_ReadOSD = new System.Windows.Forms.Button();
            this.label5 = new System.Windows.Forms.Label();
            this.cbxAutoUpdate = new System.Windows.Forms.CheckBox();
            this.cbxShowUpdateDialog = new System.Windows.Forms.CheckBox();
            this.hint = new System.Windows.Forms.ToolTip(this.components);
            this.picAcc = new System.Windows.Forms.PictureBox();
            this.picMag = new System.Windows.Forms.PictureBox();
            this.picGyr = new System.Windows.Forms.PictureBox();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.picGraphA = new System.Windows.Forms.PictureBox();
            this.picGraphM = new System.Windows.Forms.PictureBox();
            this.picGraphG = new System.Windows.Forms.PictureBox();
            this.label4 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.tAx = new System.Windows.Forms.TextBox();
            this.tAy = new System.Windows.Forms.TextBox();
            this.tAz = new System.Windows.Forms.TextBox();
            this.tMz = new System.Windows.Forms.TextBox();
            this.tMy = new System.Windows.Forms.TextBox();
            this.tMx = new System.Windows.Forms.TextBox();
            this.label8 = new System.Windows.Forms.Label();
            this.label9 = new System.Windows.Forms.Label();
            this.label10 = new System.Windows.Forms.Label();
            this.tGz = new System.Windows.Forms.TextBox();
            this.tGy = new System.Windows.Forms.TextBox();
            this.tGx = new System.Windows.Forms.TextBox();
            this.label11 = new System.Windows.Forms.Label();
            this.label12 = new System.Windows.Forms.Label();
            this.label13 = new System.Windows.Forms.Label();
            this.Reset = new System.Windows.Forms.Button();
            this.chkRaw = new System.Windows.Forms.CheckBox();
            ((System.ComponentModel.ISupportInitialize)(this.picAcc)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.picMag)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.picGyr)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.picGraphA)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.picGraphM)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.picGraphG)).BeginInit();
            this.SuspendLayout();
            // 
            // CMB_ComPort
            // 
            this.CMB_ComPort.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.CMB_ComPort.FormattingEnabled = true;
            this.CMB_ComPort.Location = new System.Drawing.Point(756, 596);
            this.CMB_ComPort.Name = "CMB_ComPort";
            this.CMB_ComPort.Size = new System.Drawing.Size(98, 21);
            this.CMB_ComPort.TabIndex = 4;
            // 
            // BUT_ReadOSD
            // 
            this.BUT_ReadOSD.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.BUT_ReadOSD.Location = new System.Drawing.Point(860, 594);
            this.BUT_ReadOSD.Name = "BUT_ReadOSD";
            this.BUT_ReadOSD.Size = new System.Drawing.Size(100, 23);
            this.BUT_ReadOSD.TabIndex = 6;
            this.BUT_ReadOSD.Text = "Connect";
            this.BUT_ReadOSD.UseVisualStyleBackColor = true;
            this.BUT_ReadOSD.Click += new System.EventHandler(this.btnConnect_Click);
            // 
            // label5
            // 
            this.label5.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(692, 599);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(58, 13);
            this.label5.TabIndex = 16;
            this.label5.Text = "Serial Port:";
            // 
            // cbxAutoUpdate
            // 
            this.cbxAutoUpdate.AutoSize = true;
            this.cbxAutoUpdate.Checked = true;
            this.cbxAutoUpdate.CheckState = System.Windows.Forms.CheckState.Checked;
            this.cbxAutoUpdate.Location = new System.Drawing.Point(7, 6);
            this.cbxAutoUpdate.Name = "cbxAutoUpdate";
            this.cbxAutoUpdate.Size = new System.Drawing.Size(160, 17);
            this.cbxAutoUpdate.TabIndex = 0;
            this.cbxAutoUpdate.Text = "Check for updates at startup";
            this.cbxAutoUpdate.UseVisualStyleBackColor = true;
            // 
            // cbxShowUpdateDialog
            // 
            this.cbxShowUpdateDialog.AutoSize = true;
            this.cbxShowUpdateDialog.Checked = true;
            this.cbxShowUpdateDialog.CheckState = System.Windows.Forms.CheckState.Checked;
            this.cbxShowUpdateDialog.Location = new System.Drawing.Point(7, 29);
            this.cbxShowUpdateDialog.Name = "cbxShowUpdateDialog";
            this.cbxShowUpdateDialog.Size = new System.Drawing.Size(157, 17);
            this.cbxShowUpdateDialog.TabIndex = 1;
            this.cbxShowUpdateDialog.Text = "Prompt for update at startup";
            this.cbxShowUpdateDialog.UseVisualStyleBackColor = true;
            // 
            // picAcc
            // 
            this.picAcc.Location = new System.Drawing.Point(25, 49);
            this.picAcc.Name = "picAcc";
            this.picAcc.Size = new System.Drawing.Size(300, 300);
            this.picAcc.TabIndex = 17;
            this.picAcc.TabStop = false;
            // 
            // picMag
            // 
            this.picMag.Location = new System.Drawing.Point(351, 49);
            this.picMag.Name = "picMag";
            this.picMag.Size = new System.Drawing.Size(300, 300);
            this.picMag.TabIndex = 18;
            this.picMag.TabStop = false;
            // 
            // picGyr
            // 
            this.picGyr.Location = new System.Drawing.Point(676, 49);
            this.picGyr.Name = "picGyr";
            this.picGyr.Size = new System.Drawing.Size(300, 300);
            this.picGyr.TabIndex = 19;
            this.picGyr.TabStop = false;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(142, 19);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(34, 13);
            this.label1.TabIndex = 20;
            this.label1.Text = "Accel";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(478, 19);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(50, 13);
            this.label2.TabIndex = 21;
            this.label2.Text = "Compass";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(697, 19);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(53, 13);
            this.label3.TabIndex = 22;
            this.label3.Text = "Sum Gyro";
            // 
            // picGraphA
            // 
            this.picGraphA.Location = new System.Drawing.Point(27, 381);
            this.picGraphA.Name = "picGraphA";
            this.picGraphA.Size = new System.Drawing.Size(298, 179);
            this.picGraphA.TabIndex = 23;
            this.picGraphA.TabStop = false;
            this.picGraphA.Click += new System.EventHandler(this.picGraphA_Click);
            // 
            // picGraphM
            // 
            this.picGraphM.Location = new System.Drawing.Point(351, 381);
            this.picGraphM.Name = "picGraphM";
            this.picGraphM.Size = new System.Drawing.Size(298, 179);
            this.picGraphM.TabIndex = 24;
            this.picGraphM.TabStop = false;
            this.picGraphM.Click += new System.EventHandler(this.picGraphM_Click);
            // 
            // picGraphG
            // 
            this.picGraphG.Location = new System.Drawing.Point(676, 381);
            this.picGraphG.Name = "picGraphG";
            this.picGraphG.Size = new System.Drawing.Size(298, 179);
            this.picGraphG.TabIndex = 25;
            this.picGraphG.TabStop = false;
            this.picGraphG.Click += new System.EventHandler(this.picGraphG_Click);
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.ForeColor = System.Drawing.Color.Red;
            this.label4.Location = new System.Drawing.Point(40, 358);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(14, 13);
            this.label4.TabIndex = 26;
            this.label4.Text = "X";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.ForeColor = System.Drawing.Color.Green;
            this.label6.Location = new System.Drawing.Point(128, 358);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(14, 13);
            this.label6.TabIndex = 27;
            this.label6.Text = "Y";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.ForeColor = System.Drawing.Color.Blue;
            this.label7.Location = new System.Drawing.Point(223, 358);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(14, 13);
            this.label7.TabIndex = 28;
            this.label7.Text = "Z";
            // 
            // tAx
            // 
            this.tAx.Location = new System.Drawing.Point(60, 355);
            this.tAx.Name = "tAx";
            this.tAx.Size = new System.Drawing.Size(53, 20);
            this.tAx.TabIndex = 29;
            // 
            // tAy
            // 
            this.tAy.Location = new System.Drawing.Point(145, 355);
            this.tAy.Name = "tAy";
            this.tAy.Size = new System.Drawing.Size(53, 20);
            this.tAy.TabIndex = 30;
            // 
            // tAz
            // 
            this.tAz.Location = new System.Drawing.Point(243, 355);
            this.tAz.Name = "tAz";
            this.tAz.Size = new System.Drawing.Size(53, 20);
            this.tAz.TabIndex = 31;
            // 
            // tMz
            // 
            this.tMz.Location = new System.Drawing.Point(578, 355);
            this.tMz.Name = "tMz";
            this.tMz.Size = new System.Drawing.Size(53, 20);
            this.tMz.TabIndex = 37;
            // 
            // tMy
            // 
            this.tMy.Location = new System.Drawing.Point(480, 355);
            this.tMy.Name = "tMy";
            this.tMy.Size = new System.Drawing.Size(53, 20);
            this.tMy.TabIndex = 36;
            // 
            // tMx
            // 
            this.tMx.Location = new System.Drawing.Point(395, 355);
            this.tMx.Name = "tMx";
            this.tMx.Size = new System.Drawing.Size(53, 20);
            this.tMx.TabIndex = 35;
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.ForeColor = System.Drawing.Color.Blue;
            this.label8.Location = new System.Drawing.Point(558, 358);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(14, 13);
            this.label8.TabIndex = 34;
            this.label8.Text = "Z";
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.ForeColor = System.Drawing.Color.Green;
            this.label9.Location = new System.Drawing.Point(463, 358);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(14, 13);
            this.label9.TabIndex = 33;
            this.label9.Text = "Y";
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.ForeColor = System.Drawing.Color.Red;
            this.label10.Location = new System.Drawing.Point(375, 358);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(14, 13);
            this.label10.TabIndex = 32;
            this.label10.Text = "X";
            // 
            // tGz
            // 
            this.tGz.Location = new System.Drawing.Point(898, 355);
            this.tGz.Name = "tGz";
            this.tGz.Size = new System.Drawing.Size(53, 20);
            this.tGz.TabIndex = 43;
            // 
            // tGy
            // 
            this.tGy.Location = new System.Drawing.Point(800, 355);
            this.tGy.Name = "tGy";
            this.tGy.Size = new System.Drawing.Size(53, 20);
            this.tGy.TabIndex = 42;
            // 
            // tGx
            // 
            this.tGx.Location = new System.Drawing.Point(715, 355);
            this.tGx.Name = "tGx";
            this.tGx.Size = new System.Drawing.Size(53, 20);
            this.tGx.TabIndex = 41;
            // 
            // label11
            // 
            this.label11.AutoSize = true;
            this.label11.ForeColor = System.Drawing.Color.Blue;
            this.label11.Location = new System.Drawing.Point(878, 358);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(14, 13);
            this.label11.TabIndex = 40;
            this.label11.Text = "Z";
            // 
            // label12
            // 
            this.label12.AutoSize = true;
            this.label12.ForeColor = System.Drawing.Color.Green;
            this.label12.Location = new System.Drawing.Point(783, 358);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(14, 13);
            this.label12.TabIndex = 39;
            this.label12.Text = "Y";
            // 
            // label13
            // 
            this.label13.AutoSize = true;
            this.label13.ForeColor = System.Drawing.Color.Red;
            this.label13.Location = new System.Drawing.Point(695, 358);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(14, 13);
            this.label13.TabIndex = 38;
            this.label13.Text = "X";
            // 
            // Reset
            // 
            this.Reset.Location = new System.Drawing.Point(881, 12);
            this.Reset.Name = "Reset";
            this.Reset.Size = new System.Drawing.Size(93, 27);
            this.Reset.TabIndex = 44;
            this.Reset.Text = "Reset to 0";
            this.Reset.UseVisualStyleBackColor = true;
            this.Reset.Click += new System.EventHandler(this.Reset_Click);
            // 
            // chkRaw
            // 
            this.chkRaw.AutoSize = true;
            this.chkRaw.CheckAlign = System.Drawing.ContentAlignment.MiddleRight;
            this.chkRaw.Location = new System.Drawing.Point(773, 18);
            this.chkRaw.Name = "chkRaw";
            this.chkRaw.Size = new System.Drawing.Size(43, 17);
            this.chkRaw.TabIndex = 45;
            this.chkRaw.Text = "raw";
            this.chkRaw.UseVisualStyleBackColor = true;
            this.chkRaw.CheckedChanged += new System.EventHandler(this.chkRaw_CheckedChanged);
            // 
            // Compass
            // 
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.None;
            this.ClientSize = new System.Drawing.Size(988, 629);
            this.Controls.Add(this.chkRaw);
            this.Controls.Add(this.Reset);
            this.Controls.Add(this.tGz);
            this.Controls.Add(this.tGy);
            this.Controls.Add(this.tGx);
            this.Controls.Add(this.label11);
            this.Controls.Add(this.label12);
            this.Controls.Add(this.label13);
            this.Controls.Add(this.tMz);
            this.Controls.Add(this.tMy);
            this.Controls.Add(this.tMx);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.tAz);
            this.Controls.Add(this.tAy);
            this.Controls.Add(this.tAx);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.picGraphG);
            this.Controls.Add(this.picGraphM);
            this.Controls.Add(this.picGraphA);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.picGyr);
            this.Controls.Add(this.picMag);
            this.Controls.Add(this.picAcc);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.BUT_ReadOSD);
            this.Controls.Add(this.CMB_ComPort);
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.Name = "Compass";
            this.Text = "Compass oientation tool";
            this.FormClosed += new System.Windows.Forms.FormClosedEventHandler(this.OSD_FormClosed);
            this.Load += new System.EventHandler(this.OSD_Load);
            ((System.ComponentModel.ISupportInitialize)(this.picAcc)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.picMag)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.picGyr)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.picGraphA)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.picGraphM)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.picGraphG)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        public System.Windows.Forms.ComboBox CMB_ComPort;

        private System.Windows.Forms.Button BUT_ReadOSD;
        //        private System.Windows.Forms.RadioButton rbtBatterymAh;
        //        private System.Windows.Forms.RadioButton rbtBatteryPercent;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.CheckBox cbxAutoUpdate;
        private System.Windows.Forms.CheckBox cbxShowUpdateDialog;
        private System.Windows.Forms.ToolTip hint;
        private System.Windows.Forms.PictureBox picAcc;
        private System.Windows.Forms.PictureBox picMag;
        private System.Windows.Forms.PictureBox picGyr;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.PictureBox picGraphA;
        private System.Windows.Forms.PictureBox picGraphM;
        private System.Windows.Forms.PictureBox picGraphG;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.TextBox tAx;
        private System.Windows.Forms.TextBox tAy;
        private System.Windows.Forms.TextBox tAz;
        private System.Windows.Forms.TextBox tMz;
        private System.Windows.Forms.TextBox tMy;
        private System.Windows.Forms.TextBox tMx;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.TextBox tGz;
        private System.Windows.Forms.TextBox tGy;
        private System.Windows.Forms.TextBox tGx;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.Button Reset;
        private System.Windows.Forms.CheckBox chkRaw;
    }

}

