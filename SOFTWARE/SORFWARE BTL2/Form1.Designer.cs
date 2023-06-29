﻿
namespace SORFWARE_BTL2
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.label12 = new System.Windows.Forms.Label();
            this.txtReadSerial = new System.Windows.Forms.TextBox();
            this.txbRecieError = new System.Windows.Forms.TextBox();
            this.groupBox2 = new System.Windows.Forms.GroupBox();
            this.btnOpen = new System.Windows.Forms.Button();
            this.ccbBaudrate = new System.Windows.Forms.ComboBox();
            this.ccbComName = new System.Windows.Forms.ComboBox();
            this.serialPort1 = new System.IO.Ports.SerialPort(this.components);
            this.groupBox3 = new System.Windows.Forms.GroupBox();
            this.btnSelect = new System.Windows.Forms.Button();
            this.label11 = new System.Windows.Forms.Label();
            this.txbSetpoint = new System.Windows.Forms.TextBox();
            this.btnReset = new System.Windows.Forms.Button();
            this.txbTest = new System.Windows.Forms.TextBox();
            this.btnGetdata = new System.Windows.Forms.Button();
            this.btnTuning = new System.Windows.Forms.Button();
            this.btnSend = new System.Windows.Forms.Button();
            this.label3 = new System.Windows.Forms.Label();
            this.label1 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.txbKd = new System.Windows.Forms.TextBox();
            this.txbKi = new System.Windows.Forms.TextBox();
            this.txbKp = new System.Windows.Forms.TextBox();
            this.zedGraphPID = new ZedGraph.ZedGraphControl();
            this.tabPage1 = new System.Windows.Forms.TabPage();
            this.groupBox5 = new System.Windows.Forms.GroupBox();
            this.zedGraphAcc = new ZedGraph.ZedGraphControl();
            this.groupBox4 = new System.Windows.Forms.GroupBox();
            this.zedGraphVelocity = new ZedGraph.ZedGraphControl();
            this.tabPage2 = new System.Windows.Forms.TabPage();
            this.groupBox8 = new System.Windows.Forms.GroupBox();
            this.btnConnectUSB = new System.Windows.Forms.Button();
            this.label15 = new System.Windows.Forms.Label();
            this.txbVender = new System.Windows.Forms.TextBox();
            this.label14 = new System.Windows.Forms.Label();
            this.txbProductID = new System.Windows.Forms.TextBox();
            this.label13 = new System.Windows.Forms.Label();
            this.txbVenderID = new System.Windows.Forms.TextBox();
            this.groupBox7 = new System.Windows.Forms.GroupBox();
            this.txbRms = new System.Windows.Forms.TextBox();
            this.label10 = new System.Windows.Forms.Label();
            this.button4 = new System.Windows.Forms.Button();
            this.btnRun = new System.Windows.Forms.Button();
            this.btnSendSeting = new System.Windows.Forms.Button();
            this.btnDraw = new System.Windows.Forms.Button();
            this.label9 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label4 = new System.Windows.Forms.Label();
            this.txbAccMax = new System.Windows.Forms.TextBox();
            this.txbVelMax = new System.Windows.Forms.TextBox();
            this.txbPosMax = new System.Windows.Forms.TextBox();
            this.groupBox6 = new System.Windows.Forms.GroupBox();
            this.zedGraphPosRef = new ZedGraph.ZedGraphControl();
            this.tabControl1 = new System.Windows.Forms.TabControl();
            this.groupBox1.SuspendLayout();
            this.groupBox2.SuspendLayout();
            this.groupBox3.SuspendLayout();
            this.tabPage1.SuspendLayout();
            this.groupBox5.SuspendLayout();
            this.groupBox4.SuspendLayout();
            this.tabPage2.SuspendLayout();
            this.groupBox8.SuspendLayout();
            this.groupBox7.SuspendLayout();
            this.groupBox6.SuspendLayout();
            this.tabControl1.SuspendLayout();
            this.SuspendLayout();
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.label12);
            this.groupBox1.Controls.Add(this.txtReadSerial);
            this.groupBox1.Controls.Add(this.txbRecieError);
            this.groupBox1.Location = new System.Drawing.Point(12, 23);
            this.groupBox1.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Padding = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox1.Size = new System.Drawing.Size(547, 238);
            this.groupBox1.TabIndex = 2;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "RECEIVE DATA";
            // 
            // label12
            // 
            this.label12.AutoSize = true;
            this.label12.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label12.ForeColor = System.Drawing.SystemColors.HotTrack;
            this.label12.Location = new System.Drawing.Point(202, 107);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(133, 17);
            this.label12.TabIndex = 11;
            this.label12.Text = "ERROR/SET/ACT";
            // 
            // txtReadSerial
            // 
            this.txtReadSerial.Location = new System.Drawing.Point(5, 21);
            this.txtReadSerial.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.txtReadSerial.Multiline = true;
            this.txtReadSerial.Name = "txtReadSerial";
            this.txtReadSerial.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
            this.txtReadSerial.Size = new System.Drawing.Size(535, 80);
            this.txtReadSerial.TabIndex = 0;
            // 
            // txbRecieError
            // 
            this.txbRecieError.Location = new System.Drawing.Point(5, 132);
            this.txbRecieError.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.txbRecieError.Multiline = true;
            this.txbRecieError.Name = "txbRecieError";
            this.txbRecieError.ScrollBars = System.Windows.Forms.ScrollBars.Both;
            this.txbRecieError.Size = new System.Drawing.Size(535, 97);
            this.txbRecieError.TabIndex = 10;
            // 
            // groupBox2
            // 
            this.groupBox2.Controls.Add(this.btnOpen);
            this.groupBox2.Controls.Add(this.ccbBaudrate);
            this.groupBox2.Controls.Add(this.ccbComName);
            this.groupBox2.Location = new System.Drawing.Point(565, 23);
            this.groupBox2.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox2.Name = "groupBox2";
            this.groupBox2.Padding = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox2.Size = new System.Drawing.Size(203, 238);
            this.groupBox2.TabIndex = 3;
            this.groupBox2.TabStop = false;
            this.groupBox2.Text = "INFOR";
            // 
            // btnOpen
            // 
            this.btnOpen.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnOpen.ForeColor = System.Drawing.SystemColors.HotTrack;
            this.btnOpen.Location = new System.Drawing.Point(68, 176);
            this.btnOpen.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.btnOpen.Name = "btnOpen";
            this.btnOpen.Size = new System.Drawing.Size(75, 23);
            this.btnOpen.TabIndex = 2;
            this.btnOpen.Text = "Open";
            this.btnOpen.UseVisualStyleBackColor = true;
            this.btnOpen.Click += new System.EventHandler(this.btnOpen_Click);
            // 
            // ccbBaudrate
            // 
            this.ccbBaudrate.FormattingEnabled = true;
            this.ccbBaudrate.Items.AddRange(new object[] {
            "9600",
            "4800",
            "19200",
            "115200"});
            this.ccbBaudrate.Location = new System.Drawing.Point(45, 117);
            this.ccbBaudrate.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.ccbBaudrate.Name = "ccbBaudrate";
            this.ccbBaudrate.Size = new System.Drawing.Size(121, 24);
            this.ccbBaudrate.TabIndex = 1;
            // 
            // ccbComName
            // 
            this.ccbComName.FormattingEnabled = true;
            this.ccbComName.Location = new System.Drawing.Point(45, 55);
            this.ccbComName.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.ccbComName.Name = "ccbComName";
            this.ccbComName.Size = new System.Drawing.Size(121, 24);
            this.ccbComName.TabIndex = 0;
            // 
            // serialPort1
            // 
            this.serialPort1.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(this.serialPort1_DataReceived);
            // 
            // groupBox3
            // 
            this.groupBox3.Controls.Add(this.btnSelect);
            this.groupBox3.Controls.Add(this.label11);
            this.groupBox3.Controls.Add(this.txbSetpoint);
            this.groupBox3.Controls.Add(this.btnReset);
            this.groupBox3.Controls.Add(this.txbTest);
            this.groupBox3.Controls.Add(this.btnGetdata);
            this.groupBox3.Controls.Add(this.btnTuning);
            this.groupBox3.Controls.Add(this.btnSend);
            this.groupBox3.Controls.Add(this.label3);
            this.groupBox3.Controls.Add(this.label1);
            this.groupBox3.Controls.Add(this.label2);
            this.groupBox3.Controls.Add(this.txbKd);
            this.groupBox3.Controls.Add(this.txbKi);
            this.groupBox3.Controls.Add(this.txbKp);
            this.groupBox3.Location = new System.Drawing.Point(12, 267);
            this.groupBox3.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox3.Name = "groupBox3";
            this.groupBox3.Padding = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox3.Size = new System.Drawing.Size(755, 198);
            this.groupBox3.TabIndex = 4;
            this.groupBox3.TabStop = false;
            this.groupBox3.Text = "PID";
            // 
            // btnSelect
            // 
            this.btnSelect.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnSelect.ForeColor = System.Drawing.SystemColors.HotTrack;
            this.btnSelect.Location = new System.Drawing.Point(591, 117);
            this.btnSelect.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.btnSelect.Name = "btnSelect";
            this.btnSelect.Size = new System.Drawing.Size(100, 50);
            this.btnSelect.TabIndex = 9;
            this.btnSelect.Text = "SELECT";
            this.btnSelect.UseVisualStyleBackColor = true;
            this.btnSelect.Click += new System.EventHandler(this.btnSelect_Click);
            // 
            // label11
            // 
            this.label11.AutoSize = true;
            this.label11.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label11.ForeColor = System.Drawing.Color.Red;
            this.label11.Location = new System.Drawing.Point(597, 40);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(90, 17);
            this.label11.TabIndex = 8;
            this.label11.Text = "SET POINT";
            // 
            // txbSetpoint
            // 
            this.txbSetpoint.Location = new System.Drawing.Point(591, 68);
            this.txbSetpoint.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.txbSetpoint.Name = "txbSetpoint";
            this.txbSetpoint.Size = new System.Drawing.Size(100, 22);
            this.txbSetpoint.TabIndex = 7;
            // 
            // btnReset
            // 
            this.btnReset.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnReset.ForeColor = System.Drawing.SystemColors.HotTrack;
            this.btnReset.Location = new System.Drawing.Point(105, 162);
            this.btnReset.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.btnReset.Name = "btnReset";
            this.btnReset.Size = new System.Drawing.Size(75, 23);
            this.btnReset.TabIndex = 5;
            this.btnReset.Text = "RESET";
            this.btnReset.UseVisualStyleBackColor = true;
            this.btnReset.Click += new System.EventHandler(this.btnReset_Click_1);
            // 
            // txbTest
            // 
            this.txbTest.Location = new System.Drawing.Point(105, 135);
            this.txbTest.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.txbTest.Multiline = true;
            this.txbTest.Name = "txbTest";
            this.txbTest.Size = new System.Drawing.Size(75, 22);
            this.txbTest.TabIndex = 6;
            // 
            // btnGetdata
            // 
            this.btnGetdata.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnGetdata.ForeColor = System.Drawing.SystemColors.HotTrack;
            this.btnGetdata.Location = new System.Drawing.Point(321, 121);
            this.btnGetdata.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.btnGetdata.Name = "btnGetdata";
            this.btnGetdata.Size = new System.Drawing.Size(148, 50);
            this.btnGetdata.TabIndex = 2;
            this.btnGetdata.Text = "DRAW";
            this.btnGetdata.UseVisualStyleBackColor = true;
            this.btnGetdata.Click += new System.EventHandler(this.btnGetdata_Click);
            // 
            // btnTuning
            // 
            this.btnTuning.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnTuning.ForeColor = System.Drawing.SystemColors.HotTrack;
            this.btnTuning.Location = new System.Drawing.Point(411, 43);
            this.btnTuning.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.btnTuning.Name = "btnTuning";
            this.btnTuning.Size = new System.Drawing.Size(148, 50);
            this.btnTuning.TabIndex = 2;
            this.btnTuning.Text = "TUNING";
            this.btnTuning.UseVisualStyleBackColor = true;
            this.btnTuning.Click += new System.EventHandler(this.btnTuning_Click);
            // 
            // btnSend
            // 
            this.btnSend.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnSend.ForeColor = System.Drawing.SystemColors.HotTrack;
            this.btnSend.Location = new System.Drawing.Point(230, 43);
            this.btnSend.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.btnSend.Name = "btnSend";
            this.btnSend.Size = new System.Drawing.Size(148, 50);
            this.btnSend.TabIndex = 2;
            this.btnSend.Text = "SEND";
            this.btnSend.UseVisualStyleBackColor = true;
            this.btnSend.Click += new System.EventHandler(this.btnSend_Click);
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label3.ForeColor = System.Drawing.Color.Maroon;
            this.label3.Location = new System.Drawing.Point(19, 98);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(29, 17);
            this.label3.TabIndex = 1;
            this.label3.Text = "KD";
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label1.ForeColor = System.Drawing.Color.Maroon;
            this.label1.Location = new System.Drawing.Point(19, 43);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(28, 17);
            this.label1.TabIndex = 1;
            this.label1.Text = "KP";
            this.label1.Click += new System.EventHandler(this.label2_Click);
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label2.ForeColor = System.Drawing.Color.Maroon;
            this.label2.Location = new System.Drawing.Point(19, 70);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(22, 17);
            this.label2.TabIndex = 1;
            this.label2.Text = "KI";
            this.label2.Click += new System.EventHandler(this.label2_Click);
            // 
            // txbKd
            // 
            this.txbKd.Location = new System.Drawing.Point(92, 96);
            this.txbKd.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.txbKd.Name = "txbKd";
            this.txbKd.Size = new System.Drawing.Size(100, 22);
            this.txbKd.TabIndex = 0;
            // 
            // txbKi
            // 
            this.txbKi.Location = new System.Drawing.Point(92, 68);
            this.txbKi.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.txbKi.Name = "txbKi";
            this.txbKi.Size = new System.Drawing.Size(100, 22);
            this.txbKi.TabIndex = 0;
            // 
            // txbKp
            // 
            this.txbKp.Location = new System.Drawing.Point(92, 39);
            this.txbKp.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.txbKp.Name = "txbKp";
            this.txbKp.Size = new System.Drawing.Size(100, 22);
            this.txbKp.TabIndex = 0;
            // 
            // zedGraphPID
            // 
            this.zedGraphPID.Location = new System.Drawing.Point(13, 473);
            this.zedGraphPID.Margin = new System.Windows.Forms.Padding(5);
            this.zedGraphPID.Name = "zedGraphPID";
            this.zedGraphPID.ScrollGrace = 0D;
            this.zedGraphPID.ScrollMaxX = 0D;
            this.zedGraphPID.ScrollMaxY = 0D;
            this.zedGraphPID.ScrollMaxY2 = 0D;
            this.zedGraphPID.ScrollMinX = 0D;
            this.zedGraphPID.ScrollMinY = 0D;
            this.zedGraphPID.ScrollMinY2 = 0D;
            this.zedGraphPID.Size = new System.Drawing.Size(755, 406);
            this.zedGraphPID.TabIndex = 5;
            this.zedGraphPID.UseExtendedPrintDialog = true;
            // 
            // tabPage1
            // 
            this.tabPage1.Controls.Add(this.groupBox5);
            this.tabPage1.Controls.Add(this.groupBox4);
            this.tabPage1.Location = new System.Drawing.Point(4, 25);
            this.tabPage1.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.tabPage1.Name = "tabPage1";
            this.tabPage1.Padding = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.tabPage1.Size = new System.Drawing.Size(947, 869);
            this.tabPage1.TabIndex = 0;
            this.tabPage1.Text = "GRAPH VEL/ACC";
            this.tabPage1.UseVisualStyleBackColor = true;
            this.tabPage1.Click += new System.EventHandler(this.tabPage1_Click);
            // 
            // groupBox5
            // 
            this.groupBox5.Controls.Add(this.zedGraphAcc);
            this.groupBox5.Location = new System.Drawing.Point(21, 389);
            this.groupBox5.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox5.Name = "groupBox5";
            this.groupBox5.Padding = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox5.Size = new System.Drawing.Size(902, 464);
            this.groupBox5.TabIndex = 2;
            this.groupBox5.TabStop = false;
            this.groupBox5.Text = "GRAPH GIA TOC";
            // 
            // zedGraphAcc
            // 
            this.zedGraphAcc.Location = new System.Drawing.Point(7, 34);
            this.zedGraphAcc.Margin = new System.Windows.Forms.Padding(5);
            this.zedGraphAcc.Name = "zedGraphAcc";
            this.zedGraphAcc.ScrollGrace = 0D;
            this.zedGraphAcc.ScrollMaxX = 0D;
            this.zedGraphAcc.ScrollMaxY = 0D;
            this.zedGraphAcc.ScrollMaxY2 = 0D;
            this.zedGraphAcc.ScrollMinX = 0D;
            this.zedGraphAcc.ScrollMinY = 0D;
            this.zedGraphAcc.ScrollMinY2 = 0D;
            this.zedGraphAcc.Size = new System.Drawing.Size(873, 423);
            this.zedGraphAcc.TabIndex = 0;
            this.zedGraphAcc.UseExtendedPrintDialog = true;
            // 
            // groupBox4
            // 
            this.groupBox4.Controls.Add(this.zedGraphVelocity);
            this.groupBox4.Location = new System.Drawing.Point(5, 9);
            this.groupBox4.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox4.Name = "groupBox4";
            this.groupBox4.Padding = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox4.Size = new System.Drawing.Size(918, 376);
            this.groupBox4.TabIndex = 1;
            this.groupBox4.TabStop = false;
            this.groupBox4.Text = "GRAPH VAN TOC";
            // 
            // zedGraphVelocity
            // 
            this.zedGraphVelocity.Location = new System.Drawing.Point(19, 36);
            this.zedGraphVelocity.Margin = new System.Windows.Forms.Padding(5);
            this.zedGraphVelocity.Name = "zedGraphVelocity";
            this.zedGraphVelocity.ScrollGrace = 0D;
            this.zedGraphVelocity.ScrollMaxX = 0D;
            this.zedGraphVelocity.ScrollMaxY = 0D;
            this.zedGraphVelocity.ScrollMaxY2 = 0D;
            this.zedGraphVelocity.ScrollMinX = 0D;
            this.zedGraphVelocity.ScrollMinY = 0D;
            this.zedGraphVelocity.ScrollMinY2 = 0D;
            this.zedGraphVelocity.Size = new System.Drawing.Size(877, 337);
            this.zedGraphVelocity.TabIndex = 0;
            this.zedGraphVelocity.UseExtendedPrintDialog = true;
            // 
            // tabPage2
            // 
            this.tabPage2.Controls.Add(this.groupBox8);
            this.tabPage2.Controls.Add(this.groupBox7);
            this.tabPage2.Controls.Add(this.groupBox6);
            this.tabPage2.Location = new System.Drawing.Point(4, 25);
            this.tabPage2.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.tabPage2.Name = "tabPage2";
            this.tabPage2.Padding = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.tabPage2.Size = new System.Drawing.Size(947, 869);
            this.tabPage2.TabIndex = 1;
            this.tabPage2.Text = "OPERATION";
            this.tabPage2.UseVisualStyleBackColor = true;
            // 
            // groupBox8
            // 
            this.groupBox8.Controls.Add(this.btnConnectUSB);
            this.groupBox8.Controls.Add(this.label15);
            this.groupBox8.Controls.Add(this.txbVender);
            this.groupBox8.Controls.Add(this.label14);
            this.groupBox8.Controls.Add(this.txbProductID);
            this.groupBox8.Controls.Add(this.label13);
            this.groupBox8.Controls.Add(this.txbVenderID);
            this.groupBox8.Location = new System.Drawing.Point(21, 19);
            this.groupBox8.Name = "groupBox8";
            this.groupBox8.Size = new System.Drawing.Size(494, 267);
            this.groupBox8.TabIndex = 10;
            this.groupBox8.TabStop = false;
            this.groupBox8.Text = "USB CONNECT";
            // 
            // btnConnectUSB
            // 
            this.btnConnectUSB.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnConnectUSB.ForeColor = System.Drawing.SystemColors.MenuHighlight;
            this.btnConnectUSB.Location = new System.Drawing.Point(21, 89);
            this.btnConnectUSB.Name = "btnConnectUSB";
            this.btnConnectUSB.Size = new System.Drawing.Size(102, 70);
            this.btnConnectUSB.TabIndex = 2;
            this.btnConnectUSB.Text = "CONNECT";
            this.btnConnectUSB.UseVisualStyleBackColor = true;
            this.btnConnectUSB.Click += new System.EventHandler(this.btnConnectUSB_Click);
            // 
            // label15
            // 
            this.label15.AutoSize = true;
            this.label15.Font = new System.Drawing.Font("Microsoft Sans Serif", 10.2F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label15.ForeColor = System.Drawing.Color.Maroon;
            this.label15.Location = new System.Drawing.Point(141, 179);
            this.label15.Name = "label15";
            this.label15.Size = new System.Drawing.Size(68, 20);
            this.label15.TabIndex = 1;
            this.label15.Text = "Vendor";
            // 
            // txbVender
            // 
            this.txbVender.Location = new System.Drawing.Point(236, 164);
            this.txbVender.Multiline = true;
            this.txbVender.Name = "txbVender";
            this.txbVender.Size = new System.Drawing.Size(240, 35);
            this.txbVender.TabIndex = 0;
            // 
            // label14
            // 
            this.label14.AutoSize = true;
            this.label14.Font = new System.Drawing.Font("Microsoft Sans Serif", 10.2F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label14.ForeColor = System.Drawing.Color.Maroon;
            this.label14.Location = new System.Drawing.Point(141, 111);
            this.label14.Name = "label14";
            this.label14.Size = new System.Drawing.Size(93, 20);
            this.label14.TabIndex = 1;
            this.label14.Text = "ProductID";
            // 
            // txbProductID
            // 
            this.txbProductID.Location = new System.Drawing.Point(236, 102);
            this.txbProductID.Multiline = true;
            this.txbProductID.Name = "txbProductID";
            this.txbProductID.Size = new System.Drawing.Size(240, 35);
            this.txbProductID.TabIndex = 0;
            // 
            // label13
            // 
            this.label13.AutoSize = true;
            this.label13.Font = new System.Drawing.Font("Microsoft Sans Serif", 10.2F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label13.ForeColor = System.Drawing.Color.Maroon;
            this.label13.Location = new System.Drawing.Point(141, 41);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(87, 20);
            this.label13.TabIndex = 1;
            this.label13.Text = "VendorID";
            // 
            // txbVenderID
            // 
            this.txbVenderID.Location = new System.Drawing.Point(236, 34);
            this.txbVenderID.Multiline = true;
            this.txbVenderID.Name = "txbVenderID";
            this.txbVenderID.Size = new System.Drawing.Size(240, 31);
            this.txbVenderID.TabIndex = 0;
            // 
            // groupBox7
            // 
            this.groupBox7.Controls.Add(this.txbRms);
            this.groupBox7.Controls.Add(this.label10);
            this.groupBox7.Controls.Add(this.button4);
            this.groupBox7.Controls.Add(this.btnRun);
            this.groupBox7.Controls.Add(this.btnSendSeting);
            this.groupBox7.Controls.Add(this.btnDraw);
            this.groupBox7.Controls.Add(this.label9);
            this.groupBox7.Controls.Add(this.label7);
            this.groupBox7.Controls.Add(this.label5);
            this.groupBox7.Controls.Add(this.label8);
            this.groupBox7.Controls.Add(this.label6);
            this.groupBox7.Controls.Add(this.label4);
            this.groupBox7.Controls.Add(this.txbAccMax);
            this.groupBox7.Controls.Add(this.txbVelMax);
            this.groupBox7.Controls.Add(this.txbPosMax);
            this.groupBox7.Location = new System.Drawing.Point(521, 19);
            this.groupBox7.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox7.Name = "groupBox7";
            this.groupBox7.Padding = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox7.Size = new System.Drawing.Size(420, 267);
            this.groupBox7.TabIndex = 9;
            this.groupBox7.TabStop = false;
            this.groupBox7.Text = "Operation";
            // 
            // txbRms
            // 
            this.txbRms.Location = new System.Drawing.Point(69, 196);
            this.txbRms.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.txbRms.Multiline = true;
            this.txbRms.Name = "txbRms";
            this.txbRms.Size = new System.Drawing.Size(100, 41);
            this.txbRms.TabIndex = 4;
            // 
            // label10
            // 
            this.label10.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label10.ForeColor = System.Drawing.Color.Maroon;
            this.label10.Location = new System.Drawing.Point(11, 207);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(53, 38);
            this.label10.TabIndex = 3;
            this.label10.Text = "RMS";
            // 
            // button4
            // 
            this.button4.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.button4.ForeColor = System.Drawing.SystemColors.HotTrack;
            this.button4.Location = new System.Drawing.Point(276, 199);
            this.button4.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.button4.Name = "button4";
            this.button4.Size = new System.Drawing.Size(127, 42);
            this.button4.TabIndex = 2;
            this.button4.Text = "GET";
            this.button4.UseVisualStyleBackColor = true;
            this.button4.Click += new System.EventHandler(this.button4_Click);
            // 
            // btnRun
            // 
            this.btnRun.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnRun.ForeColor = System.Drawing.SystemColors.HotTrack;
            this.btnRun.Location = new System.Drawing.Point(276, 151);
            this.btnRun.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.btnRun.Name = "btnRun";
            this.btnRun.Size = new System.Drawing.Size(127, 42);
            this.btnRun.TabIndex = 2;
            this.btnRun.Text = "RUN";
            this.btnRun.UseVisualStyleBackColor = true;
            this.btnRun.Click += new System.EventHandler(this.btnRun_Click);
            // 
            // btnSendSeting
            // 
            this.btnSendSeting.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnSendSeting.ForeColor = System.Drawing.SystemColors.HotTrack;
            this.btnSendSeting.Location = new System.Drawing.Point(276, 103);
            this.btnSendSeting.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.btnSendSeting.Name = "btnSendSeting";
            this.btnSendSeting.Size = new System.Drawing.Size(127, 42);
            this.btnSendSeting.TabIndex = 2;
            this.btnSendSeting.Text = "SEND SETING";
            this.btnSendSeting.UseVisualStyleBackColor = true;
            this.btnSendSeting.Click += new System.EventHandler(this.btnSendSeting_Click);
            // 
            // btnDraw
            // 
            this.btnDraw.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btnDraw.ForeColor = System.Drawing.SystemColors.HotTrack;
            this.btnDraw.Location = new System.Drawing.Point(276, 54);
            this.btnDraw.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.btnDraw.Name = "btnDraw";
            this.btnDraw.Size = new System.Drawing.Size(127, 42);
            this.btnDraw.TabIndex = 2;
            this.btnDraw.Text = "MOTION";
            this.btnDraw.UseVisualStyleBackColor = true;
            this.btnDraw.Click += new System.EventHandler(this.btnDraw_Click);
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label9.ForeColor = System.Drawing.Color.Maroon;
            this.label9.Location = new System.Drawing.Point(176, 149);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(86, 17);
            this.label9.TabIndex = 1;
            this.label9.Text = "Unit/Sec^2";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label7.ForeColor = System.Drawing.Color.Maroon;
            this.label7.Location = new System.Drawing.Point(176, 103);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(69, 17);
            this.label7.TabIndex = 1;
            this.label7.Text = "Unit/Sec";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label5.ForeColor = System.Drawing.Color.Maroon;
            this.label5.Location = new System.Drawing.Point(176, 57);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(37, 17);
            this.label5.TabIndex = 1;
            this.label5.Text = "Unit";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label8.ForeColor = System.Drawing.Color.Maroon;
            this.label8.Location = new System.Drawing.Point(5, 149);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(62, 17);
            this.label8.TabIndex = 1;
            this.label8.Text = "AccMax";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label6.ForeColor = System.Drawing.Color.Maroon;
            this.label6.Location = new System.Drawing.Point(5, 103);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(59, 17);
            this.label6.TabIndex = 1;
            this.label6.Text = "VelMax";
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Font = new System.Drawing.Font("Microsoft Sans Serif", 7.8F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label4.ForeColor = System.Drawing.Color.Maroon;
            this.label4.Location = new System.Drawing.Point(5, 57);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(60, 17);
            this.label4.TabIndex = 1;
            this.label4.Text = "PosRef";
            // 
            // txbAccMax
            // 
            this.txbAccMax.Location = new System.Drawing.Point(69, 148);
            this.txbAccMax.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.txbAccMax.Name = "txbAccMax";
            this.txbAccMax.Size = new System.Drawing.Size(100, 22);
            this.txbAccMax.TabIndex = 0;
            // 
            // txbVelMax
            // 
            this.txbVelMax.Location = new System.Drawing.Point(69, 102);
            this.txbVelMax.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.txbVelMax.Name = "txbVelMax";
            this.txbVelMax.Size = new System.Drawing.Size(100, 22);
            this.txbVelMax.TabIndex = 0;
            // 
            // txbPosMax
            // 
            this.txbPosMax.Location = new System.Drawing.Point(69, 55);
            this.txbPosMax.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.txbPosMax.Name = "txbPosMax";
            this.txbPosMax.Size = new System.Drawing.Size(100, 22);
            this.txbPosMax.TabIndex = 0;
            // 
            // groupBox6
            // 
            this.groupBox6.Controls.Add(this.zedGraphPosRef);
            this.groupBox6.Location = new System.Drawing.Point(21, 310);
            this.groupBox6.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox6.Name = "groupBox6";
            this.groupBox6.Padding = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.groupBox6.Size = new System.Drawing.Size(907, 542);
            this.groupBox6.TabIndex = 1;
            this.groupBox6.TabStop = false;
            this.groupBox6.Text = "GRAPH POSITION";
            // 
            // zedGraphPosRef
            // 
            this.zedGraphPosRef.Location = new System.Drawing.Point(52, 30);
            this.zedGraphPosRef.Margin = new System.Windows.Forms.Padding(5);
            this.zedGraphPosRef.Name = "zedGraphPosRef";
            this.zedGraphPosRef.ScrollGrace = 0D;
            this.zedGraphPosRef.ScrollMaxX = 0D;
            this.zedGraphPosRef.ScrollMaxY = 0D;
            this.zedGraphPosRef.ScrollMaxY2 = 0D;
            this.zedGraphPosRef.ScrollMinX = 0D;
            this.zedGraphPosRef.ScrollMinY = 0D;
            this.zedGraphPosRef.ScrollMinY2 = 0D;
            this.zedGraphPosRef.Size = new System.Drawing.Size(805, 490);
            this.zedGraphPosRef.TabIndex = 0;
            this.zedGraphPosRef.UseExtendedPrintDialog = true;
            // 
            // tabControl1
            // 
            this.tabControl1.Controls.Add(this.tabPage2);
            this.tabControl1.Controls.Add(this.tabPage1);
            this.tabControl1.Dock = System.Windows.Forms.DockStyle.Right;
            this.tabControl1.Location = new System.Drawing.Point(796, 0);
            this.tabControl1.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.tabControl1.Name = "tabControl1";
            this.tabControl1.SelectedIndex = 0;
            this.tabControl1.Size = new System.Drawing.Size(955, 898);
            this.tabControl1.TabIndex = 7;
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 16F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1751, 898);
            this.Controls.Add(this.tabControl1);
            this.Controls.Add(this.zedGraphPID);
            this.Controls.Add(this.groupBox3);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.groupBox2);
            this.Margin = new System.Windows.Forms.Padding(3, 2, 3, 2);
            this.Name = "Form1";
            this.Text = "Form1";
            this.Load += new System.EventHandler(this.Form1_Load);
            this.KeyDown += new System.Windows.Forms.KeyEventHandler(this.Form1_KeyDown);
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.groupBox2.ResumeLayout(false);
            this.groupBox3.ResumeLayout(false);
            this.groupBox3.PerformLayout();
            this.tabPage1.ResumeLayout(false);
            this.groupBox5.ResumeLayout(false);
            this.groupBox4.ResumeLayout(false);
            this.tabPage2.ResumeLayout(false);
            this.groupBox8.ResumeLayout(false);
            this.groupBox8.PerformLayout();
            this.groupBox7.ResumeLayout(false);
            this.groupBox7.PerformLayout();
            this.groupBox6.ResumeLayout(false);
            this.tabControl1.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.TextBox txtReadSerial;
        private System.Windows.Forms.GroupBox groupBox2;
        private System.Windows.Forms.Button btnOpen;
        private System.Windows.Forms.ComboBox ccbBaudrate;
        private System.Windows.Forms.ComboBox ccbComName;
        private System.IO.Ports.SerialPort serialPort1;
        private System.Windows.Forms.GroupBox groupBox3;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.TextBox txbKd;
        private System.Windows.Forms.TextBox txbKi;
        private System.Windows.Forms.TextBox txbKp;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Button btnGetdata;
        private System.Windows.Forms.Button btnTuning;
        private System.Windows.Forms.Button btnSend;
        private ZedGraph.ZedGraphControl zedGraphPID;
        private System.Windows.Forms.TextBox txbTest;
        private System.Windows.Forms.Button btnReset;
        private System.Windows.Forms.Button btnSelect;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.TextBox txbSetpoint;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.TextBox txbRecieError;
        private System.Windows.Forms.TabPage tabPage1;
        private System.Windows.Forms.GroupBox groupBox5;
        private ZedGraph.ZedGraphControl zedGraphAcc;
        private System.Windows.Forms.GroupBox groupBox4;
        private ZedGraph.ZedGraphControl zedGraphVelocity;
        private System.Windows.Forms.TabPage tabPage2;
        private System.Windows.Forms.GroupBox groupBox7;
        private System.Windows.Forms.TextBox txbRms;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.Button button4;
        private System.Windows.Forms.Button btnRun;
        private System.Windows.Forms.Button btnSendSeting;
        private System.Windows.Forms.Button btnDraw;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.TextBox txbAccMax;
        private System.Windows.Forms.TextBox txbVelMax;
        private System.Windows.Forms.TextBox txbPosMax;
        private System.Windows.Forms.GroupBox groupBox6;
        private ZedGraph.ZedGraphControl zedGraphPosRef;
        private System.Windows.Forms.TabControl tabControl1;
        private System.Windows.Forms.GroupBox groupBox8;
        private System.Windows.Forms.Button btnConnectUSB;
        private System.Windows.Forms.Label label15;
        private System.Windows.Forms.TextBox txbVender;
        private System.Windows.Forms.Label label14;
        private System.Windows.Forms.TextBox txbProductID;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.TextBox txbVenderID;
    }
}

