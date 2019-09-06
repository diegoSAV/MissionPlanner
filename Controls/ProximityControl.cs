using log4net;
using System;
using System.Drawing;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Diagnostics;
using System.Windows.Forms;
using MissionPlanner.ArduPilot;
using static MAVLink;
using MissionPlanner.Properties;
using System.ComponentModel;
using MissionPlanner.Utilities;

namespace MissionPlanner.Controls
{
    public class ProximityControl : Form
    {
        private static readonly ILog log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

        MAVState _parent;
        private Proximity.directionState _dS => _parent.Proximity.DirectionState;
       
        KeyValuePair<MAVLINK_MSG_ID, Func<MAVLinkMessage, bool>> sub_attitude;

        int yaw;
        int desired_yaw = 0;

        private Timer timer1;
        private TextBox textBox1;
        private Button button1;
        private Button button2;
        private Button button3;
        private IContainer components;


        public bool DataAvailable { get; set; } = false;

        public ProximityControl(MAVState state)
        {
            InitializeComponent();

            _parent = state;

            Paint += Temp_Paint;
            KeyPress += Temp_KeyPress;
            Resize += Temp_Resize;
            FormClosing += ProximityControl_FormClosing; ;

            sub_attitude = state.parent.SubscribeToPacketType(MAVLINK_MSG_ID.ATTITUDE, messageReceived_attitude);

            timer1.Interval = 100;
            timer1.Tick += (s, e) => { Invalidate(); };

            timer1.Start();
        }

        private void ProximityControl_FormClosing(object sender, FormClosingEventArgs e)
        {
            timer1.Stop();
        }

        private void Temp_Resize(object sender, EventArgs e)
        {
            Invalidate();
        }

        private bool messageReceived_attitude(MAVLinkMessage arg)
        {
            //accept any compid, but filter sysid
            if (arg.sysid != _parent.sysid)
                return true;

            if (arg.msgid == (uint)MAVLINK_MSG_ID.ATTITUDE)
            {
                var message = arg.ToStructure<mavlink_attitude_t>();
                yaw = (int)(message.yaw * 180 / (float)3.1416);    
            }

            return true;
        }
        private void Temp_KeyPress(object sender, KeyPressEventArgs e)
        {
            switch (e.KeyChar)
            {
                case '+':
                    desired_yaw += 1;
                    e.Handled = true;
                    break;
                case '-':
                    desired_yaw -= 1;
                    e.Handled = true;
                    break;
                case ']':
                    desired_yaw++;
                    e.Handled = true;
                    break;
                case (char)Keys.Enter:
                    string yaw_text = textBox1.Text;
                    desired_yaw = Convert.ToInt32(yaw_text);
                    e.Handled = true;
                    break;
            }

            // prevent 0's
            if (screenradius < 1)
                screenradius = 50;
            if (mavsize < 1)
                mavsize = 1;

            Invalidate();
        }

        //cm's
        public float screenradius = 500;
        public float mavsize = 80;

        private void Temp_Paint(object sender, PaintEventArgs e)
        {
            var rawdata = _dS.GetRaw();

            e.Graphics.Clear(BackColor);

            var midx = e.ClipRectangle.Width / 2.0f;
            var midy = e.ClipRectangle.Height / 2.0f;

            Text ="Crash at  " + (desired_yaw);

            // 11m radius = 22 m coverage
            var scale = ((screenradius+50) * 2) / Math.Min(Height,Width);
            // 80cm quad / scale
            var size = mavsize / scale;

            switch(_parent.cs.firmware)
            {
                case Firmwares.ArduCopter2:
                    var imw = size/2;
                   // e.Graphics.DrawImage(Resources.quadicon, midx - imw, midy - imw, size, size);
                    break;
            }

            foreach (var temp in rawdata.ToList())
            {
                Vector3 location = new Vector3(0, Math.Min(temp.Distance / scale, (screenradius) / scale), 0);

                var halflength = location.length() / 2.0f;
                var doublelength = location.length() * 2.0f;
                var length = location.length();
                Pen redpen = new Pen(Color.Red, 3);
                float move = 5;
                var font = new Font(SystemFonts.DefaultFont.FontFamily, SystemFonts.DefaultFont.Size+140, FontStyle.Bold);

                switch (temp.Orientation)
                {
                    case MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_NONE:
                        location.rotate(Rotation.ROTATION_NONE);
                        e.Graphics.DrawString((temp.Distance / 100).ToString("0.00"), font, System.Drawing.Brushes.Red, 0, 0);
                        draw_compas(e);
                        break;
                    case MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_45:
                        location.rotate(Rotation.ROTATION_YAW_45);
                        e.Graphics.DrawString((temp.Distance/100).ToString("0.0m"), font, System.Drawing.Brushes.Green, midx - (float)location.x- move*8, midy - (float)location.y+ move);
                        e.Graphics.DrawArc(redpen, (float)(midx - length), (float)(midy - length), (float)doublelength, (float)doublelength, 22.5f - 90f, 45f);
                        break;
                    case MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_90:
                        location.rotate(Rotation.ROTATION_YAW_90);
                        e.Graphics.DrawString((temp.Distance/100).ToString("0.0m"), font, System.Drawing.Brushes.Green, midx - (float)location.x- move*8, midy - (float)location.y);
                        e.Graphics.DrawArc(redpen, (float)(midx - length), (float)(midy - length), (float)doublelength, (float)doublelength, 67.5f - 90f, 45f);
                        break;
                    case MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_135:
                        location.rotate(Rotation.ROTATION_YAW_135);
                        e.Graphics.DrawString((temp.Distance/100).ToString("0.0m"), font, System.Drawing.Brushes.Green, midx - (float)location.x- move*8, midy - (float)location.y- move);
                        e.Graphics.DrawArc(redpen, (float)(midx - length), (float)(midy - length), (float)doublelength, (float)doublelength, 112.5f - 90f, 45f);
                        break;
                    case MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_180:
                        location.rotate(Rotation.ROTATION_YAW_180);
                        e.Graphics.DrawString((temp.Distance/100).ToString("0.0m"), font, System.Drawing.Brushes.Green, midx - (float)location.x-move*2, midy - (float)location.y-move*3);
                        e.Graphics.DrawArc(redpen, (float)(midx - length), (float)(midy - length), (float)doublelength, (float)doublelength, 157.5f - 90f, 45f);
                        break;
                    case MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_225:
                        location.rotate(Rotation.ROTATION_YAW_225);
                        e.Graphics.DrawString((temp.Distance/100).ToString("0.0m"), font, System.Drawing.Brushes.Green, midx - (float)location.x+ move, midy - (float)location.y- move);
                        e.Graphics.DrawArc(redpen, (float)(midx - length), (float)(midy - length), (float)doublelength, (float)doublelength, 202.5f - 90f, 45f);
                        break;
                    case MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_270:
                        location.rotate(Rotation.ROTATION_YAW_270);
                        e.Graphics.DrawString((temp.Distance/100).ToString("0.0m"), font, System.Drawing.Brushes.Green, midx - (float)location.x+move, midy - (float)location.y);
                        e.Graphics.DrawArc(redpen, (float)(midx - length), (float)(midy - length), (float)doublelength, (float)doublelength, 247.5f - 90f, 45f);
                        break;
                    case MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_YAW_315:
                        location.rotate(Rotation.ROTATION_YAW_315);
                        e.Graphics.DrawString((temp.Distance/100).ToString("0.0m"), font, System.Drawing.Brushes.Green, midx - (float)location.x+move, midy - (float)location.y);
                        e.Graphics.DrawArc(redpen, (float)(midx - length), (float)(midy - length), (float)doublelength, (float)doublelength, 292.5f - 90f, 45f);
                        break;
                }
            }
        }

        public new void Show()
        {
            if (!IsDisposed)
            {
                if (Visible)
                    return;

                base.Show();
            }
            else
            {
                Dispose();
                _parent.Proximity = new Proximity(_parent);
                base.Show();
            }
        }

        public new void Dispose()
        {
            timer1.Stop();
        }

        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.textBox1 = new System.Windows.Forms.TextBox();
            this.button1 = new System.Windows.Forms.Button();
            this.button2 = new System.Windows.Forms.Button();
            this.button3 = new System.Windows.Forms.Button();
            this.SuspendLayout();
            // 
            // textBox1
            // 
            this.textBox1.Location = new System.Drawing.Point(429, 12);
            this.textBox1.Name = "textBox1";
            this.textBox1.Size = new System.Drawing.Size(100, 20);
            this.textBox1.TabIndex = 0;
            this.textBox1.Text = "Introudire YAW";
            // 
            // button1
            // 
            this.button1.Location = new System.Drawing.Point(440, 38);
            this.button1.Name = "button1";
            this.button1.Size = new System.Drawing.Size(75, 23);
            this.button1.TabIndex = 1;
            this.button1.Text = "Intro Yaw";
            this.button1.UseVisualStyleBackColor = true;
            this.button1.Click += new System.EventHandler(this.button1_Click);
            // 
            // button2
            // 
            this.button2.Location = new System.Drawing.Point(440, 67);
            this.button2.Name = "button2";
            this.button2.Size = new System.Drawing.Size(75, 23);
            this.button2.TabIndex = 2;
            this.button2.Text = "+ 90";
            this.button2.UseVisualStyleBackColor = true;
            this.button2.Click += new System.EventHandler(this.button2_Click);
            // 
            // button3
            // 
            this.button3.Location = new System.Drawing.Point(440, 96);
            this.button3.Name = "button3";
            this.button3.Size = new System.Drawing.Size(75, 23);
            this.button3.TabIndex = 3;
            this.button3.Text = "- 90";
            this.button3.UseVisualStyleBackColor = true;
            this.button3.Click += new System.EventHandler(this.button3_Click);
            // 
            // ProximityControl
            // 
            this.ClientSize = new System.Drawing.Size(533, 481);
            this.Controls.Add(this.button3);
            this.Controls.Add(this.button2);
            this.Controls.Add(this.button1);
            this.Controls.Add(this.textBox1);
            this.Name = "ProximityControl";
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        private void button1_Click(object sender, EventArgs e)
        {
            string yaw_text = textBox1.Text;
            desired_yaw = Convert.ToInt32(yaw_text);
            if (desired_yaw > 360 || desired_yaw < 1)
                desired_yaw = 0;
        }

        private void button2_Click(object sender, EventArgs e)
        {
            desired_yaw += 90;
            if (desired_yaw > 360)
                desired_yaw -= 360;
            else if (desired_yaw < 1)
                desired_yaw += 360;
        }

        private void button3_Click(object sender, EventArgs e)
        {
            desired_yaw -= 90;
            if (desired_yaw > 360)
                desired_yaw -= 360;
            else if (desired_yaw < 1)
                desired_yaw += 360;
        }
        private void draw_compas(PaintEventArgs e)
        {
            var font = new Font(SystemFonts.DefaultFont.FontFamily, SystemFonts.DefaultFont.Size + 15, FontStyle.Bold);
            Pen pen = new Pen(Color.White, 3);
            Rectangle compas = new Rectangle(10, 250, 250, 250);
            Rectangle arrow = new Rectangle(130, 260, 10, 120);
            e.Graphics.DrawEllipse(pen, compas);
            e.Graphics.DrawString(desired_yaw.ToString("0"), font, System.Drawing.Brushes.Yellow, 120, 210);
            e.Graphics.DrawImage(Resources.arrow, compas);
        }
    }
}