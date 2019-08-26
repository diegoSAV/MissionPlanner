using log4net;
using System;
using System.Drawing;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Diagnostics;
using System.Windows.Forms;
using MissionPlanner.ArduPilot;
using NetCoreAudio;
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
        private Proximity.auxiliaryData _aX => _parent.Proximity.AuxiliaryData;
       
        KeyValuePair<MAVLINK_MSG_ID, Func<MAVLinkMessage, bool>> sub;
        int imu;

        private Player sounder; 
        private Timer timer1;
        private Stopwatch chronometer;
        private IContainer components;
        private int desired_lenght = 100;
        private int desired_distance = 150;
        private int Hit_Detected = 0;

        public bool DataAvailable { get; set; } = false;

        public ProximityControl(MAVState state)
        {
            InitializeComponent();

            _parent = state;

            Paint += Temp_Paint;
            KeyPress += Temp_KeyPress;
            Resize += Temp_Resize;
            FormClosing += ProximityControl_FormClosing; ;

            sub = state.parent.SubscribeToPacketType(MAVLINK_MSG_ID.RAW_IMU, messageReceived);

            timer1.Interval = 100;
            timer1.Tick += (s, e) => { Invalidate(); };

            timer1.Start();
        }

        private void ProximityControl_FormClosing(object sender, FormClosingEventArgs e)
        {
            timer1.Stop();
            chronometer.Stop();
        }

        private void Temp_Resize(object sender, EventArgs e)
        {
            Invalidate();
        }

        private bool messageReceived(MAVLinkMessage arg)
        {
            //accept any compid, but filter sysid
            if (arg.sysid != _parent.sysid)
                return true;

            if (arg.msgid == (uint)MAVLINK_MSG_ID.RAW_IMU)
            {
                 var message = arg.ToStructure<mavlink_raw_imu_t>();
                imu = message.xacc; 
                if (imu > 100)
                {
                    if (Hit_Detected == 0)
                    {
                        sounder.Play(@"C:\Users\diega\Music\test2.wav");
                        Hit_Detected = 1;
                    }
                    
                }
                else if (Hit_Detected == 1)
                    Hit_Detected = 0;

            }

            return true;
        }
        private void Temp_KeyPress(object sender, KeyPressEventArgs e)
        {
            switch (e.KeyChar)
            {
                case '+':
                    desired_distance += 1;
                    e.Handled = true;
                    break;
                case '-':
                    desired_distance -= 1;
                    e.Handled = true;
                    break;
                case ']':
                    desired_lenght++;
                    e.Handled = true;
                    break;
                case '[':
                    desired_lenght--;
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

            Text ="Crash at  " + (desired_distance) + " cm and sound during" + (desired_lenght) + "cm";

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
                long miliseconds = chronometer.ElapsedMilliseconds;
                Pen redpen = new Pen(Color.Red, 3);
                float move = 5;
                var font = new Font(SystemFonts.DefaultFont.FontFamily, SystemFonts.DefaultFont.Size+140, FontStyle.Bold);

                switch (temp.Orientation)
                {
                    case MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_NONE:
                        location.rotate(Rotation.ROTATION_NONE);
                        if ((desired_distance + desired_lenght) < temp.Distance)
                        {
                            e.Graphics.DrawString((temp.Distance / 100).ToString("0.00"), font, System.Drawing.Brushes.White, 0, 0);
                        }
                        else if (desired_distance < temp.Distance)
                        {
                            if(((temp.Distance - desired_distance)*2000/desired_lenght)<miliseconds)
                            {
                                sounder.Play(@"C:\Users\diega\Music\test.wav");
                                chronometer.Restart();
                            }
                            e.Graphics.DrawString((temp.Distance / 100).ToString("0.00"), font, System.Drawing.Brushes.Green, 0, 0);
                        }
                        else
                        {
                            e.Graphics.DrawString((temp.Distance / 100).ToString("0.00"), font, System.Drawing.Brushes.Red, 0, 0);
                            chronometer.Restart();
                        }
                       
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
            chronometer.Stop();
            timer1.Stop();
        }

        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.chronometer = new System.Diagnostics.Stopwatch();
            this.chronometer.Start();
            this.SuspendLayout();
            this.sounder = new NetCoreAudio.Player(); 
            // 
            // ProximityControl
            // 
            this.ClientSize = new System.Drawing.Size(471, 282);
            this.Name = "ProximityControl";
            this.ResumeLayout(false);

        }
    }
}