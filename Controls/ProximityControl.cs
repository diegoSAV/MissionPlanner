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
        KeyValuePair<MAVLINK_MSG_ID, Func<MAVLinkMessage, bool>> sub_gps;
        KeyValuePair<MAVLINK_MSG_ID, Func<MAVLinkMessage, bool>> sub_camera;

        int yaw;
        int desired_yaw = 0;
        float vz;
        ushort index_photo;
        float relative_alt;

        private Timer timer1;
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

            sub_attitude = state.parent.SubscribeToPacketType(MAVLINK_MSG_ID.ATTITUDE, MessageReceived_attitude);
            sub_gps = state.parent.SubscribeToPacketType(MAVLINK_MSG_ID.GLOBAL_POSITION_INT, MessageReceived_gps);
            sub_camera = state.parent.SubscribeToPacketType(MAVLINK_MSG_ID.CAMERA_FEEDBACK, MessageReceived_camera);

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

        private bool MessageReceived_attitude(MAVLinkMessage arg)
        {
            //accept any compid, but filter sysid
            if (arg.sysid != _parent.sysid)
                return true;

            if (arg.msgid == (uint)MAVLINK_MSG_ID.ATTITUDE)
            {
                var message = arg.ToStructure<mavlink_attitude_t>();
                yaw = (int)(message.yaw * 180 / Math.PI);
                if (yaw < 1)
                {
                    yaw = 360 + yaw;
                }
            }
            return true;
        }

        private bool MessageReceived_gps(MAVLinkMessage arg)
        {
            //accept any compid, but filter sysid
            if (arg.sysid != _parent.sysid)
                return true;

            if (arg.msgid == (uint)MAVLINK_MSG_ID.GLOBAL_POSITION_INT)
            {
                var message = arg.ToStructure<mavlink_global_position_int_t>();
                vz = Math.Abs((float)message.vz / (float)100.0);
            }
            return true;
        }

        private bool MessageReceived_camera(MAVLinkMessage arg)
        {
            //accept any compid, but filter sysid
            if (arg.sysid != _parent.sysid)
                return true;

            if (arg.msgid == (uint)MAVLINK_MSG_ID.CAMERA_FEEDBACK)
            {
              /*
              this.time_usec = time_usec;
              this.lat = lat;
              this.lng = lng;
              this.alt_msl = alt_msl;
              this.alt_rel = alt_rel;
              this.roll = roll;
              this.pitch = pitch;
              this.yaw = yaw;
              this.foc_len = foc_len;
              this.img_idx = img_idx;
              this.target_system = target_system;
              this.cam_idx = cam_idx;
              this.flags = flags;
              this.completed_captures = completed_captures;
              */
                var message = arg.ToStructure<mavlink_camera_feedback_t>();
                index_photo = message.img_idx;
                relative_alt = message.alt_rel;
            }
            return true;
        }

        private void Temp_KeyPress(object sender, KeyPressEventArgs e)
        {
            switch (e.KeyChar)
            {
                case (char)Keys.Enter:
                    desired_yaw = yaw;
                    e.Handled = true;
                    break;
                case 'a':
                    desired_yaw += 90;
                    if (desired_yaw > 360)
                        desired_yaw -= 360;
                    else if (desired_yaw < 1)
                        desired_yaw += 360;
                    e.Handled = true;
                    break;
                case 'q':
                    desired_yaw -= 90;
                    if (desired_yaw > 360)
                        desired_yaw -= 360;
                    else if (desired_yaw < 1)
                        desired_yaw += 360;
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

            Text ="SupAirVision Custom Window  ";
            Draw_compas(e);
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
                        e.Graphics.DrawString((temp.Distance / 100).ToString("0.00"), font, System.Drawing.Brushes.White, 0, 0);
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
            this.SuspendLayout();
            // 
            // ProximityControl
            // 
            this.ClientSize = new System.Drawing.Size(584, 455);
            this.Name = "ProximityControl";
            this.ResumeLayout(false);

        }

        private void Draw_compas(PaintEventArgs e)
        {
            var font = new Font(SystemFonts.DefaultFont.FontFamily, SystemFonts.DefaultFont.Size + 30, FontStyle.Bold);
            var font2 = new Font(SystemFonts.DefaultFont.FontFamily, SystemFonts.DefaultFont.Size + 70, FontStyle.Bold);
            var font3 = new Font(SystemFonts.DefaultFont.FontFamily, SystemFonts.DefaultFont.Size + 30, FontStyle.Bold);
            var brush = new SolidBrush(Color.White);
            var brush2 = new SolidBrush(Color.Green);
            int pos_y = 310;
            int pos_x;
            Pen pen = new Pen(Color.White, 3);
            Pen pen2 = new Pen(Color.Green, 7);
            Pen pen3 = new Pen(Color.Yellow, 5);
            Rectangle compas = new Rectangle(10, 250, 250, 250);

            e.Graphics.DrawEllipse(pen, compas);

            int difference_yaw = desired_yaw - yaw;
            if (difference_yaw > 180)
                difference_yaw = (360 - difference_yaw) * (-1);
            else if (difference_yaw < -179)
                difference_yaw = (360 + difference_yaw) * (-1);
            difference_yaw = difference_yaw * (-1);
            if (Math.Abs(difference_yaw) > 99)
                pos_x = 25;
            else if (Math.Abs(difference_yaw) > 9)
                pos_x = 60;
            else pos_x = 95;
            if (difference_yaw < 0)
                pos_x -= 25;

            e.Graphics.DrawString(desired_yaw.ToString("0"), font, System.Drawing.Brushes.White, 75, 410);

            if(Math.Abs(difference_yaw) > 15)
                e.Graphics.DrawString(difference_yaw.ToString("0"), font2, System.Drawing.Brushes.Red, pos_x, pos_y);
            else if (Math.Abs(difference_yaw) > 5)
                e.Graphics.DrawString(difference_yaw.ToString("0"), font2, System.Drawing.Brushes.Yellow, pos_x, pos_y);
            else e.Graphics.DrawString(difference_yaw.ToString("0"), font2, System.Drawing.Brushes.Green, pos_x, pos_y);

            if (vz > 1)
            {
                e.Graphics.DrawString("Speed", font3, System.Drawing.Brushes.Red, 300, 240);
                e.Graphics.DrawString("Warning", font3, System.Drawing.Brushes.Red, 280, 300);
                e.Graphics.DrawString(vz.ToString("0.00"), font3, System.Drawing.Brushes.Red, 330, 360);
            }
            else if (vz >0.6)
            {
                e.Graphics.DrawString("Speed", font3, System.Drawing.Brushes.Yellow, 300, 240);
                e.Graphics.DrawString("Warning", font3, System.Drawing.Brushes.Yellow, 280, 300);
                e.Graphics.DrawString(vz.ToString("0.00"), font3, System.Drawing.Brushes.Yellow, 330, 360);
            }

            double circle_x = 135 + (110 * Math.Sin((double)difference_yaw * Math.PI / 180));
            double circle_y = 375 - (110 * Math.Cos((double)difference_yaw * Math.PI / 180));
            Point point_a = new Point((int)circle_x, (int)circle_y);
            double circle_x2 = 135 + (90 * Math.Sin((double)(difference_yaw - 4) * Math.PI / 180));
            double circle_y2 = 375 - (90 * Math.Cos((double)(difference_yaw - 4) * Math.PI / 180));
            Point point_b = new Point((int)circle_x2, (int)circle_y2);
            double circle_x3 = 135 + (90 * Math.Sin((double)(difference_yaw + 4) * Math.PI / 180));
            double circle_y3 = 375 - (90 * Math.Cos((double)(difference_yaw + 4) * Math.PI / 180));
            Point point_c = new Point((int)circle_x3, (int)circle_y3);
            Point[] triangle = { point_a, point_b, point_c };
            e.Graphics.FillPolygon(brush, triangle);
            e.Graphics.FillRectangle(brush, 133, 240, 3, 20);
            e.Graphics.DrawArc(pen3, compas, 255, 30);
            e.Graphics.DrawArc(pen2, compas, 265, 10);

            e.Graphics.DrawString(index_photo.ToString(), font, System.Drawing.Brushes.Red, 500, 10);
        }
    }
}