using log4net;
using System;
using System.Drawing;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Diagnostics;
using System.Windows.Forms;
using System.IO;
using MissionPlanner.ArduPilot;
using static MAVLink; 
using MissionPlanner.Properties;
using System.ComponentModel;
using MissionPlanner.Utilities;
using CsvHelper;
using NetCoreAudio;

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

        private Player Sounder;
        string Sound_path;

        int drone_yaw;
        int desired_yaw = 0;
        bool is_log_created = false;
        string log_name;
        float drone_rel_alt;
        float last_photo_alt;
        int photo_count = 0;
        int side_photo_count = 0;
        float vz;
        double proximity_sensor;

        List<ImagesLog> images_list = new List<ImagesLog>();

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
                drone_yaw = (int)(message.yaw * 180 / Math.PI);
                if (drone_yaw < 1)
                {
                    drone_yaw = 360 + drone_yaw;
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
                drone_rel_alt = (float)message.relative_alt / 1000;
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
            
                Sounder.Play(Sound_path);
                photo_count++;
                side_photo_count++;
                images_list.Add(new ImagesLog(arg, proximity_sensor));
                if (is_log_created == false)
                {

                    DateTime time = DateTime.Now;
                    log_name = time.ToString("yyyMMdd-HHmmss");
                    log_name = log_name + ".csv";
                    is_log_created = true;
                }
                using (var reader = new StreamWriter(log_name))
                using (var csv = new CsvWriter(reader))
                {
                    csv.WriteRecords(images_list);
                }
                last_photo_alt = drone_rel_alt;
            }
            return true;
        }

        private void Temp_KeyPress(object sender, KeyPressEventArgs e)
        {
            switch (e.KeyChar)
            {
                case (char)Keys.Enter:
                    desired_yaw = drone_yaw;
                    side_photo_count = 0;
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
                case 'z':
                    desired_yaw += 1;
                    if (desired_yaw > 360)
                        desired_yaw -= 360;
                    else if (desired_yaw < 1)
                        desired_yaw += 360;
                    e.Handled = true;
                    break;
                case 's':
                    desired_yaw -= 1;
                    if (desired_yaw > 360)
                        desired_yaw -= 360;
                    else if (desired_yaw < 1)
                        desired_yaw += 360;
                    e.Handled = true;
                    break;
                case 'p':
                    _parent.parent.doCommand(MAV_CMD.DO_DIGICAM_CONTROL, 0, 0, 0, 0, 1, 0, 0); 
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
                var font2 = new Font(SystemFonts.DefaultFont.FontFamily, SystemFonts.DefaultFont.Size + 80, FontStyle.Bold);

                switch (temp.Orientation)
                {
                    case MAV_SENSOR_ORIENTATION.MAV_SENSOR_ROTATION_NONE:
                        location.rotate(Rotation.ROTATION_NONE);
                        if(temp.Distance > 999)
                            e.Graphics.DrawString((temp.Distance / 100).ToString("0.0"), font2, System.Drawing.Brushes.White, 0, 0);
                        else
                            e.Graphics.DrawString((temp.Distance / 100).ToString("0.0"), font, System.Drawing.Brushes.White, 0, 0);
                        proximity_sensor = temp.Distance;
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
            this.Sounder = new NetCoreAudio.Player();
            this.SuspendLayout();
            // 
            // ProximityControl
            // 
            this.ClientSize = new System.Drawing.Size(900, 500);
            this.Name = "ProximityControl";
            this.ResumeLayout(false);

            Sound_path = Directory.GetCurrentDirectory();
            Sound_path = Directory.GetParent(Sound_path).FullName;
            Sound_path = Directory.GetParent(Sound_path).FullName;
            Sound_path = Directory.GetParent(Sound_path).FullName;
            Sound_path = Sound_path + @"\Resources\Camera.wav";

        }

        private void Draw_compas(PaintEventArgs e)
        {
            var font = new Font(SystemFonts.DefaultFont.FontFamily, SystemFonts.DefaultFont.Size + 30, FontStyle.Bold);
            var font2 = new Font(SystemFonts.DefaultFont.FontFamily, SystemFonts.DefaultFont.Size + 70, FontStyle.Bold);
            var font3 = new Font(SystemFonts.DefaultFont.FontFamily, SystemFonts.DefaultFont.Size + 30, FontStyle.Bold);
            var font4 = new Font(SystemFonts.DefaultFont.FontFamily, SystemFonts.DefaultFont.Size + 15, FontStyle.Bold);
            var brush = new SolidBrush(Color.White);
            var brush2 = new SolidBrush(Color.Green);
            int pos_y = 310;
            int pos_x;
            Pen pen = new Pen(Color.White, 3);
            Pen pen2 = new Pen(Color.Green, 7);
            Pen pen3 = new Pen(Color.Yellow, 5);
            Rectangle compas = new Rectangle(10, 250, 250, 250);

            e.Graphics.DrawEllipse(pen, compas);

            int difference_yaw = desired_yaw - drone_yaw;
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
                e.Graphics.DrawString("Speed", font3, System.Drawing.Brushes.Red, 330, 240);
                e.Graphics.DrawString("Warning", font3, System.Drawing.Brushes.Red, 310, 300);
                e.Graphics.DrawString(vz.ToString("0.00"), font3, System.Drawing.Brushes.Red, 360, 360);
            }
            else if (vz >0.6)
            {
                e.Graphics.DrawString("Speed", font3, System.Drawing.Brushes.Yellow, 330, 240);
                e.Graphics.DrawString("Warning", font3, System.Drawing.Brushes.Yellow, 310, 300);
                e.Graphics.DrawString(vz.ToString("0.00"), font3, System.Drawing.Brushes.Yellow, 360, 360);
            }

            double circle_x = 135 + (114 * Math.Sin((double)difference_yaw * Math.PI / 180));
            double circle_y = 375 - (114 * Math.Cos((double)difference_yaw * Math.PI / 180));
            Point point_a = new Point((int)circle_x, (int)circle_y);
            double circle_x2 = 135 + (85 * Math.Sin((double)(difference_yaw - 5) * Math.PI / 180));
            double circle_y2 = 375 - (85 * Math.Cos((double)(difference_yaw - 5) * Math.PI / 180));
            Point point_b = new Point((int)circle_x2, (int)circle_y2);
            double circle_x3 = 135 + (85 * Math.Sin((double)(difference_yaw + 5) * Math.PI / 180));
            double circle_y3 = 375 - (85 * Math.Cos((double)(difference_yaw + 5) * Math.PI / 180));
            Point point_c = new Point((int)circle_x3, (int)circle_y3);
            Point[] triangle = { point_a, point_b, point_c };
            e.Graphics.FillPolygon(brush, triangle);
            e.Graphics.FillRectangle(brush, 133, 240, 3, 20);
            e.Graphics.DrawArc(pen3, compas, 255, 30);
            e.Graphics.DrawArc(pen2, compas, 265, 10);

            e.Graphics.DrawString("Hauteur :", font4, System.Drawing.Brushes.White, 460, 40);
            e.Graphics.DrawString((drone_rel_alt - last_photo_alt).ToString("0.0m"), font, System.Drawing.Brushes.White, 620, 32);
            e.Graphics.DrawString("Photo Count :", font4, System.Drawing.Brushes.White, 460, 90);
            e.Graphics.DrawString(photo_count.ToString(), font, System.Drawing.Brushes.White, 670, 85);
            e.Graphics.DrawString("Side photo :", font4, System.Drawing.Brushes.White, 460, 140);
            e.Graphics.DrawString(side_photo_count.ToString(), font, System.Drawing.Brushes.White, 670, 135);

        }
    }

    public class ImagesLog 
    {
        public ulong time_usec { get; set; }
        public int lat { get; set; }
        public int lng { get; set; }
        public float alt_msl { get; set; }
        public float alt_rel { get; set; }
        public float roll { get; set; }
        public float pitch { get; set; }
        public float yaw { get; set; }
        public ushort img_idx { get; set; }
        public double prx_sensor { get; set; }


        public  ImagesLog(MAVLinkMessage arg, double proximity_sensor)
        {
            var message = arg.ToStructure<mavlink_camera_feedback_t>();
            time_usec = message.time_usec;
            lat = message.lat;
            lng = message.lng;
            alt_msl = message.alt_msl;
            alt_rel = message.alt_rel;
            roll = message.roll;
            pitch = message.pitch;
            yaw = message.yaw;
            img_idx = message.img_idx;
            prx_sensor = proximity_sensor;
        }
    }
}