using IPC;
using Godot;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Sensor;
using Geometry = RosSharp.RosBridgeClient.MessageTypes.Geometry;

namespace UI
{
    public partial class CoreTabUI : BaseTabUI
    {
        private readonly CoreCtrlState controlMsg = new();
        private const string CoreControlTopic = "/core/control/state";

        private readonly Geometry.Twist twistMsg = new() { angular = new(0, 0, 0), linear = new(0, 0, 0) };
        private const string TwistTopic = "/core/control/cmd_vel";

        private readonly PtzControl ptzMsg = new();
        private const string PTZTopic = "/ptz/control";

        private const string GPSTopic = "/core/feedback/gps/fix";
        private const string IMUTopic = "/core/feedback/imu/data";

        public bool TankDriving = false;

        [ExportCategory("Core")]
        [ExportGroup("Motors")]
        [Export]
        public ProgressBar LMotor;
        [Export]
        public ProgressBar RMotor;
        [Export]
        public Button DrivingMode;
        [Export]
        public ProgressBar MaxMotorDrive;
        [Export]
        public Texture2D Tank, Wheel;

        private Geometry.Vector3 Lin = new(), Ang = new();

        [ExportGroup("Brake")]
        [Export]
        private ColoredIndicator BrakeIndicator;
        [Export]
        private TextureRect Brake;
        public bool BrakeState = true;

        public float MaxSpeed = 0.6f;
        private double MaxSpeedCollector = 0;

        [ExportGroup("PTZ")]
        [ExportSubgroup("Rendering")]
        [Export]
        public SubViewport PTZSubViewport;
        [Export]
        public MeshInstance3D PTZAxis0;
        [Export]
        public MeshInstance3D PTZAxis1;

        private Vector2 WindowSize;

        [ExportSubgroup("Controlling")]
        [Export]
        public GridContainer PTZButtonCont;
        [Export]
        public Label PTZRotX, PTZRotY;

        [Export]
        public VSlider Zoom;

        [ExportGroup("Feedback")]
        [Export]
        public Control Needle;
        private NavSatFix PrevFix = new();
        [Export]
        public Control HeadingIndicator;

        public override void _Ready()
        {
            base._Ready();

            // Setting up PTZ stuff
            int windowSize = (int)(GetWindow().Size.Y * 0.125f);
            if (windowSize < 16) windowSize = 16;
            PTZSubViewport.Size = new Vector2I(windowSize, windowSize);

            GetTree().Root.SizeChanged += Resize;

            PTZRotX.Text = "Y:" + ptzMsg.yaw.ToString().PadLeft(3, ' ');
            PTZRotY.Text = "P:" + ptzMsg.pitch.ToString().PadLeft(3, ' ');
            ptzMsg.control_mode = 1;

            // Arrays assigned in the editor dont't properly serialize across git. I hate this too.
            // Button[] PTZButtons = [.. PTZButtonCont.GetChildren().Where(static _ => _ is Button).Cast<Button>()];

            // I know this is ugly...
            // PTZButtons[0].Pressed += () => { ptzMsg.pitch += 5; PushToPTZ(); };
            // PTZButtons[1].Pressed += () => { ptzMsg.yaw -= 5; PushToPTZ(); };
            // PTZButtons[2].Pressed += () => { ptzMsg.pitch = ptzMsg.yaw = 0; PushToPTZ(); };
            // PTZButtons[3].Pressed += () => { ptzMsg.yaw += 5; PushToPTZ(); };
            // PTZButtons[4].Pressed += () => { ptzMsg.pitch -= 5; PushToPTZ(); };

            DrivingMode.Toggled += (pressed) => { TankDriving = pressed; (DrivingMode.GetChild(0) as TextureRect).Texture = pressed ? Tank : Wheel; };
        }

        public void Resize()
        {
            WindowSize = GetWindow().Size;
            if (Size.Y < WindowSize.Y * 0.125f)
                PTZSubViewport.Size = new Vector2I((int)(WindowSize.Y * 0.125f), (int)(WindowSize.Y * 0.125f));
        }

        public override void _Process(double delta)
        {
            base._Process(delta);

            if (TankDriving)
            {
                LMotor.Value = LeftStick.Y;
                RMotor.Value = RightStick.Y;
                Lin.x = (LeftStick.Y + RightStick.Y) * 0.5f;
                Ang.z = (RightStick.Y - LeftStick.Y) * 0.5f;
            }
            else
            {
                LMotor.Value = LeftStick.Y;
                RMotor.Value = RightStick.X;
                Lin.x = LeftStick.Y;
                Ang.z = -RightStick.X;
            }

            Lin.x *= 1.75;
            Ang.z *= 1.5;

            // If ANY of these buttons are pressed, rover brakes
            bool b = XButton + AButton + BButton > 0;
            if (b != BrakeState)
            {
                BrakeState = b;
                BrakeIndicator.Value = b;
                Brake.Visible = b;
            }

            if (UpButtonDown)
                ptzMsg.pitch += (float)delta;
            else if (DownButtonDown)
                ptzMsg.pitch -= (float)delta;
            if (RightButtonDown)
                ptzMsg.yaw += (float)delta;
            else if (LeftButtonDown)
                ptzMsg.yaw -= (float)delta;
            ptzMsg.zoom_level += (float)delta * (WhiteButton - BlackButton);

            // (In/De)crease boost mode amount. Collector
            // stores the frame delta so that it takes a while to actually
            // increase the max speed
            if (RightTrigger >= 1f && LeftTrigger >= 1f)
            {
                MaxSpeed = 0.5f;
                MaxMotorDrive.Value = 50;
            }
            else if (RightTrigger >= 1f)
            {
                MaxSpeedCollector += delta * 50;
                if (MaxSpeedCollector > 1)
                {
                    MaxSpeedCollector = 0;
                    if (MaxSpeed < 1)
                    {
                        MaxSpeed += (float)delta;
                        MaxMotorDrive.Value = MaxSpeed * 100f;
                    }
                }
            }
            else if (LeftTrigger >= 1f)
            {
                MaxSpeedCollector -= delta * 50;
                if (MaxSpeedCollector < -1)
                {
                    MaxSpeedCollector = 0;
                    if (MaxSpeed > 0)
                    {
                        MaxSpeed -= (float)delta;
                        MaxMotorDrive.Value = MaxSpeed * 100f;
                    }
                }
            }
        }

        // Wraps a value between a low and high numberset
        public static float ConstWrap(float value, float min, float max)
        {
            float range = max - min;
            return min + ((((value - min) % range) + range) % range);
        }

        // Kinda like EmitToROS but is called on demand instead of constantly
        public void PushToPTZ()
        {
            if (ptzMsg.pitch > 134f)
                ptzMsg.pitch = 134f;
            else if (ptzMsg.pitch < -134f)
                ptzMsg.pitch = -134f;

            ptzMsg.yaw = ConstWrap(ptzMsg.yaw, 0f, 360f);

            // Literally just caching the conversion
            float ZoomAmount = (float)Zoom.Value;

            PTZAxis0.RotationDegrees = Vector3.Up * ptzMsg.yaw;
            PTZAxis1.RotationDegrees = Vector3.Right * ptzMsg.pitch;

            // User feedback
            GD.Print($"Setting PTZ rotation to (X:{ptzMsg.yaw},Y:{ptzMsg.pitch}) and zoom {ZoomAmount}");
            PTZRotX.Text = "Y:" + ptzMsg.yaw.ToString().PadLeft(3, ' ');
            PTZRotY.Text = "P:" + ptzMsg.pitch.ToString().PadLeft(3, ' ');

            ROS.Publish(PTZTopic, ptzMsg);
            if (ptzMsg.zoom_level != ZoomAmount)
            {
                // Don't blame me for this.
                // From PtzControl:
                //// 1: Absolute angle control (using yaw/pitch)
                //// 2: Single axis control (using axis_id and angle)
                //// 3: Absolute zoom control (using zoom_level)
                ptzMsg.control_mode = 3;
                ptzMsg.zoom_level = ZoomAmount;
                ROS.Publish(PTZTopic, ptzMsg);
                ptzMsg.control_mode = 1;
            }
        }

        public override bool AdvertiseToROS()
        {
            QOS ControlQOS = new(
                QOS.Policy.History.Keep_last,
                2,
                QOS.Policy.Reliability.Best_Effort,
                QOS.Policy.Durability.Volatile,
                QOS.Policy.Duration_Unspecified,
                QOS.Policy.Duration_Unspecified
            );
            ROS.AdvertiseTopic<CoreCtrlState>(CoreControlTopic, ControlQOS);
            ROS.AdvertiseTopic<Geometry.Twist>(TwistTopic, ControlQOS);
            ROS.AdvertiseTopic<PtzControl>(PTZTopic);

            const float deg2rad = Mathf.Pi / 180;
            const float rad2deg = 180 / Mathf.Pi;
            ROS.TopicSubscribe<NavSatFix>(GPSTopic, (fix) =>
            {
                const float minf = -71f * deg2rad;
                const float maxf = 127f * deg2rad;
                // // Calculates the average velocity of all motors while preventing outliers from impacting it
                // // That way if the rover is highcentered we don't report a speed of like 3mph
                // float median = (cf.fl_motor.velocity + cf.fr_motor.velocity + cf.bl_motor.velocity + cf.br_motor.velocity) * 0.25f; // float flMAD = Mathf.Abs(cf.fl_motor.velocity - median); // float frMAD = Mathf.Abs(cf.fr_motor.velocity - median); // float blMAD = Mathf.Abs(cf.bl_motor.velocity - median); // float brMAD = Mathf.Abs(cf.br_motor.velocity - median); // float MAD = (flMAD + frMAD + blMAD + brMAD) * 0.25f * 3f; // float sum = 0; // byte count = 0; // if(flMAD <= MAD) {sum += cf.fl_motor.velocity; count++;} // if(frMAD <= MAD) {sum += cf.fr_motor.velocity; count++;} // if(blMAD <= MAD) {sum += cf.bl_motor.velocity; count++;} // if(brMAD <= MAD) {sum += cf.br_motor.velocity; count++;} // count = byte.Clamp(count, 1, 4); // const float VelRatio = 16f; // float t = Mathf.Clamp((float)(sum / count) / VelRatio / 8, 0, 1); // Needle.SetDeferred(Control.PropertyName.Rotation, minf + t * (maxf - minf));

                double dLat = Mathf.Sin((fix.latitude - PrevFix.latitude) * 0.5d);
                dLat *= dLat;
                double dLon = Mathf.Sin((fix.longitude - PrevFix.longitude) * 0.5d);
                dLon *= dLon;
                dLon *= Mathf.Cos(PrevFix.latitude) * Mathf.Cos(fix.latitude);

                double a = dLat + dLon;
                double c = 2d * Mathf.Atan2(Mathf.Sqrt(a), Mathf.Sqrt(1d - a));
                double d = 3958.8d * c;

                double vel = d / (fix.header.stamp.sec - PrevFix.header.stamp.sec);

                Needle.SetDeferred(Control.PropertyName.Rotation, minf + Mathf.Clamp(d / 8d, 0d, 1d) * (maxf - minf));

                PrevFix = fix;
            });

            ROS.TopicSubscribe<Imu>(IMUTopic, (data) =>
            {
                Geometry.Quaternion q = data.orientation;
                double phi = Mathf.Atan2(2d * (q.w * q.z + q.x * q.y), 1d - 2d * (q.y * q.y + q.z * q.z)) * rad2deg;
                HeadingIndicator.SetDeferred(Control.PropertyName.Position, new Vector2((float)(phi * 1.188889d), -20f));
            });

            return true;
        }

        public override void EmitToROS()
        {
            twistMsg.linear = Lin;
            twistMsg.angular = Ang;

            ROS.Publish(TwistTopic, twistMsg);

            controlMsg.max_duty = MaxSpeed;
            controlMsg.brake_mode = BrakeState;

            ROS.Publish(CoreControlTopic, controlMsg);
        }

        public override void _ExitTree()
        {
            twistMsg.linear = new(0, 0, 0);
            twistMsg.angular = new(0, 0, 0);

            ROS.Publish(TwistTopic, twistMsg);

            controlMsg.max_duty = 0f;
            controlMsg.brake_mode = true;

            ROS.Publish(CoreControlTopic, controlMsg);
            ROS.TopicUnsubscribe(GPSTopic);
            ROS.TopicUnsubscribe(IMUTopic);
        }
    }
}
