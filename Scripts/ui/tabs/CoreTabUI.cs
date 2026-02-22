using IPC;
using Godot;
using System.Linq;
using RosSharp.RosBridgeClient.MessageTypes.Astra;
using Geometry = RosSharp.RosBridgeClient.MessageTypes.Geometry;

namespace UI
{
    public partial class CoreTabUI : BaseTabUI
    {
        private CoreCtrlState controlMsg = new();
        private const string CoreControlTopic = "/core/control/state";

        private Geometry.Twist twistMsg = new() { angular = new(0, 0, 0), linear = new(0, 0, 0) };
        private const string TwistTopic = "/core/twist";

        private PtzControl ptzMsg = new();
        private const string PTZTopic = "/ptz/control";

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
        public Camera3D PTZRenderCam;
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
            Button[] PTZButtons = [.. PTZButtonCont.GetChildren().Where(static _ => _ is Button).Cast<Button>()];

            // I know this is ugly...
            PTZButtons[0].Pressed += () => { ptzMsg.pitch += 5; PushToPTZ(); };
            PTZButtons[1].Pressed += () => { ptzMsg.yaw -= 5; PushToPTZ(); };
            PTZButtons[2].Pressed += () => { ptzMsg.pitch = ptzMsg.yaw = 0; PushToPTZ(); };
            PTZButtons[3].Pressed += () => { ptzMsg.yaw += 5; PushToPTZ(); };
            PTZButtons[4].Pressed += () => { ptzMsg.pitch -= 5; PushToPTZ(); };

            DrivingMode.Toggled += (pressed) => { TankDriving = pressed; (DrivingMode.GetChild(0) as TextureRect).Texture = pressed ? Tank : Wheel; };
        }

        public void Resize()
        {
            if (this.Size.Y < WindowSize.Y * 0.125f)
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

            // If ANY of these buttons are pressed, rover brakes
            bool b = XButton + AButton + BButton > 0;
            if (b != BrakeState)
            {
                BrakeState = b;
                BrakeIndicator.Value = b;
                Brake.Visible = b;
            }

            // (In/De)crease boost mode amount. Collector
            // stores the frame delta so that it takes a while to actually
            // increase the max speed
            if (RightTrigger >= 1f && LeftTrigger >= 1f)
            {
                MaxSpeed = 0.5f;
                MaxMotorDrive.Value = MaxSpeed * 100f;
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

            // Used to keep PTZ in check
            WindowSize = GetWindow().Size;
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
            if (ptzMsg.pitch > 134)
                ptzMsg.pitch = 134;
            else if (ptzMsg.pitch < -134)
                ptzMsg.pitch = -134;

            ptzMsg.yaw = ConstWrap(ptzMsg.yaw, 0, 360);

            // Literally just caching the conversion
            float ZoomAmount = (float)Zoom.Value;

            PTZAxis0.RotationDegrees = Godot.Vector3.Up * ptzMsg.yaw;
            PTZAxis1.RotationDegrees = Godot.Vector3.Right * ptzMsg.pitch;

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

        public override void AdvertiseToROS()
        {
            ROS.AdvertiseMessage<CoreCtrlState>(CoreControlTopic);
            ROS.AdvertiseMessage<Geometry.Twist>(TwistTopic);
            ROS.AdvertiseMessage<PtzControl>(PTZTopic);
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
        }
    }
}
