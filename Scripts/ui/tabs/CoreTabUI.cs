using IPC;
using DEBUG;
using Godot;
using RosSharp.RosBridgeClient.MessageTypes.Astra;

namespace ui
{
    public partial class CoreTabUI : BaseTabUI
    {
        private CoreControl msg = new();
        private PtzControl ptzmsg = new();


        public const bool TankDriving = true;

        [ExportCategory("Core")]
        [ExportGroup("Motors")]
        [Export]
        public ProgressBar LMotor;
        [Export]
        public ProgressBar RMotor;

        [ExportGroup("PTZ")]
        [Export]
        public string ControlPTZTopic;

        [ExportSubgroup("Rendering")]
        [Export]
        public Camera3D PTZRenderCam;
        [Export]
        public SubViewport PTZSubViewport;
        [Export]
        public MeshInstance3D PTZAxis0;
        [Export]
        public MeshInstance3D PTZAxis1;

        [ExportSubgroup("Controlling")]
        [Export]
        public Label PTZRotX, PTZRotY;

        [Export]
        public VSlider Zoom;

        [ExportGroup("Brake")]
        [Export]
        private ColoredIndicator BrakeIndicator;
        [Export]
        private TextureRect Brake;
        public bool BrakeState = true;

        public int MaxSpeed = 60;
        private double MaxSpeedCollector = 0;

        public override void _Ready()
        {
            base._Ready();

            int windowSize = (int)(GetWindow().Size.Y * 0.125f);
            if (windowSize < 16) windowSize = 16;
            PTZSubViewport.Size = new Vector2I(windowSize, windowSize);

            GetTree().Root.SizeChanged += Resize;

            PTZRotX.Text = "Y:" + ptzmsg.yaw.ToString().PadLeft(3, ' ');
            PTZRotY.Text = "P:" + ptzmsg.pitch.ToString().PadLeft(3, ' ');
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
            }
            else
            {
#pragma warning disable
                LMotor.Value = LeftStick.Y + RightStick.X;
                RMotor.Value = LeftStick.Y - RightStick.X;
            }

            bool b = XButton + AButton + BButton > 0;
            if (b != BrakeState)
            {
                BrakeState = b;
                BrakeIndicator.Value = b;
                Brake.Visible = b;
            }

            if (UpButtonDown)
                PTZUp();
            else if (DownButtonDown)
                PTZDown();
            if (LeftButtonDown)
                PTZLeft();
            else if (RightButtonDown)
                PTZRight();

            if (RightTrigger > 0)
            {
                MaxSpeedCollector += delta * 15;
                if (MaxSpeedCollector > 1)
                {
                    MaxSpeedCollector = 0;
                    if (MaxSpeed < 100)
                        MaxSpeed++;
                }
            }
            else if (LeftTrigger > 0)
            {
                MaxSpeedCollector -= delta * 15;
                if (MaxSpeedCollector < -1)
                {
                    MaxSpeedCollector = 0;
                    if (MaxSpeed > 0)
                        MaxSpeed--;
                }
            }

            // Meant to stress-test ROSBridge
            // if (ROS.ROSReady && !threadstarted)
            // {
            //     new Thread(new e().a).Start();
            //     threadstarted = true;
            // }
        }

        public static float ConstWrap(float value, float min, float max)
        {
            float range = max - min;
            return min + ((((value - min) % range) + range) % range);
        }

        public void PTZUp()
        {
            ptzmsg.pitch += 5;
            PushToPTZ();
        }

        public void PTZDown()
        {
            ptzmsg.pitch -= 5;
            PushToPTZ();
        }

        public void PTZLeft()
        {
            ptzmsg.yaw -= 5;
            PushToPTZ();
        }

        public void PTZRight()
        {
            ptzmsg.yaw += 5;
            PushToPTZ();
        }

        public void PTZCenter()
        {
            ptzmsg.pitch = ptzmsg.yaw = 0;
            PushToPTZ();
        }

        public void PushToPTZ()
        {
            if (ptzmsg.pitch > 134)
                ptzmsg.pitch = 134;
            else if (ptzmsg.pitch < -134)
                ptzmsg.pitch = -134;

            ptzmsg.yaw = ConstWrap(ptzmsg.yaw, 0, 360);

            float ZoomAmount = (float)Zoom.Value;

            PTZAxis0.RotationDegrees = Vector3.Up * ptzmsg.yaw;
            PTZAxis1.RotationDegrees = Vector3.Right * ptzmsg.pitch;
            Debug.Log(DebugID, $"Setting PTZ rotation to (X:{ptzmsg.yaw},Y:{ptzmsg.pitch}) and zoom {ZoomAmount}");
            PTZRotX.Text = "Y:" + ptzmsg.yaw.ToString().PadLeft(3, ' ');
            PTZRotY.Text = "P:" + ptzmsg.pitch.ToString().PadLeft(3, ' ');

            ptzmsg.control_mode = 1;
            ROS.Publish(ControlPTZTopic, ptzmsg);
            if (ptzmsg.zoom_level != ZoomAmount)
            {
                ptzmsg.control_mode = 3;
                ptzmsg.zoom_level = ZoomAmount;
                ROS.Publish(ControlPTZTopic, ptzmsg);
            }
        }

        public override void AdvertiseToROS()
        {
            ROS.RequestTopic<CoreControl>(ControlTopicName);
            ROS.RequestTopic<PtzControl>(ControlPTZTopic);
        }

        public override void EmitToROS()
        {
            msg.left_stick = (float)LMotor.Value;
            msg.right_stick = (float)RMotor.Value;
            msg.max_speed = MaxSpeed;
            msg.brake = BrakeState;
            msg.turn_to_enable = false;
            msg.turn_to = 0f;
            msg.turn_to_timeout = 0f;

            ROS.Publish(ControlTopicName, msg);
        }

        public override void _ExitTree()
        {
            msg.left_stick = 0f;
            msg.right_stick = 0f;
            msg.max_speed = 0;
            msg.brake = true;
            msg.turn_to_enable = false;
            msg.turn_to = 0f;
            msg.turn_to_timeout = 0f;

            ROS.Publish(ControlTopicName, msg);
        }
    }

    // Meant to stress-test ROSBridge
    // public class e
    // {
    //     public void a()
    //     {
    //         ROS.ROSSocket.Advertise<SocketFeedback>("/arm/feedback/socket");
    //         System.Random r = new System.Random();
    //         SocketFeedback sf = new SocketFeedback(r.NextSingle(), r.NextSingle(), r.NextSingle(), r.NextSingle(), r.NextSingle(), r.NextSingle(),
    //             r.NextSingle(), r.NextSingle(), r.NextSingle(), r.NextSingle(), r.NextSingle(), r.NextSingle(), r.NextSingle(), r.NextSingle(),
    //             r.NextSingle(), r.NextSingle(), r.NextSingle(), r.NextSingle(), r.NextSingle(), r.NextSingle());
    //         while (true)
    //         {
    //             ROS.ROSSocket.Publish("/arm/feedback/socket", sf);
    //             Thread.Sleep(1);
    //         }
    //     }
    // }
}
