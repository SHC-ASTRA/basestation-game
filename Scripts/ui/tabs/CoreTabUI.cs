using IPC;
using DEBUG;
using Godot;
using RosSharp.RosBridgeClient.MessageTypes.Astra;

namespace ui
{
    public partial class CoreTabUI : BaseTabUI
    {
        private CoreControl msg = new();

        public const bool TankDriving = true;

        [Export]
        public ProgressBar LMotor, RMotor;

        [ExportCategory("PTZ")]
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
        public Label PTZRotX, PTZRotY, PTZRotZ;

        [Export]
        public VSlider Zoom;

        public Vector3I PTZRotation;
        public float PTZZoom;

        public override void _Ready()
        {
            base._Ready();

            int windowSize = (int)(GetWindow().Size.Y * 0.125f);
            if (windowSize < 16) windowSize = 16;
            PTZSubViewport.Size = new Vector2I(windowSize, windowSize);

            PTZRotX.Text = "Y:" + PTZRotation.X.ToString().PadLeft(3, ' ');
            PTZRotY.Text = "P:" + PTZRotation.Y.ToString().PadLeft(3, ' ');
            PTZRotZ.Text = "R:" + PTZRotation.Z.ToString().PadLeft(3, ' ');
        }

        public override void _Process(double delta)
        {
            base._Process(delta);

            if (this.Size.Y < WindowSize.Y * 0.125f)
            {
                PTZSubViewport.Size = new Vector2I((int)(WindowSize.Y * 0.125f), (int)(WindowSize.Y * 0.125f));
            }

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

            if (UpButton)
                PTZUp();
            else if (DownButton)
                PTZDown();
            if (LeftButton)
                PTZLeft();
            else if (RightButton)
                PTZRight();

            // Meant to stress-test ROSBridge
            // if (ROS.ROSReady && !threadstarted)
            // {
            //     new Thread(new e().a).Start();
            //     threadstarted = true;
            // }
        }

        public static int ConstWrap(ref int value, int min, int max)
        {
            int range = max - min;
            return min + ((((value - min) % range) + range) % range);
        }

        public void PTZUp()
        {
            PTZRotation.Y += 5;
            PushToPTZ();
        }

        public void PTZDown()
        {
            PTZRotation.Y -= 5;
            PushToPTZ();
        }

        public void PTZLeft()
        {
            PTZRotation.X -= 5;
            PushToPTZ();
        }

        public void PTZRight()
        {
            PTZRotation.X += 5;
            PushToPTZ();
        }

        public void PTZRollLeft()
        {
            PTZRotation.Z -= 5;
            PushToPTZ();
        }

        public void PTZRollRight()
        {
            PTZRotation.Z += 5;
            PushToPTZ();
        }

        public void PTZCenter()
        {
            PTZRotation = Vector3I.Zero;
            PushToPTZ();
        }


        public void PushToPTZ()
        {
            if (PTZRotation.Y > 105)
                PTZRotation.Y = 105;
            else if (PTZRotation.Y < -105)
                PTZRotation.Y = -105;

            PTZRotation.Z = ConstWrap(ref PTZRotation.Z, 0, 360);

            PTZRotation.X = ConstWrap(ref PTZRotation.X, 0, 360);

            PTZAxis0.RotationDegrees = Vector3.Up * PTZRotation.X;
            PTZAxis1.RotationDegrees = Vector3.Right * PTZRotation.Y;
            Debug.Log(DebugID, $"Setting PTZ rotation to {PTZRotation} and zoom {PTZZoom}");
            PTZRotX.Text = "Y:" + PTZRotation.X.ToString().PadLeft(3, ' ');
            PTZRotY.Text = "P:" + PTZRotation.Y.ToString().PadLeft(3, ' ');
            PTZRotZ.Text = "R:" + PTZRotation.Z.ToString().PadLeft(3, ' ');
        }

        public override void AdvertiseToROS()
        {
            ROS.RequestTopic<CoreControl>(TopicName);
        }

        public override void EmitToROS()
        {
            msg.left_stick = (float)LMotor.Value;
            msg.right_stick = (float)RMotor.Value;
            msg.max_speed = 60;
            msg.brake = BButton > 0;
            msg.turn_to_enable = false;
            msg.turn_to = 0f;
            msg.turn_to_timeout = 0f;

            ROS.Publish("/core/control", msg);
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
