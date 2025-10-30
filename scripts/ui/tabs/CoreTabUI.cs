using IPC;
using DEBUG;
using Godot;

namespace ui
{
    public partial class CoreTabUI : BaseTabUI
    {
        private static int? debugID;

        public const bool TankDriving = true;

        [Export]
        public ProgressBar LMotor, RMotor;

        [ExportCategory("PTZ")]
        [Export]
        public Camera3D PTZRenderer;
        [Export]
        public SubViewport PTZSVP;
        [Export]
        public MeshInstance3D PTZAxis0;
        [Export]
        public MeshInstance3D PTZAxis1;

        [Export]
        public Label PTZRot;

        [Export]
        public VSlider Zoom;

        public Vector3I PTZRotation;
        public float PTZZoom;

        public override void _Ready()
        {
            debugID = Debug.RegisterDebugData(Variant.From<string>(""));
            base._Ready();
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

            if (UpButton)
                PTZUp();
            else if (DownButton)
                PTZDown();
            if (LeftButton)
                PTZLeft();
            else if (RightButton)
                PTZRight();
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

        public void PTZCenter()
        {
            PTZRotation = Vector3I.Zero;
            PushToPTZ();
        }

        public void PTZZoomIn()
        {
            PTZZoom += 0.05f;
            PushToPTZ();
        }

        public void PTZZoomOut()
        {
            PTZZoom -= 0.05f;
            PushToPTZ();
        }

        public void PushToPTZ()
        {
            if (PTZRotation.Y > 105)
                PTZRotation.Y = 105;
            else if (PTZRotation.Y < -105)
                PTZRotation.Y = -105;
            PTZRotation.X = ConstWrap(ref PTZRotation.X, -180, 180);
            PTZAxis0.RotationDegrees = Vector3.Up * PTZRotation.X;
            PTZAxis1.RotationDegrees = Vector3.Right * PTZRotation.Y;
            Debug.Log(debugID, $"Setting PTZ rotation to {PTZRotation} and zoom {PTZZoom}");
            ROS.Publish("/test", new RosSharp.RosBridgeClient.MessageTypes.Std.String(""));
            PTZRot.Text = string.Join(' ', "Y:", PTZRotation.X, "P:", PTZRotation.Y, "R:", PTZRotation.Z);
        }
    }
}
