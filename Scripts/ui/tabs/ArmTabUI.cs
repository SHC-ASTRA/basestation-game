using IPC;
using Godot;
using RosSharp.RosBridgeClient.MessageTypes.Astra;

namespace ui
{
    public partial class ArmTabUI : BaseTabUI
    {
        private ArmManual msg = new();

        [ExportCategory("Arm")]
        [Export]
        public Panel brake;

        [ExportSubgroup("Axese")]
        [Export]
        public MidwayHProgressBar Axis0drive, Axis1drive, Axis2drive, Axis3drive;

        [ExportCategory("End Effector")]
        [Export]
        public Panel endEffectorAttitude;

        [Export]
        public ProgressBar gripperDrive;

        [Export]
        public ProgressBar linearActuatorDrive;

        [Export]
        public Panel laserPanel;

        public override void _Ready()
        {
            base._Ready();
        }

        public override void _Process(double delta)
        {
            base._Process(delta);

            Axis0drive.Parallax = LeftStick.X;
            Axis1drive.Parallax = LeftStick.Y;
            Axis2drive.Parallax = RightStick.X;
            Axis3drive.Parallax = RightStick.Y;
        }

        public override void AdvertiseToROS()
        {
            ROS.RequestTopic<ArmManual>(TopicName);
        }

        public override void EmitToROS()
        {

        }
    }
}
