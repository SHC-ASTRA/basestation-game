using IPC;
using Godot;
using RosSharp.RosBridgeClient.MessageTypes.Astra;

namespace ui
{
    public partial class ArmTabUI : BaseTabUI
    {
        private ArmManual msg = new();

        [ExportCategory("Arm")]
        [ExportSubgroup("Axese")]
        [Export]
        private MidwayHProgressBar Axis0drive;
        [Export]
        private MidwayHProgressBar Axis1drive;
        [Export]
        private MidwayHProgressBar Axis2drive;
        [Export]
        private MidwayHProgressBar Axis3drive;

        [ExportSubgroup("Brake")]
        [Export]
        private ColoredIndicator BrakeIndicator;
        [Export]
        private TextureRect Brake;
        public bool BrakeState = true;

        [ExportSubgroup("End Effector")]
        [Export]
        private MidwayHProgressBar EndEffectorRoll, EndEffectorYaw;
        [Export]
        private MidwayHProgressBar LinearActuatorDrive, GripperDrive;

        [ExportSubgroup("Laser")]
        [Export]
        private ColoredIndicator LaserIndicator;
        [Export]
        private TextureRect Laser;
        public bool LaserState = false;

        public override void _Ready()
        {
            base._Ready();
        }

        public override void _Process(double delta)
        {
            base._Process(delta);

            Axis0drive.Value = LeftStick.X;
            Axis1drive.Value = LeftStick.Y;
            Axis2drive.Value = RightStick.X;
            Axis3drive.Value = RightStick.Y;

            EndEffectorRoll.Value = (UpButton ? 1 : 0) + (DownButton ? -1 : 0);
            EndEffectorYaw.Value = (LeftButton ? -1 : 0) + (RightButton ? 1 : 0);

            GripperDrive.Value = RightTrigger - LeftTrigger;
            LinearActuatorDrive.Value = RightBumper - LeftBumper;

            bool b = BButton + AButton + XButton != 0;
            if (b != BrakeState)
            {
                BrakeState = b;
                BrakeIndicator.Value = b;
                Brake.Visible = b;
            }

            bool i = Mathf.RoundToInt(YButton) != 0;
            if (i != LaserState)
            {
                LaserState = i;
                LaserIndicator.Value = i;
                Laser.Visible = i;
            }
        }

        public override void AdvertiseToROS()
        {
            ROS.RequestTopic<ArmManual>(ControlTopicName);
        }

        public override void EmitToROS()
        {
            msg.axis0 = Mathf.RoundToInt(Axis0drive.Value);
            msg.axis1 = Mathf.RoundToInt(Axis1drive.Value);
            msg.axis2 = Mathf.RoundToInt(Axis2drive.Value);
            msg.axis3 = Mathf.RoundToInt(Axis3drive.Value);

            msg.brake = BrakeState;

            msg.effector_roll = Mathf.RoundToInt(EndEffectorRoll.Value);
            msg.effector_yaw = Mathf.RoundToInt(EndEffectorYaw.Value);

            msg.gripper = Mathf.RoundToInt(GripperDrive.Value);
            msg.linear_actuator = Mathf.RoundToInt(LinearActuatorDrive.Value);

            msg.laser = LaserState ? 1 : 0;

            ROS.Publish(ControlTopicName, msg);
        }

        public override void _ExitTree()
        {
            msg.axis0 = msg.axis1 = msg.axis2 = msg.axis3 = 0;
            msg.brake = true;
            msg.effector_roll = msg.effector_yaw = 0;
            msg.gripper = 0;
            msg.linear_actuator = msg.laser = 0;
            ROS.Publish(ControlTopicName, msg);
        }
    }
}
