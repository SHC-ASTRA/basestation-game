using IPC;
using Godot;
using RosSharp.RosBridgeClient.MessageTypes.Astra;

namespace UI
{
    public partial class ArmTabUI : BaseTabUI
    {
        private ArmManual controlMsg = new();
        private const string ControlTopic = "/arm/control/manual";

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
        private int EFRoll, EFYaw;

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

            EFRoll = (UpButton ? 1 : 0) + (DownButton ? -1 : 0);
            EndEffectorRoll.Value = EFRoll;
            EFYaw = (LeftButton ? -1 : 0) + (RightButton ? 1 : 0);
            EndEffectorYaw.Value = EFYaw;

            GripperDrive.Value = RightTrigger - LeftTrigger;
            LinearActuatorDrive.Value = RightBumper - LeftBumper;

            bool b = BButton + AButton + XButton != 0;
            if (b != BrakeState)
            {
                BrakeState = b;
                BrakeIndicator.Value = b;
                Brake.Visible = b;
            }
            // Just reusing the variable
            b = Mathf.RoundToInt(YButton) != 0;
            if (b != LaserState)
            {
                LaserState = b;
                LaserIndicator.Value = b;
                Laser.Visible = b;
            }
        }

        public override void AdvertiseToROS()
        {
            ROS.AdvertiseMessage<ArmManual>(ControlTopic);
        }

        public override void EmitToROS()
        {
            controlMsg.axis0 = Mathf.RoundToInt(LeftStick.X);
            controlMsg.axis1 = Mathf.RoundToInt(LeftStick.Y);
            controlMsg.axis2 = Mathf.RoundToInt(RightStick.X);
            controlMsg.axis3 = Mathf.RoundToInt(RightStick.Y);

            controlMsg.brake = BrakeState;

            controlMsg.effector_roll = Mathf.RoundToInt(EFRoll);
            controlMsg.effector_yaw = Mathf.RoundToInt(EFYaw);

            controlMsg.gripper = Mathf.RoundToInt(RightTrigger - LeftTrigger);
            controlMsg.linear_actuator = Mathf.RoundToInt(RightBumper - LeftBumper);

            controlMsg.laser = LaserState ? 1 : 0;

            ROS.Publish(ControlTopic, controlMsg);
        }

        public override void _ExitTree()
        {
            controlMsg.axis0 = controlMsg.axis1 = controlMsg.axis2 = controlMsg.axis3 = 0;
            controlMsg.brake = true;
            controlMsg.effector_roll = controlMsg.effector_yaw = 0;
            controlMsg.gripper = 0;
            controlMsg.linear_actuator = controlMsg.laser = 0;
            ROS.Publish(ControlTopic, controlMsg);
        }
    }
}
