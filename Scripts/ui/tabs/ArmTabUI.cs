using IPC;
using Godot;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Control;

namespace UI
{
    public partial class ArmTabUI : BaseTabUI
    {
        private JointJog controlMsg = new();
        private const string ControlTopic = "/arm/control/manual";

        private ArmCtrlState controlTwist = new();
        private const string ControlState = "/arm/control/state";

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

        private const int
            Axis0 = 0,
            Axis1 = 1,
            Axis2 = 2,
            Axis3 = 3,
            WristYaw = 4,
            WristRoll = 5,
            EFGripper = 6;

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

        public override bool AdvertiseToROS()
        {
            // controlMsg.joint_names = URDFTranslator.JointNames(URDFDownloader.Robots["Arm"]);
            controlMsg.joint_names = [
                "axis_0_joint",
                "axis_1_joint",
                "axis_2_joint",
                "axis_3_joint",
                "wrist_yaw_joint",
                "wrist_roll_joint",
                "ef_gripper_left_joint",
            ];
            controlMsg.velocities = new double[controlMsg.joint_names.Length];

            QOS ControlQOS = new QOS(
                QOS.Policy.History.Keep_last,
                2,
                QOS.Policy.Reliability.Best_Effort,
                QOS.Policy.Durability.Volatile,
                QOS.Policy.Duration_Unspecified,
                QOS.Policy.Duration_Unspecified
            );
            ROS.AdvertiseTopic<JointJog>(ControlTopic, ControlQOS);
            ROS.AdvertiseTopic<ArmCtrlState>(ControlState, ControlQOS);
            return true;
        }

        public override void EmitToROS()
        {
            controlMsg.velocities[Axis0] = LeftStick.X;
            controlMsg.velocities[Axis1] = LeftStick.Y;
            controlMsg.velocities[Axis2] = RightStick.X;
            controlMsg.velocities[Axis3] = RightStick.Y;

            controlMsg.velocities[WristYaw] = Mathf.RoundToInt(EFYaw);
            controlMsg.velocities[WristRoll] = Mathf.RoundToInt(EFRoll);

            controlMsg.velocities[EFGripper] = Mathf.RoundToInt(RightTrigger - LeftTrigger);

            controlTwist.laser = LaserState;
            controlTwist.brake_mode = BrakeState;

            ROS.Publish(ControlTopic, controlMsg);
            ROS.Publish(ControlState, controlTwist);
        }

        public override void _ExitTree()
        {
            controlMsg.velocities[Axis0] = controlMsg.velocities[Axis1] = controlMsg.velocities[Axis2] = controlMsg.velocities[Axis3] = 0;
            controlTwist.brake_mode = true;
            controlMsg.velocities[WristRoll] = controlMsg.velocities[WristYaw] = 0;
            controlMsg.velocities[EFGripper] = 0;
            controlTwist.laser = false;
            ROS.Publish(ControlTopic, controlMsg);
            ROS.Publish(ControlState, controlTwist);
        }
    }
}
