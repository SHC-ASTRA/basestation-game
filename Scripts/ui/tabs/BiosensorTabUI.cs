using IPC;
using Godot;
using Godot.Collections;
using RosSharp.RosBridgeClient.Actionlib;
using RosSharp.RosBridgeClient.MessageTypes.Std;
using RosSharp.RosBridgeClient.MessageTypes.Astra;
using RosSharp.RosBridgeClient.MessageTypes.Action;

namespace ui
{
    public partial class BiosensorTabUI : BaseTabUI
    {
        ROSActionClient<BioVacuumAction, BioVacuumActionGoal, BioVacuumActionResult, BioVacuumActionFeedback, BioVacuumGoal, BioVacuumResult, BioVacuumFeedback> VacuumClient;
        BioVacuumActionGoal BVAG = new BioVacuumActionGoal(new Header(), new GoalInfo(), new BioVacuumGoal());
        private BioVacuumAction BioVacuumHandler = new();
        private const string VacuumCtrl = "/valve_fan";

        private BioDistributor controlMsg = new();
        private const string ctrl = "/bio/control/distributor";

        private BioTestTubeRequest tubeMsg = new();
        private const string TubeCtrl = "/bio/control/testtubes";

        [ExportCategory("CITADEL")]

        [ExportGroup("Vacuum system")]
        [Export]
        public SpinBox FanSpeed;
        [Export]
        public SpinBox FanTime;
        [Export]
        public SelectionBox VacuumSelector;

        [Export]
        public Button VacuumCommit;
        [Export]
        public TextureRect VacuumCommitTexture;

        [ExportGroup("Distributors")]
        [Export]
        public Array<Button> Distributors;

        [ExportGroup("Test Tubes")]
        [Export]
        public SpinBox ExtensionAmount;
        [Export]
        public SelectionBox TestTubeSelector;
        [Export]
        public Button TestTubeCommit;
        [Export]
        public TextureRect TestTubeCommitTexture;

        [ExportGroup("Arm")]
        [Export]
        public Button VibrationMotor;

        [ExportCategory("FAERIE")]
        [Export]
        public Button Laser;
        [Export]
        public Button LaserFailsafe;

        [ExportCategory("SFX")]
        [Export]
        public AudioStreamMP3 Failsafe;
        [Export]
        public AudioStreamMP3 Fire;
        [Export]
        public AudioStreamPlayer2D Source;

        public override void _Ready()
        {
            base._Ready();

            VacuumCommit.ButtonDown += () =>
            {
                VacuumCommitTexture.Visible = true;
                BVAG.args.valve_id = (sbyte)VacuumSelector.prevIndex;
                BVAG.args.fan_time_ms = (int)FanTime.Value;
                BVAG.args.fan_duty_cycle = (sbyte)FanTime.Value;
                VacuumClient.PublishActionGoal(BVAG);
            };
            VacuumCommit.ButtonUp += () => { VacuumCommitTexture.Visible = false; };


            TestTubeCommit.ButtonDown += () =>
            {
                TestTubeCommitTexture.Visible = true;
                tubeMsg.tube_id = (sbyte)TestTubeSelector.prevIndex;
                tubeMsg.extension_percent = (sbyte)ExtensionAmount.Value;
                ROS.PublishServiceGoal<BioTestTubeRequest, BioTestTubeResponse>(TubeCtrl, (_) => { }, tubeMsg);
            };
            TestTubeCommit.ButtonUp += () => { TestTubeCommitTexture.Visible = false; };


            LaserFailsafe.Toggled += (bool t) => { if (t) { Source.Stream = Failsafe; Source.Play(); } };
            Laser.Pressed += () => { Source.Stream = Fire; Source.Play(); };
        }

        public override void AdvertiseToROS()
        {
            // HP Lovecraft couldn't dream of something this horrendous
            VacuumClient = ROS.AdvertiseAction<
                BioVacuumAction,
                BioVacuumActionGoal,
                BioVacuumActionResult,
                BioVacuumActionFeedback,
                BioVacuumGoal,
                BioVacuumResult,
                BioVacuumFeedback
            >(
                actionName: VacuumCtrl,
                act: new BioVacuumAction(),
                actionGoalHandler: (_) => { },
                actionCancelHandler: (_, _) => { },
                goalStatus: new GoalStatus(),
                feedbackCallback: () => { },
                resultCallback: () =>
                {
                    if (VacuumClient.lastResultSuccess == false)
                        GD.Print("Request failed!");
                },
                statusCallback: () =>
                {
                    GD.Print("\n" + ((ActionStatus)(VacuumClient.goalStatus.status)).ToString() + "\n");
                }
            );

            ROS.AdvertiseService<
                BioTestTubeRequest,
                BioTestTubeResponse
            >(
                serviceName: TubeCtrl,
                handler: (BioTestTubeRequest _, out BioTestTubeResponse a) =>
                {
                    GD.Print($"Extending tube {_.tube_id}, By {_.extension_percent}%");
                    a = new();
                    return true;
                }
            );

            ROS.AdvertiseMessage<BioDistributor>(ctrl);
        }

        public override void EmitToROS()
        {
            controlMsg.distibutor[0] = Distributors[0].ButtonPressed;
            controlMsg.distibutor[1] = Distributors[1].ButtonPressed;
            controlMsg.distibutor[2] = Distributors[2].ButtonPressed;
            ROS.Publish(ctrl, controlMsg);
        }

        public override void _ExitTree()
        {
            ROS.ROSSocket.Unadvertise("/valve_fan");
        }
    }
}
