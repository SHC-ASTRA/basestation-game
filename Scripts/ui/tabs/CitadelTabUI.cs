using IPC;
using Godot;
using RosSharp.RosBridgeClient.Actionlib;
using RosSharp.RosBridgeClient.MessageTypes.Std;
using RosSharp.RosBridgeClient.MessageTypes.Astra;
using RosSharp.RosBridgeClient.MessageTypes.Action;

namespace UI
{
    public partial class CitadelTabUI : BaseTabUI
    {
        // Vacuum Action
        ROSActionClient<
            BioVacuumAction,
            BioVacuumActionGoal, BioVacuumActionResult, BioVacuumActionFeedback,
            BioVacuumGoal, BioVacuumResult, BioVacuumFeedback
        > VacuumClient;
        BioVacuumGoal BVAG = new BioVacuumGoal();
        private BioVacuumAction BioVacuumHandler = new();
        private const string VacuumCtrl = "/bio/actions/vacuum";

        // Distributor Message
        private BioDistributor distributors = new();
        private const string DistributorTopic = "/bio/control/distributor";

        // Scythe Message
        private ScytheControl scythe = new();
        private const string ScytheTopic = "/bio/control/scythe";

        // TestTube Service
        private BioTestTubeRequest tubeMsg = new();
        private const string TubeCtrl = "/bio/control/test_tube";

        [ExportCategory("CITADEL")]
        [ExportGroup("Vacuum system")]
        [Export]
        public SpinBox FanDuty;
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
        private Button Distributor0;
        [Export]
        private Button Distributor1;
        [Export]
        private Button Distributor2;

        [ExportGroup("Test Tubes")]
        [Export]
        public SpinBox TubesExtensionAmount;
        [Export]
        public SelectionBox TestTubeSelector;
        [Export]
        public Button TestTubeCommit;
        [Export]
        public TextureRect TestTubeCommitTexture;

        [ExportGroup("Scythe")]
        [Export]
        public Button ScytheUp, ScytheDown;
        [Export]
        public ProgressBar scytheMovement;
        private float scythePosRel;

        public override void _Ready()
        {
            base._Ready();

            Rate = 15;

            VacuumCommit.ButtonDown += () =>
            {
                VacuumCommitTexture.Visible = true;
                BVAG.valve_id = (byte)VacuumSelector.prevIndex;
                BVAG.fan_time_ms = (uint)FanTime.Value;
                BVAG.fan_duty_cycle_percent = (byte)FanDuty.Value;
                VacuumClient.PublishActionGoal(BVAG);
            };
            VacuumCommit.ButtonUp += () => { VacuumCommitTexture.Visible = false; };


            TestTubeCommit.ButtonDown += () =>
            {
                TestTubeCommitTexture.Visible = true;
                tubeMsg.tube_id = (byte)TestTubeSelector.prevIndex;
                tubeMsg.milliliters = (float)TubesExtensionAmount.Value;
                ROS.PublishServiceGoal<BioTestTubeRequest, BioTestTubeResponse>(TubeCtrl, (_) => { }, tubeMsg);
            };
            TestTubeCommit.ButtonUp += () => { TestTubeCommitTexture.Visible = false; };
        }

        public override void AdvertiseToROS()
        {
            // HP Lovecraft couldn't dream of something this horrendous
            VacuumClient = ROS.AdvertiseAction<
                BioVacuumAction,
                BioVacuumActionGoal, BioVacuumActionResult, BioVacuumActionFeedback,
                BioVacuumGoal, BioVacuumResult, BioVacuumFeedback
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

            ROS.AdvertiseService<BioTestTubeRequest, BioTestTubeResponse>(
                serviceName: TubeCtrl,
                handler: (BioTestTubeRequest _, out BioTestTubeResponse a) =>
                {
                    GD.Print($"Extending tube {_.tube_id}, By {_.milliliters}%");
                    a = new();
                    return true;
                }
            );

            ROS.AdvertiseMessage<BioDistributor>(DistributorTopic);
            ROS.AdvertiseMessage<ScytheControl>(ScytheTopic);
        }

        public override void _Process(double d)
        {
            base._Process(d);
            scythePosRel = (ScytheUp.ButtonPressed ? 1 : 0) - (ScytheDown.ButtonPressed ? 1 : 0) + LeftStick.Y;
            scytheMovement.Value = scythePosRel;
        }

        public override void EmitToROS()
        {
            distributors.distributor_id[0] = Distributor0.ButtonPressed;
            distributors.distributor_id[1] = Distributor1.ButtonPressed;
            distributors.distributor_id[2] = Distributor2.ButtonPressed;
            ROS.Publish(DistributorTopic, distributors);

            scythe.move_scythe = scythePosRel;
            ROS.Publish(ScytheTopic, scythe);
        }

        public override void _ExitTree()
        {
            ROS.ROSSocket.Unadvertise(DistributorTopic);
            ROS.ROSSocket.Unadvertise(ScytheTopic);
        }
    }
}
