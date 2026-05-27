using IPC;
using Godot;
using RosSharp.RosBridgeClient.Actionlib;
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
		private readonly BioVacuumGoal BVAG = new();
		private const string VacuumCtrl = "/bio/control/vacuum";

		// Citadel Message
		private readonly CitadelControl citadelMsg = new();
		private const string citadelTopic = "/bio/control/citadel";

		// TestTube Service
		private readonly BioTestTubeRequest tubeMsg = new();
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
		public SpinBox TestTubeSelector;
		[Export]
		public Button TestTubeCommit;
		[Export]
		public TextureRect TestTubeCommitTexture;

		public override void _Ready()
		{
			base._Ready();

			Rate = 50;

			VacuumCommit.ButtonDown += () =>
			{
				VacuumCommitTexture.Visible = true;
				BVAG.valve_id = (byte)VacuumSelector.prevIndex;
				BVAG.fan_time_ms = (uint)FanTime.Value * 1000;
				BVAG.fan_duty_cycle_percent = (byte)FanDuty.Value;
				VacuumClient.PublishActionGoal(BVAG);
			};
			VacuumCommit.ButtonUp += () => { VacuumCommitTexture.Visible = false; };

			TestTubeCommit.ButtonDown += () =>
			{
				TestTubeCommitTexture.Visible = true;
				tubeMsg.tube_id = (byte)Mathf.RoundToInt(TestTubeSelector.Value);
				tubeMsg.extended = true;
				ROS.PublishServiceGoal<BioTestTubeRequest, BioTestTubeResponse>(TubeCtrl, (_) => { }, tubeMsg);
			};
			TestTubeCommit.ButtonUp += () => { TestTubeCommitTexture.Visible = false; };
		}

		public override bool AdvertiseToROS()
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
					GD.Print("\n" + ((ActionStatus)VacuumClient.goalStatus.status).ToString() + "\n");
				}
			);

			ROS.AdvertiseService<BioTestTubeRequest, BioTestTubeResponse>(
				serviceName: TubeCtrl,
				handler: (BioTestTubeRequest _, out BioTestTubeResponse response) =>
				{
					GD.Print($"Extending tube {_.tube_id}, By {_.extended}%");
					response = new();
					return true;
				}
			);

			ROS.AdvertiseTopic<CitadelControl>(citadelTopic);

			return true;
		}

		public override void EmitToROS()
		{
			citadelMsg.distributors_ctrl[0] = Distributor0.ButtonPressed;
			citadelMsg.distributors_ctrl[1] = Distributor1.ButtonPressed;
			citadelMsg.distributors_ctrl[2] = Distributor2.ButtonPressed;
			ROS.Publish(citadelTopic, citadelMsg);
		}

		public override void _ExitTree()
		{
			ROS.ROSSocket.Unadvertise(citadelTopic);
		}
	}
}
