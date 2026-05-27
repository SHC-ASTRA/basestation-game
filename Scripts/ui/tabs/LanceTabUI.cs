using IPC;
using Godot;

namespace UI
{
    public partial class LanceTabUI : BaseTabUI
    {
        private readonly LanceControl lance = new();
        private const string LanceTopic = "/bio/control/lance";

        private readonly FireLibsRequest libsIn = new();
        private readonly FireLibsResponse libsOut = new();
        private const string LibsTopic = "/bio/libs/fire";

        [ExportCategory("LANCE")]
        [ExportGroup("Lance")]
        [Export]
        public Button DrillBoomDown;
        [Export]
        public Button DrillBoomUp;
        [Export]
        public ProgressBar DrillBoomSpeed;
        private float _DrillBoomValue;

        [ExportSubgroup("Scabbard")]
        [Export]
        public ProgressBar DrillSpeed;
        private double _DrillSpeedValue;

        [ExportGroup("Vacuum")]
        [Export]
        public Button VacuumBoomDown;
        [Export]
        public Button VacuumBoomUp;
        [Export]
        public ProgressBar VacuumBoomSpeed;
        private float _VacuumBoomValue;

        [ExportGroup("ARQUEBUS")]
        [Export]
        public Button Laser;
        [Export]
        public Button LaserFailsafe;
        private bool FailsafeEngaged;

        [ExportSubgroup("SFX")]
        [Export]
        public AudioStreamMP3 Failsafe;
        [Export]
        public AudioStreamMP3 Fire;
        [Export]
        public AudioStreamPlayer2D Source;

        private double LastTriggerV;

        public override void _Ready()
        {
            base._Ready();

            Rate = 45;

            LaserFailsafe.Toggled += (t) =>
            {
                // Only play the failsafe engagement sound if
                // user toggled the failsafe on
                if (t)
                {
                    Source.Stream = Failsafe;
                    Source.Play();
                }
                FailsafeEngaged = t;
            };
            Laser.Pressed += () =>
            {
                // Only do something is the failsafe is engaged
                if (FailsafeEngaged)
                {
                    Source.Stream = Fire;
                    Source.Play();
                    libsIn.unique_number = new System.Random().NextInt64();
                    ROS.PublishServiceGoal<FireLibsRequest, FireLibsResponse>(LibsTopic, (_) => { }, libsIn);
                    FailsafeEngaged = false;
                }
            };
        }

        public override void _Process(double d)
        {
            base._Process(d);
            double TriggerV = Mathf.Round(RightTrigger - LeftTrigger);
            DrillSpeed.Value = _DrillSpeedValue = Mathf.Clamp(_DrillSpeedValue += TriggerV != LastTriggerV ? TriggerV * 0.1 : 0, -1, 1);
            LastTriggerV = TriggerV;
            // Embedded only responds to whole values at the moment. In the future, can remove Mathf.Round
            DrillBoomSpeed.Value = _DrillBoomValue = (DrillBoomDown.ButtonPressed ? 1 : 0) - (DrillBoomUp.ButtonPressed ? 1 : 0) + Mathf.Round(LeftStick.Y);
            VacuumBoomSpeed.Value = _VacuumBoomValue = (VacuumBoomDown.ButtonPressed ? 1 : 0) - (VacuumBoomUp.ButtonPressed ? 1 : 0) + Mathf.Round(RightStick.Y);
        }

        public override bool AdvertiseToROS()
        {
            ROS.AdvertiseTopic<LanceControl>(LanceTopic);

            ROS.AdvertiseService<FireLibsRequest, FireLibsResponse>(
                LibsTopic,
                (FireLibsRequest i, out FireLibsResponse o) => { o = libsOut; return true; }
            );
            return true;
        }

        public override void EmitToROS()
        {
            lance.drill_arm_ctrl = _DrillBoomValue;
            lance.drill_speed = (float)_DrillSpeedValue;
            lance.drill_laser = AButton > 0;
            lance.vacuum_arm_ctrl = _VacuumBoomValue;
            ROS.Publish(LanceTopic, lance);
        }

        public override void _ExitTree()
        {
            ROS.ROSSocket.Unadvertise(LanceTopic);
        }
    }
}
