using IPC;
using Godot;
using RosSharp.RosBridgeClient.MessageTypes.Astra;

namespace UI
{
    public partial class LanceTabUI : BaseTabUI
    {
        private FaerieControl faerie = new();
        private const string FaerieTopic = "/bio/control/faerie";

        private LibsSystemRequest libsIn = new(true);
        private LibsSystemResponse libsOut = new();
        private const string LibsTopic = "/bio/control/libs_system";

        [ExportCategory("LANCE")]
        [ExportGroup("Faerie")]
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
        private float _DrillSpeedValue;

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

        public override void _Ready()
        {
            base._Ready();

            Rate = 15;

            LaserFailsafe.Toggled += (bool t) =>
            {
                // Only play the failsafe engagement sound if
                // user toggled the failsafe on
                if (t)
                {
                    Source.Stream = Failsafe;
                    Source.Play();
                }
                else libsIn.fire_laser = false;
                FailsafeEngaged = t;
            };
            Laser.Pressed += () =>
            {
                // Only do something is the failsafe is engaged
                if (FailsafeEngaged)
                {
                    Source.Stream = Fire;
                    Source.Play();
                    libsIn.fire_laser = true;
                    ROS.PublishServiceGoal<LibsSystemRequest, LibsSystemResponse>(LibsTopic, (_) => { }, libsIn);
                }
                else libsIn.fire_laser = false;
            };
        }

        public override void _Process(double d)
        {
            base._Process(d);
            DrillSpeed.Value = _DrillSpeedValue = RightTrigger - LeftTrigger;
            // Embedded only responds to whole values at the moment. In the future, can remove Mathf.Round
            DrillBoomSpeed.Value = _DrillBoomValue = (DrillBoomDown.ButtonPressed ? 1 : 0) - (DrillBoomUp.ButtonPressed ? 1 : 0) + Mathf.Round(LeftStick.Y);
        }

        public override void AdvertiseToROS()
        {
            ROS.AdvertiseTopic<FaerieControl>(FaerieTopic);

            ROS.AdvertiseService<LibsSystemRequest, LibsSystemResponse>(
                LibsTopic,
                (LibsSystemRequest i, out LibsSystemResponse o) => { o = libsOut; return true; }
            );
        }

        public override void EmitToROS()
        {
            faerie.move_faerie = _DrillBoomValue;
            faerie.drill_speed = _DrillSpeedValue;
            ROS.Publish(FaerieTopic, faerie);
        }

        public override void _ExitTree()
        {
            ROS.ROSSocket.Unadvertise(FaerieTopic);
        }
    }
}
