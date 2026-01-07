using IPC;
using Godot;
using System.Collections.Generic;
using RosSharp.RosBridgeClient.MessageTypes.Astra;

namespace ui
{
    public partial class BiosensorTabUI : BaseTabUI
    {
        private BioControl msg = new();

        [ExportCategory("CITADEL")]
        [ExportGroup("Pumps")]
        [Export]
        public SpinBox[] PumpAmounts;
        [Export]
        public Button[] PumpRuns;
        private Queue<(int, float)> PumpQueue = new();

        [ExportGroup("Fans")]
        [Export]
        public SpinBox[] FanDurations;
        [Export]
        public Button[] FanRuns;
        private Queue<(int, int)> FanQueue = new();

        [ExportGroup("Servos")]
        [Export]
        public Button[] ServoRun;
        private Queue<(int, bool)> ServoQueue = new();

        [ExportGroup("Arm")]
        [Export]
        public VSlider BioArm;
        [Export]
        public Button VibrationMotor;

        [ExportCategory("FAERIE")]
        [Export]
        public Button Laser;
        [Export]
        public Button LaserFailsafe;
        [Export]
        public VSlider DrillDrive;
        [Export]
        public VSlider DrillArm;

        public override void _Ready()
        {
            base._Ready();

            for (int i = 0; i < PumpAmounts.Length - 1; i++)
                PumpRuns[i].Pressed += (() => PumpQueue.Enqueue((i + 1, (float)PumpAmounts[i].Value)));
            for (int i = 0; i < FanDurations.Length - 1; i++)
                FanRuns[i].Pressed += (() => FanQueue.Enqueue((i + 1, (int)FanDurations[i].Value)));
            for (int i = 0; i < ServoRun.Length - 1; i++)
                ServoRun[i].Pressed += (() => ServoQueue.Enqueue((i + 1, ServoRun[i].ButtonPressed)));
        }

        public override void _Process(double delta)
        {
            base._Process(delta);
        }

        public override void AdvertiseToROS()
        {
            ROS.RequestTopic<BioControl>(ControlTopicName);
        }

        public override void EmitToROS()
        {
            (int, float) PumpToRun = (0, 0f);
            (int, int) FanToRun = (0, 0);
            (int, bool) ServoToRun = (0, false);

            if (PumpQueue.Count > 0)
                PumpToRun = PumpQueue.Dequeue();
            if (PumpQueue.Count > 0)
                FanToRun = FanQueue.Dequeue();
            if (PumpQueue.Count > 0)
                ServoToRun = ServoQueue.Dequeue();

            DriveElectronics(PumpToRun.Item1, PumpToRun.Item2, FanToRun.Item1, FanToRun.Item2, ServoToRun.Item1, ServoToRun.Item2);

            msg.bio_arm = Mathf.RoundToInt(BioArm.Value);
            msg.vibration_motor = VibrationMotor.ButtonPressed ? 1 : 0;

            msg.laser = Laser.ButtonPressed && LaserFailsafe.ButtonPressed ? 1 : 0;
            msg.drill = Mathf.RoundToInt(DrillDrive.Value);
            msg.drill_arm = Mathf.RoundToInt(DrillArm.Value);

            ROS.Publish(ControlTopicName, msg);
        }

        public override void _ExitTree()
        {
            msg.laser = 0;
            msg.drill = 0;
            msg.drill_arm = 0;
            msg.bio_arm = 0;
            msg.vibration_motor = 0;

            DriveElectronics(1, 0, 1, 0, 1, false);

            ROS.Publish(ControlTopicName, msg);

            DriveElectronics(2, 0, 2, 0, 2, false);

            ROS.Publish(ControlTopicName, msg);

            DriveElectronics(3, 0, 3, 0, 3, false);

            ROS.Publish(ControlTopicName, msg);
        }

        private void DriveElectronics(int pumpid, float pump, int fanid, int fan, int servoid, bool servo)
        {
            msg.pump_id = pumpid;
            msg.pump_amount = pump;

            msg.fan_id = fanid;
            msg.fan_duration = fan;

            msg.servo_id = servoid;
            msg.servo_state = servo;
        }
    }
}
