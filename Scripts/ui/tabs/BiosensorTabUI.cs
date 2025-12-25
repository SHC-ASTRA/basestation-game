using IPC;
using Godot;
using System.Collections.Generic;
using RosSharp.RosBridgeClient.MessageTypes.Astra;

namespace ui
{
    public partial class BiosensorTabUI : BaseTabUI
    {
        private BioControl msg;

        [ExportCategory("Biosensor")]
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
        public Button[] ServoStates;
        [Export]
        public Button[] ServoRun;
        private Queue<(int, bool)> ServoQueue = new();

        public override void _Ready()
        {
            msg = new();

            for (int i = 0; i < PumpAmounts.Length; i++)
                PumpRuns[i].Pressed += (() => PumpQueue.Enqueue((i + 1, (float)PumpAmounts[i].Value)));
            for (int i = 0; i < FanDurations.Length; i++)
                FanRuns[i].Pressed += (() => FanQueue.Enqueue((i + 1, (int)FanDurations[i].Value)));
            for (int i = 0; i < ServoStates.Length; i++)
                ServoRun[i].Pressed += (() => ServoQueue.Enqueue((i + 1, ServoStates[i].ButtonPressed)));
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

            msg.pump_id = PumpToRun.Item1;
            msg.pump_amount = PumpToRun.Item2;

            msg.fan_id = FanToRun.Item1;
            msg.fan_duration = FanToRun.Item2;

            msg.servo_id = ServoToRun.Item1;
            msg.servo_state = ServoToRun.Item2;


        }

        public override void _ExitTree()
        {
        }
    }
}
