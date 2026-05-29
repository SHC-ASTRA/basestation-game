using IPC;
using Godot;
using RosSharp.RosBridgeClient.MessageTypes.Sensor;

namespace UI.Debug
{
    public partial class Battery : Control
    {
        [Export]
        public ColoredLabel BatVoltage;

        public override void _Ready()
        {
            ROS.TopicSubscribe<BatteryState>(FEEDBACK.BATTERY, (v) =>
                BatVoltage.SetValue("[VALUE1]v | [VALUE2]%", v.voltage, v.percentage));
        }
    }
}
