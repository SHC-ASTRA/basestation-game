using IPC;
using Godot;
using RosSharp.RosBridgeClient.MessageTypes.Astra;

namespace ui
{
    public partial class AutonomyTabUI : BaseTabUI
    {
        public override void AdvertiseToROS()
        {
            ROS.RequestTopic<AutoNav>(ControlTopicName);
        }
        public override void EmitToROS() { }
        public override void _ExitTree() { }
    }
}
