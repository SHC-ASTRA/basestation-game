using RosSharp.RosBridgeClient;

namespace UI
{
    public partial class DroneTabUI : BaseTabUI
    {
        public override bool AdvertiseToROS() { return false; }
        public override void EmitToROS() { }
        public override void _ExitTree() { }
    }
}
