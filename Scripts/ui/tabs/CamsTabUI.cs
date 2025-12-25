using IPC;
using Godot;
using RosSharp.RosBridgeClient.MessageTypes.Astra;

namespace ui
{
    public partial class CamsTabUI : BaseTabUI
    {
        public override void AdvertiseToROS() { }
        public override void EmitToROS() { }
        public override void _ExitTree() { }
    }
}
