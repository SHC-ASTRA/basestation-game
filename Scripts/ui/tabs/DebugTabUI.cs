using DEBUG;

namespace ui
{
    public partial class DebugTabUI : BaseTabUI
    {
        void _Process()
        {
            Debug.PullLogs();
        }

        public override void AdvertiseToROS() { }
        public override void EmitToROS() { }
        public override void _ExitTree() { }
    }
}
