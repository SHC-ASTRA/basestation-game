
namespace UI
{
    public partial class CamsTabUI : BaseTabUI
    {
        public override bool AdvertiseToROS() { return false; }
        public override void EmitToROS() { }
        public override void _ExitTree() { }
    }
}
