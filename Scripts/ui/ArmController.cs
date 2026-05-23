using Godot;
using UI.Debug.Arm;

namespace UI
{
    public partial class ArmController : Control
    {
        [Export]
        public TextureRect UpperArm, Forearm, EF;

        public override void _Process(double delta)
        {
            UpperArm.Rotation = (ArmDebug.Axis1.position % 2 - 1) * 180;
            Forearm.Rotation = (ArmDebug.Axis2.position % 2 - 1) * 180;
            EF.Rotation = (ArmDebug.Axis3.position % 2 - 1) * 180;
        }
    }
}
