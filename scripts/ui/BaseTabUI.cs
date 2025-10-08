using Godot;
namespace ui
{
    public partial class BaseTabUI : Control
    {
        [Export]
        public Vector2 leftStick, rightStick;

        // Left bumper slows down driving, right bumper speeds it up
        // 'A' button (south) enables brake mode
        // Joysticks have deadzones (0.05) so the motors don't sound angry when it's sitting still
        // Controller rumbles when headless is loaded and ready for control
        // Default driving speed should be closer to walking speed now, pending testing
        // Turning inverts when going backwards to make reversing more intuitive (kinda jank rn tho)
        // Turning follows curve (angular.z ** 3) to help with minor adjustments when driving in straight line

        public override void _Process(double delta)
        {
            leftStick = Input.GetVector("LStickHN", "LStickHP", "LStickVN", "LStickVP");
            rightStick = Input.GetVector("RStickHN", "RStickHP", "RStickVN", "RStickVP");

        }
    }
}
