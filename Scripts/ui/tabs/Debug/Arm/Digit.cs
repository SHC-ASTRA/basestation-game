using Godot;
using static UI.Debug.Debug;

namespace UI.Debug
{
    public partial class Digit : FeedbackProvider<DigitFeedback>
    {
        [Export]
        public Label WristAngle;
        public override void FeedbackHandler()
        {
            if (!visible)
                return;
            SetLabelText(WristAngle, $"Angle: {feedback.wrist_angle}°".PadRight("Angle: ".Length + 4));
        }
    }
}
