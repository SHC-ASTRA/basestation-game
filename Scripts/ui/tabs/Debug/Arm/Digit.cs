using Godot;
using RosSharp.RosBridgeClient.MessageTypes.Astra;

namespace UI.Debug
{
    public partial class Digit : Debug<DigitFeedback>
    {
        [Export]
        public Label WristAngle;
        public override void FeedbackHandler()
        {
            if (!Visible)
                return;
            WristAngle.Text = $"Angle: {feedback.wrist_angle.ToString()}°".PadRight("Angle: ".Length + 4);
        }
    }
}
