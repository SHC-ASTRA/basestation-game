using Godot;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Astra;

namespace UI.Debug
{
    public partial class Digit : Debug<DigitFeedback>
    {
        [Export]
        public Label WristAngle;
        public override SubscriptionHandler<DigitFeedback> GetFeedbackHandler() => new((feedback) =>
        {
            if (!Visible)
                return;
            WristAngle.Text = $"Angle: {feedback.wrist_angle.ToString()}°".PadRight("Angle: ".Length + 4);
        });
    }
}
