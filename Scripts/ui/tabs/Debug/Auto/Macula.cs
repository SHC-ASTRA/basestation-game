using Godot;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Astra;

namespace UI.Debug
{
    public partial class Macula : Debug<MaculaFeedback>
    {
        [Export]
        TextureRect DetectedTrue;
        bool prevDetected;

        [Export]
        Label ObjectID;

        [Export]
        Label Corner0, Corner1, Corner2, Corner3;

        public override SubscriptionHandler<MaculaFeedback> GetFeedbackHandler() => new((feedback) =>
        {
            if (!Visible)
                return;

            if (prevDetected != feedback.detected)
            {
                DetectedTrue.Visible = prevDetected = feedback.detected;
                if (prevDetected)
                {
                    ObjectID.Text = $"Object ID: {feedback.object_id}";
                    Corner0.Text = $"Corner 0: {string.Join(',', feedback.x0, feedback.y0)}";
                    Corner1.Text = $"Corner 1: {string.Join(',', feedback.x1, feedback.y1)}";
                    Corner2.Text = $"Corner 2: {string.Join(',', feedback.x2, feedback.y2)}";
                    Corner3.Text = $"Corner 3: {string.Join(',', feedback.x3, feedback.y3)}";
                }
                else
                {
                    ObjectID.Text = "No object!";
                    Corner0.Text = "";
                    Corner1.Text = "";
                    Corner2.Text = "";
                    Corner3.Text = "";
                }
            }
        });
    }
}
