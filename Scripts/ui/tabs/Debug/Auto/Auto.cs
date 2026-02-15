using Godot;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Astra;

namespace UI.Debug
{
    public partial class Auto : Debug<AutoFeedback>
    {
        [Export]
        TextureRect DetectedTrue;
        bool prevDetected;

        [Export]
        Label ObjectID;

        [Export]
        Label Corner0, Corner1, Corner2, Corner3;

        public override SubscriptionHandler<AutoFeedback> GetFeedbackHandler() => new((feedback) =>
        {
            if (!Visible)
                return;


        });
    }
}
