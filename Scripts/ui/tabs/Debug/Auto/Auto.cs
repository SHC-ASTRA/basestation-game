using Godot;
using static UI.Debug.Debug;

namespace UI.Debug
{
    public partial class Auto : FeedbackProvider<AutoFeedback>
    {
        [Export]
        TextureRect DetectedTrue;
        bool prevDetected;

        [Export]
        Label ObjectID;

        [Export]
        Label Corner0, Corner1, Corner2, Corner3;

        public override void FeedbackHandler()
        {
            if (!visible)
                return;
        }
    }
}
