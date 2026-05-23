using Godot;
using static UI.Debug.Debug;

namespace UI.Debug.Auto
{
    public partial class Macula : FeedbackProvider<MaculaFeedback>
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

            if (prevDetected != feedback.detected)
            {
                DetectedTrue.SetDeferred(CanvasItem.MethodName.IsVisible, prevDetected = feedback.detected);
                if (prevDetected)
                {
                    SetLabelText(ObjectID, $"Object ID: {feedback.object_id}");
                    SetLabelText(Corner0, $"Corner 0: {string.Join(',', feedback.x0, feedback.y0)}");
                    SetLabelText(Corner1, $"Corner 1: {string.Join(',', feedback.x1, feedback.y1)}");
                    SetLabelText(Corner2, $"Corner 2: {string.Join(',', feedback.x2, feedback.y2)}");
                    SetLabelText(Corner3, $"Corner 3: {string.Join(',', feedback.x3, feedback.y3)}");
                }
                else
                {
                    SetLabelText(ObjectID, "No object!");
                    SetLabelText(Corner0, "");
                    SetLabelText(Corner1, "");
                    SetLabelText(Corner2, "");
                    SetLabelText(Corner3, "");
                }
            }
        }
    }
}
