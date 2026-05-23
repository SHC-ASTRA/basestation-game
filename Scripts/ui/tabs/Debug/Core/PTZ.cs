using Godot;
using static UI.Debug.Debug;

namespace UI.Debug.Core
{
    public partial class PTZ : FeedbackProvider<PtzFeedback>
    {
        private const string y = "Yaw:   ", p = "Pitch: ", r = "Roll:  ", degs = "° | ", ms = "m/s";
        private const int cylength = 5 + 4 + 3 + 6, cpylength = 5 + 4 + 3 + 6, crlength = 5 + 4 + 3 + 6;

        [Export]
        TextureRect ConnectedTrue;
        bool lastConnected;

        [Export]
        public Label Yaw, Pitch, Roll;

        public override void FeedbackHandler()
        {
            if (!visible)
                return;

            SetLabelText(Yaw, string.Join(y, feedback.yaw, degs, feedback.yaw_velocity, ms)
                .PadRight(cylength));
            SetLabelText(Pitch, string.Join(p, feedback.pitch, degs, feedback.pitch_velocity, ms)
                .PadRight(cpylength));
            SetLabelText(Roll, string.Join(r, feedback.roll, degs, feedback.roll_velocity, ms)
                .PadRight(crlength));

            if (lastConnected != feedback.connected)
                ConnectedTrue.SetDeferred(CanvasItem.PropertyName.Visible, lastConnected = feedback.connected);
        }
    }
}
