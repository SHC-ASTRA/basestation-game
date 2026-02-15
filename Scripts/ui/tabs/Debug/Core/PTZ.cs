using Godot;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Astra;

namespace UI.Debug
{
    public partial class PTZ : Debug<PtzFeedback>
    {
        private const string y = "Yaw:   ", p = "Pitch: ", r = "Roll:  ", degs = "° | ", ms = "m/s";
        private const int cylength = 5 + 4 + 3 + 6, cpylength = 5 + 4 + 3 + 6, crlength = 5 + 4 + 3 + 6;

        [Export]
        TextureRect ConnectedTrue;
        bool lastConnected;

        [Export]
        public Label Yaw, Pitch, Roll;

        public override SubscriptionHandler<PtzFeedback> GetFeedbackHandler() => new((feedback) =>
        {
            if (!Visible)
                return;

            Yaw.Text = string.Join(y, feedback.yaw, degs, feedback.yaw_velocity, ms)
                .PadRight(cylength);
            Pitch.Text = string.Join(p, feedback.pitch, degs, feedback.pitch_velocity, ms)
                .PadRight(cpylength);
            Roll.Text = string.Join(r, feedback.roll, degs, feedback.roll_velocity, ms)
                .PadRight(crlength);

            if (lastConnected != feedback.connected)
            {
                lastConnected = feedback.connected;
                ConnectedTrue.Visible = feedback.connected;
            }
        });
    }
}
