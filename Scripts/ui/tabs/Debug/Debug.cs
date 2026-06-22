using IPC;
using Godot;
using System.Threading.Tasks;
using RosSharp.RosBridgeClient;

namespace UI.Debug
{
    public static partial class Debug
    {
        public abstract partial class FeedbackProvider<T> : Visibility where T : Message
        {
            private bool registered = false;
            public abstract string TopicName();
            public override void _Ready()
            {
                base._Ready();
                ROS.RegsiterOnROSStart(() => Task.Run(async () =>
                {
                    if (!registered)
                    {
                        CallDeferredThreadGroup(GodotObject.MethodName.Connect, [Node.SignalName.TreeExiting, Callable.From(() => ROS.TopicUnsubscribe(TopicName()))]);
                        registered = true;
                    }
                    ROS.TopicSubscribe<T>(TopicName(), (feedback) =>
                    {
                        this.feedback = feedback;
                        CallDeferred(nameof(FeedbackHandler));
                    });
                }));
            }

            internal T feedback;
            public abstract void FeedbackHandler();
        }

        public static void SetLabelText(Label t, string s) =>
            t.SetDeferred(Label.PropertyName.Text, s);
    }
}
