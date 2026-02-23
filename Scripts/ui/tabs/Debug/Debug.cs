using IPC;
using Godot;
using RosSharp.RosBridgeClient;
using System.Threading.Tasks;

namespace UI.Debug
{
    public abstract partial class Debug<T> : Control where T : Message
    {
        [Export]
        public string TopicName;

        public override void _Ready()
        {
            base._Ready();
            SubscriptionHandler<T> s = GetFeedbackHandler();
            Task.Run(async () =>
            {
                while (!ROS.ROSReady)
                    await Task.Delay(5);
                ROS.TopicSubscribe<T>(TopicName, s);
            });
        }

        public abstract SubscriptionHandler<T> GetFeedbackHandler();
    }
}
