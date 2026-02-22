using UI;
using Godot;
using System.Linq;
using System.Threading;
using System.Net.Sockets;
using System.Threading.Tasks;
using RosSharp.RosBridgeClient;
using System.Collections.Generic;
using RosSharp.RosBridgeClient.Protocols;
using RosSharp.RosBridgeClient.MessageTypes.Action;

namespace IPC
{
    public partial class ROS : Godot.Node
    {
        public static bool ROSDEBUG;
        public static bool ROSReady = false;
        public static bool Readvertise = false;
        private static ROSAlerter alerter;

        private static ROSBridgeThread ROSThread;
        public static RosSocket ROSSocket;

        private readonly static HashSet<string> interfaceNames = new HashSet<string>();

        private static readonly string ROSIP = new System.Net.IPAddress(new byte[] { 127, 0, 0, 1 }).ToString();
        private const int ROSPort = 9090;

        private static bool run = true;

        public override void _Ready()
        {
            alerter = GetChild(0) as ROSAlerter;
            StartROS();
            base._Ready();
        }

        public async static void StartROS()
        {
            alerter.Disconnected();
            ROSReady = false;

            // Check if there's a connected ROSSocket and kill it
            if (await CheckROSOnce(ROSIP, ROSPort))
            {
                if (ROSSocket != null)
                {
                    ROSSocket.Close();
                    ROSSocket = null;
                }
                OS.DelayMsec(10);
            }

            // Check and kill any dangling ROSThread if it exists
            if (ROSThread != null)
            {
                run = false;
                ROSThread.Kill();
                ROSThread.WaitToFinish();
                ROSThread.Dispose();
            }
            run = true;


            // Spawn ROSBridgeThread instance
            ROSThread = new ROSBridgeThread();
            //  Spool the thread
            ROSThread.Start(new Callable(ROSThread, "main"));

            // Waits for ROSBridge to come up. Necessary as if we don't we might start sending/requesting data before it's ready
            if (!await WaitForRosbridgeAsync(ROSIP, ROSPort, 40, 400))
            {
                GD.PrintErr("Could not initialize Rosbridge");
                return;
            }
            GD.Print("Initialized Rosbridge");

            ROSSocket = new(new WebSocketNetProtocol($"ws://{ROSIP}:{ROSPort}"));

            if (Readvertise)
            {
                Readvertise = false;
                foreach (BaseTabUI tabs in TabController.StaticTabsParent.GetChildren().Where(static _ => _ is BaseTabUI).Cast<BaseTabUI>())
                    tabs.AdvertiseToROS();
            }
            GD.Print("\nROS Ready\n");
            ROSReady = true;
        }

        public override void _Notification(int notif)
        {
            if (notif == SceneTree.NotificationPredelete)
                _ExitTree();
        }

        // Cleans up the ROS socket so that no other ROS clients think
        // our topics are alive, and then kills the socket, then kills ROSBridge
        public override void _ExitTree()
        {
            ROSSocket.Close();
            ROSThread.Kill();
            ROSThread.WaitToFinish();
            ROSThread.Dispose();
            base._ExitTree();
        }

        public static bool interfaceExists(string topicName) => interfaceNames.Contains(topicName);

        // Takes in Message Topic type, and the Topic Name, advertises it to ROS
        public static void AdvertiseMessage<T>(string topicName) where T : Message
        {
            GD.Print($"Advertising topic {topicName}");
            ROSSocket.Advertise<T>(topicName);
            interfaceNames.Add(topicName);
        }

        /// <summary> Takes in the Service topic name and a handler, which is the <typeparamref name="A"/> Request and out ServiceCallHandler(<typeparamref name="A"/>, <typeparamref name="B"/>) response </summary>
        public static void AdvertiseService<A, B>(string serviceName, ServiceCallHandler<A, B> handler) where A : Message where B : Message
        {
            GD.Print($"Advertising service {serviceName}");
            ROSSocket.AdvertiseService<A, B>(serviceName, handler);
            interfaceNames.Add(serviceName);
        }

        /// <summary>
        /// Can I even be forgiven for this?
        /// <para> So, I'm not going to help you with everything. But here's a gist: </para>
        /// <para><paramref name="actionName"/>: Name of this action client</para>
        /// <para><paramref name="act"/>: Main, "parent" Action type</para>
        /// <para><paramref name="actionGoalHandler"/>: A lambda function, called after client receives and internally processes data</para>
        /// <para><paramref name="actionCancelHandler"/>: A lambda function, called after client receives and internall processes its cancellation</para>
        /// <para><paramref name="goalStatus"/>: Std_msgs/Action/GoalStatus message, information handled internally by ROSBridge</para>
        /// <para><paramref name="feedbackCallback"/>: Called when client received action feedback</para>
        /// <para><paramref name="resultCallback"/>: Called when client received an action result</para>
        /// <para><paramref name="statusCallback"/>: Called when client goal status changes</para>
        /// Types should be as follows:
        /// <para> <typeparamref name="A"/>: TypeAction </para> <para> <typeparamref name="B"/>: TypeActionGoal </para>
        /// <para> <typeparamref name="C"/>: TypeActionResult </para> <para> <typeparamref name="D"/>: TypeActionFeedback </para>
        /// <para> <typeparamref name="E"/>: TypeGoal </para> <para> <typeparamref name="F"/>: TypeResult </para>
        /// <para> <typeparamref name="G"/>: TypeFeedback </para>
        ///  More comments than code ❤
        /// </summary>
        public static ROSActionClient<A, B, C, D, E, F, G> AdvertiseAction<A, B, C, D, E, F, G>(
            string actionName, A act, SendActionGoalHandler<B> actionGoalHandler,
            CancelActionGoalHandler actionCancelHandler, GoalStatus goalStatus = null, System.Action
            feedbackCallback = null, System.Action resultCallback = null, System.Action statusCallback = null)
        where A : Action<B, C, D, E, F, G>
        where B : ActionGoal<E> where C : ActionResult<F> where D : ActionFeedback<G>
        where E : Message where F : Message where G : Message
        {
            GD.Print($"Advertising action {actionName}");
            return new(actionName, act, actionGoalHandler, actionCancelHandler, goalStatus, feedbackCallback, resultCallback, statusCallback);
        }

        /// <summary> Uses a globally dynamic delay so that topics
        /// cannot all advertise at once, which causes ROSBridge to fail </summary>
        public async static Task AwaitRosReady()
        {
            while (!ROSReady) { await Task.Delay(1); continue; }
            while (requestDelay < 15)
            {
                Thread.Sleep(requestDelay++ * 2);
            }
            requestDelay = 1;
        }
        static int requestDelay = 1;

        /// <summary> Takes in the target Topic name and message instance,
        /// publishes the latter to the former </summary>
        public static void Publish(string topicName, Message message)
        {
            if (!interfaceExists(topicName))
                return;
            ROSSocket.Publish(topicName, message);
        }

        /// <summary> Publishes the given <paramref name="args"/> to
        /// the target <paramref name="serviceName"/>, and returns the
        /// response as a ServiceResponseHandler(<typeparamref name="P"/>), where P is the
        /// service's response type </summary>
        public static void PublishServiceGoal<T, P>(
            string serviceName,
            ServiceResponseHandler<P> response,
            T args
        ) where T : Message where P : Message
        {
            if (!interfaceExists(serviceName))
                return;
            ROSSocket.CallService<T, P>(serviceName, response, args);
        }

        public static void TopicSubscribe<T>(string topicName, SubscriptionHandler<T> Callback) where T : Message
        => ROSSocket.Subscribe<T>(topicName, Callback);

        /// <summary> Goated Chat code, continuously attempts to
        /// connect to ROSBridge, waiting more and more as it fails.
        /// Will give up after a decent amount of time </summary>
        private static async Task<bool> WaitForRosbridgeAsync(string host, int port, int timeoutSec = 400, int pollIntervalMs = 2000, int increaseMs = 100)
        {
            int failCount = 0;
            int totalFailCount = 0;
            while (true)
            {
                if (await CheckROSOnce(host, port, timeoutSec, pollIntervalMs))
                    return true;
                GD.Print($"Failed to connect to ROSBridge container. Retrying in {pollIntervalMs}Ms");
                totalFailCount++;
                if (++failCount > 4)
                {
                    GD.Print($"Recorded five or more failures to connect to ROSBridge. Increasing retry time by {increaseMs * 0.001f}s");
                    pollIntervalMs += increaseMs;
                    failCount = 0;
                }
                if (totalFailCount + 1 == 40) return false;
                await Task.Delay(pollIntervalMs);
            }
        }

        private static async Task<bool> CheckROSOnce(string host, int port, int timeoutSec = 400, int pollIntervalMs = 2000)
        {
            try
            {
                TcpClient client = new();
                System.IAsyncResult result = client.BeginConnect(host, port, null, null);
                if (result.AsyncWaitHandle.WaitOne(pollIntervalMs) && client.Connected)
                {
                    client.EndConnect(result);
                    alerter.Connected();
                    await Task.Delay(1);
                    client.Dispose();
                    return true;
                }
                else client.Dispose();
            }
            catch { }
            return false;
        }

        private partial class ROSBridgeThread : GodotThread
        {
            public bool main()
            {
                while (run)
                {
                    // Execute only gives us the PID as an output
                    OS.Execute("ros2",
                        ["run", "rosbridge_server", "rosbridge_websocket", "--ros-args", "--params-file", "./ROS/rosbridge_conf.yaml"]
                    );
                    if (run)
                        return false;
                }
                if (ROSSocket != null)
                    ROSSocket.Close();
                // If it crashed, make. sure. it's. dead.
                OS.Execute("pkill", ["rosbridge_webso"]);
                return true;
            }

            public void Kill()
            {
                if (ROSSocket != null)
                    ROSSocket.Close();
                run = false;
                // Make sure the main thread halts until rosbridge_websocket is dead.
                OS.Execute("pkill", ["rosbridge_webso"]);
            }
        }
    }
}
