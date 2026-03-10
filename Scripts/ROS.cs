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
        // Control signals
        // Has ROS finished booting?
        public static bool ROSReady = false;
        // Is ROS coming down intentional? If not - crashed
        public static bool run = true;
        // Was Close called since the last time StartROS was?
        private static bool closed = false;

        private static ROSAlerter alerter;

        public static RosSocket ROSSocket;
        private static ROSBridgeThread ROSThread;

        private readonly static HashSet<string> topicNames = new(), actionNames = new(), serviceNames = new();

        // The port has to be a string because casting const ints to strings isn't compile-time constant
        // and it's prettier this way.
        private const string ROSIP = "127.0.0.1", ROSPort = "9090", ROSWS = $"ws://{ROSIP}:{ROSPort}";

        public override void _Ready()
        {
            alerter = GetChild(0) as ROSAlerter;
            StartROS();
            base._Ready();
        }

        public async static void StartROS()
        {
            ROSReady = false;
            closed = false;

            // Spawn ROSBridgeThread instance
            ROSThread = new ROSBridgeThread();
            //  Spool the thread
            ROSThread.Start(new Callable(ROSThread, "main"));

            // Waits for ROSBridge to come up. Necessary as if we don't we might start sending/requesting data before it's ready
            if (!await WaitForRosbridgeAsync(ROSIP, int.Parse(ROSPort), 40, 400))
            {
                GD.PrintErr("Could not initialize Rosbridge");
                return;
            }
            GD.Print("Initialized Rosbridge");

            ROSSocket = new(new WebSocketNetProtocol(ROSWS));

            // Tell all tabs to readvertise
            foreach (BaseTabUI tabs in TabController.StaticTabsParent.GetChildren().Cast<BaseTabUI>())
                tabs.AdvertiseToROS();

            GD.Print("\nROS Ready\n");
            ROSReady = true;
            run = true;
        }

        /// <summary> Cleans up the ROS socket so that no other ROS clients think
        /// our topics are alive, and then kills the socket, then kills ROSBridge </summary>
        public static void Close()
        {
            // If the thread is already closed, calling Close would cause a lot of null access errors.
            if (closed)
                return;
            closed = true;

            alerter.Disconnected();

            // This will take time so we can kill it and then clean up ROSSocket while it works
            ROSThread.Kill();
            // ROSSocket may already be closed / crashed from an error, we should check first
            if (ROSSocket != null)
            {
                // Log all the interfaces coming down
                foreach (string topicName in topicNames)
                    GD.Print($"Unadvertising topic {topicName}");
                topicNames.Clear();
                foreach (string serviceName in serviceNames)
                    GD.Print($"Unadvertising service {serviceName}");
                serviceNames.Clear();
                foreach (string actionName in actionNames)
                    GD.Print($"Unadvertising action {actionName}");
                actionNames.Clear();

                // Halt EmitToROS threads
                run = false;
                // Wipe ROSSocket from the face of the earth
                ROSSocket.Close();
                ROSSocket = null;
            }
            ROSThread.WaitToFinish();
            ROSThread.Dispose();
        }

        /// <summary> Godot-managed thread. Holds open the ROSBridge server in a blocking way.
        /// Can't auto-restart this unfortunately, get some spooky deadlock if attempted.
        /// </summary>
        private partial class ROSBridgeThread : GodotThread
        {
            public bool main()
            {
                OS.Execute("ros2",
                    ["run", "rosbridge_server", "rosbridge_websocket", "--ros-args", "--params-file", "./ROS/rosbridge_conf.yaml"]
                );

                // Alert the user via logs and on-screen if it does crash
                if (run)
                    GD.Print("Rosbridge crashed!");
                alerter.CallDeferred(nameof(alerter.Disconnected));

                return true;
            }

            public void Kill()
            {
                run = false;
                // Make sure the main thread halts until rosbridge_websocket is dead.
                OS.Execute("pkill", ["rosbridge_webso"]);
            }
        }

        public override void _ExitTree()
        {
            Close();
            base._ExitTree();
        }

        // _ExitTree() is not called if the editor stop button is clicked
        public override void _Notification(int notif)
        {
            if (notif == SceneTree.NotificationPredelete)
                Close();
        }

        // --------------------------------------
        // Networking
        // --------------------------------------

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

        // --------------------------------------
        // ROS# interaction helpers
        // --------------------------------------

        public static bool topicExists(string topicName) => topicNames.Contains(topicName);
        public static bool serviceExists(string serviceName) => serviceNames.Contains(serviceName);
        public static bool actionExists(string actionName) => actionNames.Contains(actionName);

        /// <summary> Takes Message Topic type <typeparamref name="T"/> and the corresponding <paramref name="topicName"/> and then advertises it to ROS </summary>
        public static void AdvertiseTopic<T>(string topicName, QOS q = null) where T : Message
        {
            GD.Print($"Advertising topic {topicName}");
            ROSSocket.Advertise<T>(topicName, q);
            topicNames.Add(topicName);
        }

        public static void TopicSubscribe<T>(string topicName, SubscriptionHandler<T> Callback) where T : Message
        => ROSSocket.Subscribe<T>(topicName, Callback);

        /// <summary> Takes in the Service topic name and a handler, which is the <typeparamref name="A"/> Request and
        /// out ServiceCallHandler(<typeparamref name="A"/>, <typeparamref name="B"/>) response.
        /// </summary> <remarks>
        /// <paramref name="handler"/> Should have the reponse variable assigned to a new instance of <typeparamref name="B"/>
        /// on each callback </remarks>
        public static void AdvertiseService<A, B>(string serviceName, ServiceCallHandler<A, B> handler) where A : Message where B : Message
        {
            GD.Print($"Advertising service {serviceName}");
            ROSSocket.AdvertiseService<A, B>(serviceName, handler);
            serviceNames.Add(serviceName);
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
            ROSActionClient<A, B, C, D, E, F, G> _ = new(
                actionName, act,
                actionGoalHandler, actionCancelHandler,
                goalStatus,
                feedbackCallback, resultCallback, statusCallback);
            actionNames.Add(actionName);
            return _;
        }

        /// <summary> Uses a globally dynamic delay so that topics
        /// cannot all advertise at once, which causes ROSBridge to fail </summary>
        public async static Task AwaitRosReady()
        {
            while (!ROSReady) { await Task.Delay(2); continue; }
            while (requestDelay < 15)
            {
                Thread.Sleep(requestDelay++ * 2);
            }
            requestDelay = 1;
        }
        static int requestDelay = 1;

        /// <summary> Takes in the target Topic name and message instance,
        /// publishes the latter to the former. If the topic doesn't exist,
        /// advertises it first and then publishes the message. </summary>
        public static void Publish<T>(string topicName, T message) where T : Message
        {
            if (!topicExists(topicName))
                ROSSocket.Advertise<T>(topicName);
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
            if (!serviceExists(serviceName))
                return;
            ROSSocket.CallService<T, P>(serviceName, response, args);
        }
    }
}
