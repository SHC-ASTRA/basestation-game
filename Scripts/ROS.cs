using ui;
using Godot;
using System.IO;
using System.Linq;
using static Godot.OS;
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
        public static bool ROSReady = false;
        public static bool Readvertise = false;
        public static ROS ROSServer;
        private static Godot.FileAccess RosBridgeIO, RosBridgeErr;
        private static int RosBridgePID;

        public static RosSocket ROSSocket;

        private readonly static HashSet<string> interfaceNames = new HashSet<string>();

        private static readonly string ROSIP = new System.Net.IPAddress(new byte[] { 127, 0, 0, 1 }).ToString();
        private const int ROSPort = 9090;
        private static readonly string sockAddress = $"ws://{ROSIP}:{ROSPort}";

        public static string term = System.OperatingSystem.IsLinux() ? "bash" : "pwsh";

        private static string ROSDocker = "ROS/Docker";

        private static ROSAlerter alerter;

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

            // Pipes docker ps into grep to see if the container is running; if it is, cd's to the ROS/Docker dir and kills the container
            Execute(term, ["-c", $"if docker ps | grep docker-ros2-1; then cd {ROSDocker} && docker compose down; fi"]);
            // ExecuteWithPipe creates three different IOStreams bundled in a dictionary
            Godot.Collections.Dictionary RosBridgeReturn = ExecuteWithPipe(term, ["-c", $"cd {ROSDocker} && docker compose up"]);
            // Waits for ROSBridge to come up. Necessary as if we don't we might start sending/requesting data before it's ready
            if (!await WaitForRosbridgeAsync(ROSIP, ROSPort, 40, 4000))
            {
                GD.PrintErr("Could not initialize Rosbridge");
                return;
            }
            GD.Print("Initialized Rosbridge");

            RosBridgeIO = RosBridgeReturn["stdio"].As<Godot.FileAccess>();
            RosBridgeErr = RosBridgeReturn["stderr"].As<Godot.FileAccess>();
            RosBridgePID = RosBridgeReturn["pid"].As<int>();


            string cfgpath = Path.Combine(Path.GetFullPath("."), "WebsocketIP.cfg");
            RosBridgeIO.StoreLine(string.Concat("Loading IP config from ", Path.Combine(Path.GetFullPath("."), "WebsocketIP.cfg")));
            GD.Print($"Loading IP config from {cfgpath}");

            string ip;
            if (File.Exists(cfgpath))
                ip = File.ReadAllText(cfgpath);
            else
            {
                GD.Print($"File {cfgpath} does not exist, writing defaults.");
                ip = $"ws://{ROSIP}:{ROSPort}";
                File.WriteAllText(cfgpath, ip);
            }
            ROSSocket = new(new WebSocketNetProtocol(sockAddress));
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

        public override void _ExitTree()
        {
            foreach (string s in interfaceNames)
                ROSSocket.Unadvertise(s);
            ROSSocket.Close();
            Execute(term, ["-c", $"if docker ps | grep docker-ros2-1; then cd {ROSDocker} && docker compose down; fi"]);
            base._ExitTree();
        }

        public static bool interfaceExists(string topicName) => interfaceNames.Contains(topicName);

        public static void AdvertiseMessage<T>(string topicName) where T : Message
        {
            GD.Print($"Advertising topic {topicName}");
            ROSSocket.Advertise<T>(topicName);
            interfaceNames.Add(topicName);
        }

        public static void AdvertiseService<A, B>(string serviceName, ServiceCallHandler<A, B> handler) where A : Message where B : Message
        {
            GD.Print($"Advertising service {serviceName}");
            ROSSocket.AdvertiseService<A, B>(serviceName, handler);
            interfaceNames.Add(serviceName);
        }

        // Can I even be forgiven for this?
        public static ROSActionClient<A, B, C, D, E, F, G> AdvertiseAction<A, B, C, D, E, F, G>(string actionName, A act, SendActionGoalHandler<B> actionGoalHandler, CancelActionGoalHandler actionCancelHandler, GoalStatus goalStatus = null, System.Action feedbackCallback = null, System.Action resultCallback = null, System.Action statusCallback = null)
        where A : Action<B, C, D, E, F, G>
        where B : ActionGoal<E> where C : ActionResult<F> where D : ActionFeedback<G>
        where E : Message where F : Message where G : Message
        {
            GD.Print($"Advertising action {actionName}");
            return new(actionName, act, actionGoalHandler, actionCancelHandler, goalStatus, feedbackCallback, resultCallback, statusCallback);
        }

        static int requestDelay = 1;
        public static Task AwaitRosReady()
        {
            while (!ROSReady) { Thread.Sleep(1); continue; }
            while (requestDelay < 25)
            {
                Thread.Sleep(requestDelay++ * 2);
            }
            requestDelay = 1;
            return Task.CompletedTask;
        }

        public static void Publish(string topicName, Message message)
        {
            if (!interfaceExists(topicName))
                return;
            GD.Print($"Publishing to topic {topicName}");
            ROSSocket.Publish(topicName, message);
        }

        public static void PublishServiceGoal<T, P>(
            string serviceName,
            ServiceResponseHandler<P> response,
            T args
        ) where T : Message where P : Message
        {
            if (!interfaceExists(serviceName))
                return;
            GD.Print($"Publishing to service {serviceName}");
            ROSSocket.CallService<T, P>(serviceName, response, args);
        }

        public static async Task<bool> WaitForRosbridgeAsync(string host, int port, int timeoutSec = 400, int pollIntervalMs = 2000, int increaseMs = 1000)
        {
            int failCount = 0;
            while (true)
            {
                try
                {
                    using (TcpClient client = new())
                    {
                        System.IAsyncResult result = client.BeginConnect(host, port, null, null);
                        bool success = result.AsyncWaitHandle.WaitOne(pollIntervalMs);
                        if (success && client.Connected)
                        {
                            client.EndConnect(result);
                            alerter.Connected();
                            Thread.Sleep(1);
                            return true;
                        }
                    }
                }
                catch { }
                GD.Print($"Failed to connect to ROSBridge container. Retrying in {pollIntervalMs}Ms");
                if (++failCount > 4)
                {
                    GD.Print($"Recorded five or more failures to connect to ROSBridge. Increasing retry time by {increaseMs * 0.001f}s");
                    pollIntervalMs += increaseMs;
                    failCount = 0;
                }
                await Task.Delay(pollIntervalMs);
            }
        }
    }
}
