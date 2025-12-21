using DEBUG;
using Godot;
using System;
using System.IO;
using static Godot.OS;
using System.Threading;
using System.Net.Sockets;
using System.Threading.Tasks;
using RosSharp.RosBridgeClient;
using System.Collections.Generic;
using RosSharp.RosBridgeClient.Protocols;

namespace IPC
{
    public partial class ROS : Godot.Node
    {
        public static bool ROSReady = false;
        public static ROS ROSServer;

        private static Godot.FileAccess RosBridgeIO, RosBridgeErr;
        private static int RosBridgePID, RosBridgeIOID, RosBridgeErrID;

        public static RosSocket ROSSocket;

        private readonly static HashSet<string> topicNames = new HashSet<string>();

        private static readonly string ROSIP = new System.Net.IPAddress(new byte[] { 127, 0, 0, 1 }).ToString();
        private const int ROSPort = 9090;
        private static readonly string sockAddress = $"ws://{ROSIP}:{ROSPort}";

        public static string term = System.OperatingSystem.IsLinux() ? "bash" : "pwsh";

        private static string ROSDocker = Path.Combine("ROS", "Docker");

        public override void _Ready()
        {
            StartROS();
            base._Ready();
        }

        public async static void StartROS()
        {
            // pipes docker ps into grep to see if the container is running; if it is, cd's to the ROS/Docker dir and kills the container
            Execute(term, ["-c", $"if docker ps | grep docker-ros2-1; then cd {ROSDocker} && docker compose down; fi"]);
            // ExecuteWithPipe creates three different IOStreams bundled in a dictionary
            Godot.Collections.Dictionary RosBridgeReturn = ExecuteWithPipe(term, ["-c", $"cd {ROSDocker} && docker compose up"]);
            // Waits for ROSBridge to come up. Necessary as if we don't we might start sending/requesting data before it's ready
            if (!await WaitForRosbridgeAsync(ROSIP, ROSPort, 40, 5000))
            {
                GD.PrintErr("Could not initialize Rosbridge");
                return;
            }
            GD.Print("Initialized Rosbridge");

            RosBridgeIO = RosBridgeReturn["stdio"].As<Godot.FileAccess>();
            RosBridgeErr = RosBridgeReturn["stderr"].As<Godot.FileAccess>();
            RosBridgePID = RosBridgeReturn["pid"].As<int>();

            RosBridgeIOID = Debug.RegisterDebugData(RosBridgeIO, true);
            RosBridgeErrID = Debug.RegisterDebugData(RosBridgeErr, true);

            string cfgpath = Path.Combine(Path.GetFullPath("."), "WebsocketIP.cfg");
            RosBridgeIO.StoreLine(string.Concat("Loading IP config from ", Path.Combine(Path.GetFullPath("."), "WebsocketIP.cfg")));
            Debug.Log(RosBridgeIOID, $"Loading IP config from {cfgpath}");

            string ip;
            if (File.Exists(cfgpath))
                ip = File.ReadAllText(cfgpath);
            else
            {
                Debug.Log(RosBridgeIOID, $"File {cfgpath} does not exist, writing defaults.");
                ip = $"ws://{ROSIP}:{ROSPort}";
                File.WriteAllText(cfgpath, ip);
            }
            ROSSocket = new(new WebSocketNetProtocol(sockAddress));
            ROSReady = true;
        }

        public override void _Notification(int notif)
        {
            if (notif == SceneTree.NotificationPredelete)
                _ExitTree();
        }

        public override void _ExitTree()
        {
            foreach (string s in topicNames)
                ROSSocket.Unadvertise(s);
            Execute(term, ["-c", $"if docker ps | grep docker-ros2-1; then cd {ROSDocker} && docker compose down; fi"]);
            base._ExitTree();
        }

        public static bool CheckTopicExists(string topicName) => topicNames.Contains(topicName);

        public static void RequestTopic<T>(string topicName) where T : Message
        {
            Debug.Log(RosBridgeIOID, $"Queuing {topicName} for advertisement");
            Task.Run(() => { while (!ROSReady) continue; Advertise<T>(topicName); });
        }

        public static void Advertise<T>(string topicName) where T : Message
        {
            Debug.Log(RosBridgeIOID, $"Advertising {topicName}");
            ROSSocket.Advertise<T>(topicName);
            topicNames.Add(topicName);
        }

        public static void Publish(string topic, Message message)
        {
            if (!CheckTopicExists(topic))
                return;
            Debug.Log(RosBridgeIOID, $"Writing {message.ToString()} to {topic}");
            ROSSocket.Publish(topic, message);
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
                        IAsyncResult result = client.BeginConnect(host, port, null, null);
                        bool success = result.AsyncWaitHandle.WaitOne(pollIntervalMs);
                        if (success && client.Connected)
                        {
                            client.EndConnect(result);
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
