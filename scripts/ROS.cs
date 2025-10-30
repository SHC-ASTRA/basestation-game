using DEBUG;
using System.IO;
using static Godot.OS;
using System.Threading;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Protocols;

namespace IPC
{
    public partial class ROS : Godot.Node
    {
        public static ROS ROSServer;

        private static Godot.FileAccess RosBridgeIO;
        private static int RosBridgeIOID;

        private static Godot.FileAccess RosBridgeErr;
        private static int RosBridgeErrID;

        public static int RosBridgePID;

        public static RosSocket ROSSocket;
        public static WebSocketNetProtocol ROSWebSocket;

        public override void _Ready()
        {
            Godot.Collections.Dictionary RosBridgeReturn = ExecuteWithPipe("bash", ["-c", "cd ROS/Docker && docker compose up"]);

            RosBridgeIO = RosBridgeReturn["stdio"].As<Godot.FileAccess>(); // Setup IO file accesses
            RosBridgeErr = RosBridgeReturn["stderr"].As<Godot.FileAccess>();
            RosBridgePID = RosBridgeReturn["pid"].As<int>();

            RosBridgeIOID = Debug.RegisterDebugData(RosBridgeIO);
            RosBridgeErrID = Debug.RegisterDebugData(RosBridgeErr);

            RosBridgeIO.GetLine(); // Ingest PWD
            RosBridgeIO.GetLine(); // Ingest startup response

            // Ts literally just crashes if it doesn't work, even if it's in a try statement.
            // So... Cope?
            string cfgpath = Path.Combine(Path.GetFullPath("."), "WebsocketIP.cfg");
            RosBridgeIO.StoreLine(string.Concat("Loading IP config from ", Path.Combine(Path.GetFullPath("."), "WebsocketIP.cfg")));
            string ip;
            if (File.Exists(cfgpath))
                ip = File.ReadAllText(cfgpath);
            else
            {
                RosBridgeIO.StoreLine(string.Concat("File ", cfgpath, " does not exist, writing defaults."));
                File.WriteAllText(cfgpath, "ws://192.168.1.33:9090");
                ip = "ws://192.168.1.33:9090";
            }
            ROSWebSocket = new WebSocketNetProtocol(File.ReadAllText(cfgpath));
            ROSSocket = new(ROSWebSocket);
            ROSSocket.Advertise<RosSharp.RosBridgeClient.MessageTypes.Std.String>("/test");
            new Thread(pingTimer.cycle).Start();
            base._Ready();
        }

        public static class pingTimer
        {
            public static void cycle()
            {
                while (true)
                {
                    ROS.ROSSocket.Publish("/test", new RosSharp.RosBridgeClient.MessageTypes.Std.String("Test ping"));
                    Thread.Sleep(1);
                }
            }
        }

        public static void Publish(string topic, Message messageType)
        {
            ROS.ROSSocket.Publish(topic, messageType);
        }
    }
}
