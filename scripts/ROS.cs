using static Godot.OS;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Protocols;

namespace IPC
{
    public static class ROS
    {
        public static void Init()
        {
            CreateProcess("./bash/launchstreamserver.sh", []);
            Execute("./bash/sleep.sh", ["5"]);
            RosSocket rs = new(new WebSocketNetProtocol("ws://192.168.4.8:9090"));


        }
    }
}
