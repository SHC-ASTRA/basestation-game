using static Godot.OS;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Protocols;

public static class ROS
{
    public static void Init()
    {
        Execute("", []);
        RosSocket rs = new RosSocket(new WebSocketNetProtocol("ws://192.168.4.8:6969"));


    }
}
