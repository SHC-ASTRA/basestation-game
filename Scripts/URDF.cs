using RosSharp.Urdf;
using System.Threading;
using System.Collections.Generic;
using RosSharp.RosBridgeClient.MessageTypes.Std;

namespace IPC.URDF
{
    public class URDFDownloader
    {
        public static readonly Dictionary<string, Robot> Robots = [];

        public static void DownloadURDF(string RobotName, out Thread t, bool debug = false)
        {
            Robots.Add(RobotName, null);
            t = new Thread(() =>
            {
                ROS.ROSSocket.Subscribe<String>("/robot_description", (o) =>
                {
                    System.IO.File.WriteAllText($"/tmp/URDF/{RobotName}", o.data);
                    if (debug)
                        Godot.GD.Print($"Received {RobotName} file!");
                    Robots[RobotName] = new Robot($"/tmp/URDF/{RobotName}");
                });
                ulong n = Godot.Time.GetTicksMsec();
                while (Robots[RobotName] == null)
                    if (Godot.Time.GetTicksMsec() > n + 500000)
                        return;
                    else continue;
            });
            t.Start();
        }
    }

    public static class URDFTranslator
    {
        public static string[] JointNames(Robot R)
        {
            string[] jointnames = new string[R.joints.Count];
            for (int i = 0; i < jointnames.Length - 1; i++)
                jointnames[i] = R.joints[i].name;
            return jointnames;
        }
    }
}
