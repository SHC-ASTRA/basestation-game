using RosSharp.Urdf;
using System.Threading;
using System.Threading.Tasks;
using System.Collections.Generic;
using RosSharp.RosBridgeClient.UrdfTransfer;

namespace IPC.URDF
{
    public class URDFDownloadParameters
    {
        public string RobotName;
        public System.Action<Robot> Callback;

        public URDFDownloadParameters(string RobotName, System.Action<Robot> Callback)
        {
            this.RobotName = RobotName;
            this.Callback = Callback;
        }
    }
    public class URDFDownloader
    {
        public const string URDFParamName = "robot_state_publisher:robot_description";

        public static void DownloadURDF(URDFDownloadParameters Parameters)
        {
            new Thread(new Downloader(ref Parameters).main).Start();
        }

        private class Downloader
        {
            public URDFDownloadParameters UDP;

            public Downloader(ref URDFDownloadParameters UDP) => this.UDP = UDP;

            public void main()
            {
                System.IO.Directory.CreateDirectory("/tmp/URDF");
                UrdfTransferFromRos urdfTransfer = new(ROS.ROSSocket, "/tmp/URDF/", URDFParamName, "core_rover");
                urdfTransfer.Transfer();
                if (!Task.Run<bool>(() => (urdfTransfer.Status["robotDescriptionReceived"].WaitOne(5000))).Result)
                {
                    Godot.GD.PushError("Could not download requested URDF!");
                    UDP.Callback.Invoke(null);
                }
                Godot.GD.Print($"{UDP.RobotName} Description received... ");
                UDP.Callback.Invoke(new Robot($"/tmp/URDF/{UDP.RobotName}"));
            }
        }
    }
}

public static class URDFTranslator
{
    public static string[] JointNames(Robot R)
    {
        List<string> jointnames = new();
        R.joints.ForEach(x => jointnames.Add(x.name));
        return jointnames.ToArray();
    }
}
