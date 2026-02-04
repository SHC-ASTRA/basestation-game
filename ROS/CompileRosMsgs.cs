using Godot;
using System.IO;
using RosSharp.RosBridgeClient.CMessageGeneration;

namespace IPC
{
    [Tool]
    public partial class CompileRosMsgs : EditorScript
    {
        const string successDir = "./ROS/RosSharpInterfaces";
        const string tmp = "./tmp";
        public override void _Run()
        {
            Directory.CreateDirectory(tmp);

            ServiceAutoGen.GenerateDirectoryServices("ROS/astra_msgs/srv", tmp, false);
            MessageAutoGen.GenerateDirectoryMessages("ROS/astra_msgs/msg", tmp, "astra_msgs", false);
            ActionAutoGen.GenerateDirectoryActions("ROS/astra_msgs/action", tmp, false);

            if (Directory.Exists(successDir))
                Directory.Delete(successDir, true);

            Directory.Move(tmp, successDir);

            base._Run();
        }
    }
}
