using Godot;
using System.IO;
using RosSharp.RosBridgeClient.CMessageGeneration;

namespace IPC
{
    [Tool]
    public partial class CompileRosMsgs : EditorScript
    {
        const string tmp = "./tmp";
        public override void _Run()
        {
            if (Directory.Exists(tmp))
                Directory.Delete(tmp, true); MessageAutoGen.GenerateDirectoryMessages("astra_msgs/msg", tmp, "astra_msgs", true);
            base._Run();
        }
    }
}
