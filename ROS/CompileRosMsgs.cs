using Godot;
using RosSharp.RosBridgeClient.MessageGeneration;

namespace IPC
{
    [Tool]
    public partial class CompileRosMsgs : EditorScript
    {
        public override void _Run()
        {
            MessageAutoGen.GenerateDirectoryMessages("../astra_msgs/msg", "./.tmp/");
            base._Run();
        }
    }
}
