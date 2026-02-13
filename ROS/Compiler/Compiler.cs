using System.IO;
using RosSharp.RosBridgeClient.CMessageGeneration;

namespace ROSCompiler;

public class Class1
{
    const string successDir = "../RosSharpInterfaces";
    const string tmp = "./tmp";

    static void Main()
    {

        Directory.CreateDirectory(tmp);

        ServiceAutoGen.GenerateDirectoryServices("./astra_msgs/srv", tmp, false);
        MessageAutoGen.GenerateDirectoryMessages("./astra_msgs/msg", tmp, "astra_msgs", false);
        ActionAutoGen.GenerateDirectoryActions("./astra_msgs/action", tmp, false);

        if (Directory.Exists(successDir))
            Directory.Delete(successDir, true);

        Directory.Move(tmp, successDir);
    }
}
