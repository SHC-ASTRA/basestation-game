using System;
using System.IO;
using RosSharp.RosBridgeClient.CMessageGeneration;

string pwd = Environment.GetEnvironmentVariable("PWD");

string successDir = $"{pwd}/ROS/RosSharpInterfaces";
string tmp = $"{pwd}/ROS/Compiler/tmp";

if (Directory.Exists(successDir))
    Directory.Delete(successDir, true);

if (Directory.Exists(tmp))
    Directory.Delete(tmp, true);

Directory.CreateDirectory(tmp);

(string, string)[] ToCompile = [("astra_msgs", "ASTRAMSGS"), ("control_msgs", "CONTROLMSGS")];

foreach ((string, string) ROSNamespace in ToCompile)
{
    Directory.SetCurrentDirectory(Environment.GetEnvironmentVariable(ROSNamespace.Item2));
    ServiceAutoGen.GenerateDirectoryServices("./srv", tmp, ROSNamespace.Item1, false);
    MessageAutoGen.GenerateDirectoryMessages("./msg", tmp, ROSNamespace.Item1, false);
    ActionAutoGen.GenerateDirectoryActions("./action", tmp, ROSNamespace.Item1, false);
}

if (Directory.Exists(successDir))
    Directory.Delete(tmp, true);
else try
    {
        Directory.Move(tmp, successDir);
    }
    catch (IOException E)
    {
        Console.Out.Write("Files in use!");
        Directory.Delete(tmp, true);
        return;
    }
return;
