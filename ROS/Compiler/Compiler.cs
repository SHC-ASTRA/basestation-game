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

Directory.SetCurrentDirectory(Environment.GetEnvironmentVariable("ASTRAMSGS"));

ServiceAutoGen.GenerateDirectoryServices("./srv", tmp, "astra_msgs", false);
MessageAutoGen.GenerateDirectoryMessages("./msg", tmp, "astra_msgs", false);
ActionAutoGen.GenerateDirectoryActions("./action", tmp, "astra_msgs", false);

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
