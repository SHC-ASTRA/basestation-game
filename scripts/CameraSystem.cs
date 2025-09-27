using Godot;
using static Godot.OS;

public partial class CameraSystem : Control
{
    public override void _Ready()
    {
        StartTestStream();
    }

    public void StartTestStream()
    {
        CreateProcess("bash", ["-c", "./scripts/bash/launchstreamserver.sh"]);
        Execute("bash", ["-c", "./scripts/bash/sleep.sh"]);
    }
}
