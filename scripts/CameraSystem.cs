using Godot;
using static Godot.OS;

public partial class CameraSystem : Control
{
	public void _on_ready()
	{
		StartTestStream();
	}

	public void StartTestStream()
	{
		CreateProcess("bash", ["-c", "./scripts/bash/launchstreamserver.sh"]);
		Execute("bash", ["-c", "./scripts/bash/sleep.sh"]);
		CreateProcess("bash", ["-c", "./scripts/bash/launchtestrestreamer.sh"]);
		Execute("bash", ["-c", "./scripts/bash/sleep.sh"]);
		CreateProcess("bash", ["-c", "./scripts/bash/launchtestclient.sh"]);
	}

	public static void KILLSTREAMSERVER()
	{
		CreateProcess("bash", ["-c", "scripts/bash/killstreamserver.sh"]);
	}

	public override void _Notification(int what)
	{
		if (what == NotificationWMCloseRequest)
		{
			KILLSTREAMSERVER();
			GetTree().Quit(); // default behavior
		}
	}
}
