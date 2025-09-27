using Godot;
using RosSharp.RosBridgeClient;
using System.Collections.Generic;
using RosSharp.RosBridgeClient.Protocols;

public partial class inputDaemon : Node
{
    [Signal]
    public delegate void Rover_DS_Longitudinal_EventHandler();
    [Signal]
    public delegate void Rover_DS_Turn_EventHandler();
    [Signal]
    public delegate void TAKE_ARM_CONTROL_EventHandler();
    [Signal]
    public delegate void SWITCH_IK_MODE_EventHandler();
    [Signal]
    public delegate void BLOW_UP_DRONE_EventHandler();
    [Signal]
    public delegate void BLOW_UP_CORE_PCB_EventHandler();

    public Dictionary<string, string> InputLUT = new(){
        {"W",SignalName.Rover_DS_Longitudinal_},
        {"S",SignalName.Rover_DS_Longitudinal_},

        {"A",SignalName.Rover_DS_Turn_},
        {"D",SignalName.Rover_DS_Turn_},

        {"Shift+Enter", SignalName.TAKE_ARM_CONTROL_},
        {"R", SignalName.SWITCH_IK_MODE_},

        {"Ctrl+Shift+Alt+Meta+D", SignalName.BLOW_UP_DRONE_},
        {"Ctrl+Shift+Alt+Meta+P", SignalName.BLOW_UP_CORE_PCB_}
    };

    public override void _Ready()
    {
        base._Ready();

        RosSocket RC = new RosSocket(new WebSocketNetProtocol("ws://localhost:6969"));
        Message M = new RosSharp.RosBridgeClient.MessageTypes.Std.String();
        RC.Publish("a", M);
    }

    string r = "", s;
    public override void _Input(InputEvent @event)
    {
        r = "";
        s = @event.AsText();
        GD.Print(s);
        if (InputLUT.TryGetValue(s, out r)) EmitSignal(r);
    }
}
