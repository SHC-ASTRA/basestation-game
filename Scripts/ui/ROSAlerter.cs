using IPC;
using Godot;

namespace ui
{
    ///<summary> Gives the user control over ROSBridge </summary>
    public partial class ROSAlerter : VBoxContainer
    {
        [Export]
        private Label connectText;

        [Export]
        private Panel connectBubble;

        [Export]
        private Button reconnectButton;

        private Color connected = new Color(0xaafd00ff);
        private Color disconnected = new Color(0xd24444ff);

        public override void _Ready()
        {
            // Forces ROSBridge to throw out its ownership over all current topics and reset itself
            reconnectButton.Pressed += () => { ROS.Readvertise = true; ROS.StartROS(); };
        }

        public void Disconnected()
        {
            connectText.Text = "ROS Disconnected";
            connectBubble.SelfModulate = disconnected;
        }

        public void Connected()
        {
            connectText.Text = "ROS Connected";
            connectBubble.SelfModulate = connected;
        }
    }
}
