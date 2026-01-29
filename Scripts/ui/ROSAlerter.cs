using IPC;
using Godot;

namespace ui
{
    public partial class ROSAlerter : VBoxContainer
    {
        [Export]
        private Label connectText;

        [Export]
        private Panel connectBubble;

        [Export]
        private Button reconnectButton;

        private Color True = new Color(0xaafd00ff);
        private Color False = new Color(0xd24444ff);

        public override void _Ready()
        {
            reconnectButton.Pressed += () => { ROS.Readvertise = true; ROS.StartROS(); };
        }

        public void Disconnected()
        {
            connectText.Text = "ROS Disconnected";
            connectBubble.SelfModulate = False;
        }

        public void Connected()
        {
            connectText.Text = "ROS Connected";
            connectBubble.SelfModulate = True;
        }
    }
}
