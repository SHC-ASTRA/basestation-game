using Godot;

namespace UI
{
    public partial class MapPin : Button
    {
        [Export]
        public Panel PopupPanel;

        [Export]
        public LineEdit PanelName;

        [Export]
        public OptionButton PanelNamePresets;

        [Export]
        public Label X, Y;

        [Export]
        public Button DeleteButton;

        [Export]
        public Label UnderPin;

        public override void _Ready()
        {
            Toggled += (T) => PopupPanel.Visible = T;

            PanelNamePresets.ItemSelected += (_) => { PanelName.Text = PanelNamePresets.Text; UnderPin.Text = PanelNamePresets.Text; };

            DeleteButton.Pressed += () => QueueFree();

            PanelName.TextChanged += (_) => UnderPin.Text = _;

            this.Position -= new Vector2(16, 42);
        }
    }
}
