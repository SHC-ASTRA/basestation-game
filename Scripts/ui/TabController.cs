using Godot;
using System.Collections.Generic;
using System.Linq;
namespace ui
{
    public partial class TabController : HBoxContainer
    {
        [Export]
        public Control TabsParent;

        [Export]
        public short selectedTab = 0;

        public List<Button> buttons;
        public List<BaseTabUI> tabs;

        public override void _Ready()
        {
            buttons = [.. GetChildren().Where(static _ => _ is Button).Cast<Button>()];

            short i = 0;
            while (i < buttons.Count)
            {
                buttons[i].Pressed += SwitchTab;
                i++;
            }

            tabs = [.. TabsParent.GetChildren().Where(static _ => _ is BaseTabUI).Cast<BaseTabUI>()];
        }

        public override void _Process(double delta)
        {

        }

        public void SwitchTab()
        {
            short i = 0;
            short t = -1;
            while (i < buttons.Count)
            {
                if (i == selectedTab)
                {
                    buttons[i].Disabled = false;
                    tabs[i].Hide();
                }
                if (buttons[i].ButtonPressed)
                    t = i;
                i++;
            }
            selectedTab = t;
            buttons[t].Disabled = true;
            tabs[t].Show();
        }
    }
}
