using Godot;
using System.Collections.Generic;
using System.Linq;

namespace ui
{
    public partial class TabController : HBoxContainer
    {
        [Export]
        public Control TabsParent;
        public static Control StaticTabsParent;

        [Export]
        public short selectedTab = 0;

        public List<Button> buttons;
        public List<BaseTabUI> tabs;

        public override void _Ready()
        {
            StaticTabsParent = TabsParent;

            buttons = [.. GetChildren().Where(static _ => _ is Button).Cast<Button>()];

            short i = 0;
            while (i < buttons.Count)
            {
                buttons[i].Pressed += SwitchTab;
                i++;
            }

            tabs = [.. TabsParent.GetChildren().Where(static _ => _ is BaseTabUI).Cast<BaseTabUI>()];

            i = 0;
            while (i < tabs.Count)
            {
                tabs[i].ProcessMode = ProcessModeEnum.Disabled;
                i++;
            }
            tabs[selectedTab].ProcessMode = ProcessModeEnum.Inherit;
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
                    // Makes this node not process when it's not visible
                    tabs[i].ProcessMode = ProcessModeEnum.Disabled;
                }
                if (buttons[i].ButtonPressed)
                    t = i;
                i++;
            }
            selectedTab = t;
            buttons[t].Disabled = true;
            // Makes this node process when it's visible
            tabs[t].ProcessMode = ProcessModeEnum.Inherit;
            tabs[t].Show();
        }
    }
}
