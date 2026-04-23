using Godot;
using System.Linq;
using System.Collections.Generic;

namespace UI
{
    public partial class TabController : HBoxContainer
    {
        [Export]
        public Control TabsParent;
        public static Control StaticTabsParent;

        private short selectedTab = 0;

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
                // Automatically set the selected tab and disable the button for the visible tab(s???)
                if (tabs[i].Visible)
                {
                    buttons[i].Disabled = true;
                    selectedTab = i;
                }
                tabs[i].ProcessMode = ProcessModeEnum.Disabled;
                i++;
            }
            tabs[selectedTab].ProcessMode = ProcessModeEnum.Inherit;
        }

        public void SwitchTab()
        {
            buttons[selectedTab].Disabled = false;
            tabs[selectedTab].Visible = false;
            tabs[selectedTab].ProcessMode = ProcessModeEnum.Disabled;

            for (short i = 0; i < buttons.Count; i++)
            {
                if (buttons[i].ButtonPressed)
                {
                    selectedTab = i;
                    buttons[i].Disabled = true;
                    // Makes this node process when it's visible
                    tabs[i].ProcessMode = ProcessModeEnum.Inherit;
                    tabs[i].Visible = true;
                }
            }
        }
    }
}
