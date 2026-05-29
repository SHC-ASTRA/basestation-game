using Godot;
using System.Linq;
using System.Collections.Generic;
using IPC;

namespace UI
{
    public partial class TabController : HBoxContainer
    {
        public static TabController StaticTabController;

        [Export]
        public Control TabsParent;

        public short selectedTab = 0;

        public List<Button> buttons;
        public List<BaseTabUI> tabs;

        public override void _Ready()
        {
            StaticTabController = this;

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
                i++;
            }
            ROS.RegsiterOnROSStart(() =>
            {
                if (!tabs[selectedTab].Started)
                {
                    tabs[selectedTab].Started = true;
                    tabs[selectedTab].STARTTAB();
                    tabs[selectedTab].Resume();
                }
            });
        }

        public void SwitchTab()
        {
            buttons[selectedTab].Disabled = false;

            tabs[selectedTab].Visible = false;
            tabs[selectedTab].Pause();
            tabs[selectedTab].STOPTAB();


            for (short i = 0; i < buttons.Count; i++)
            {
                if (buttons[i].ButtonPressed)
                {
                    selectedTab = i;
                    buttons[i].Disabled = true;

                    tabs[i].STARTTAB();
                    tabs[i].Resume();
                    tabs[i].Visible = true;
                }
            }
        }
    }
}
