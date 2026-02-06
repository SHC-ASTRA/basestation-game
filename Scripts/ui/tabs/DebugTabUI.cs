using Godot;
using System.Linq;

namespace UI.Debug
{
    public partial class DebugTabUI : BaseTabUI
    {
        [Export] public Voltages VoltageController;

        [Export] public Control InfoContainer;

        [Export] public OptionButton DD;

        private OptionButton[] SubDDs;

        private Control ActivePanel;
        private Node ActivePanelParent;
        private int ActivePanelIndex;

        private int LastMainIndex = -1;

        public override void _Ready()
        {
            foreach (Node N in DD.GetChildren())
                DD.AddItem(N.Name);
            DD.Selected = 0;

            // Sub-dropdowns are children of the main dropdown
            SubDDs = DD.GetChildren().OfType<OptionButton>().ToArray();

            DD.ItemSelected += OnMainSelected;

            for (int i = 0; i < SubDDs.Length; i++)
            {
                int subIndex = i;

                foreach (Node N in SubDDs[i].GetChildren())
                    SubDDs[i].AddItem(N.Name);
                SubDDs[i].Selected = 0;

                SubDDs[i].ItemSelected += (i) => ShowPanel(subIndex, (int)i);
            }

            // Initialize first main tab
            OnMainSelected(DD.Selected);
        }

        private void OnMainSelected(long index)
        {
            // Restore active panel (if any)
            RestoreActivePanel();

            // Hide old sub-dropdown
            if (LastMainIndex != -1)
                SubDDs[LastMainIndex].Visible = false;

            // Show and reset sub-dropdown selection
            SubDDs[index].Visible = true;
            SubDDs[index].Select(0);

            LastMainIndex = (int)index;

            // Show its first panel
            ShowPanel((int)index, 0);
        }

        private void ShowPanel(int subDDIndex, int itemIndex)
        {
            RestoreActivePanel();

            Control panel = SubDDs[subDDIndex].GetChild<Control>(itemIndex);

            ActivePanel = panel;
            ActivePanelParent = panel.GetParent();
            ActivePanelIndex = panel.GetIndex();

            panel.Reparent(InfoContainer);
            panel.Visible = true;
        }

        private void RestoreActivePanel()
        {
            if (ActivePanel == null)
                return;

            ActivePanel.Visible = false;
            ActivePanel.Reparent(ActivePanelParent);
            ActivePanelParent.MoveChild(ActivePanel, ActivePanelIndex);

            ActivePanel = null;
        }

        public override void AdvertiseToROS() { }
        public override void EmitToROS() { }
        public override void _ExitTree() { }
    }
}
