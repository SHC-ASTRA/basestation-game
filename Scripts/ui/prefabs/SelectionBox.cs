using Godot;

public partial class SelectionBox : MenuButton
{
    public int prevIndex;

    public override void _Ready()
    {
        base._Ready();
        GetPopup().IndexPressed += (index) =>
        {
            PopupMenu PM = GetPopup();
            PM.SetItemChecked(prevIndex, false);
            PM.SetItemDisabled(prevIndex, false);

            prevIndex = (int)index;

            PM.SetItemChecked(prevIndex, true);
            PM.SetItemDisabled(prevIndex, false);
        };
    }
}
