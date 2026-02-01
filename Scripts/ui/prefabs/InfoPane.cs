using Godot;
using System.Linq;

public partial class InfoPane : OptionButton
{
    private long lastSelected;
    private long[] lastChildSelections;
    OptionButton[] ChildButtons;
    private Node lastInfoParent;
    public HBoxContainer InfoContainer;

    public override void _Ready()
    {
        ChildButtons = [.. GetChildren().Where(static _ => _ is OptionButton).Cast<OptionButton>()];
        lastChildSelections = new long[ChildButtons.Length];

        ItemSelected += (_) =>
        {
            ChildButtons[lastSelected].Visible = false;
            ChildButtons[Selected].Visible = true;
            lastSelected = Selected;
        };

        for (int i = 0; i < ChildButtons.Length; i++)
        {
            ChildButtons[i].ItemSelected += (_) =>
            {
                (ChildButtons[i].GetChildren()[(int)lastChildSelections[i]] as Control).Visible = false;
                InfoContainer.GetChild(-1).Reparent(lastInfoParent);

                Control c = ChildButtons[i].GetChildren()[ChildButtons[i].Selected] as Control;
                lastInfoParent = c.GetParent();
                c.Reparent(InfoContainer);
                c.Visible = true;

                lastChildSelections[i] = ChildButtons[i].Selected;
            };
        }
    }
}
