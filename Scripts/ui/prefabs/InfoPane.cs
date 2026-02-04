using Godot;
using System.Linq;

public partial class InfoPane : OptionButton
{
    private long lastSelected;
    private long[] lastChildSelections;
    OptionButton[] ChildButtons;
    private Node lastInfoParent;
    private int lastIndex = 0;
    public HBoxContainer InfoContainer;

    public override void _Ready()
    {
        InfoContainer = GetParent().GetChild(-1).GetChild(0) as HBoxContainer;

        ChildButtons = [.. GetChildren().Where(static _ => _ is OptionButton).Cast<OptionButton>()];

        // It's so much simpler for the first DD :)
        ItemSelected += (_) =>
        {
            ChildButtons[lastSelected].Visible = false;
            ChildButtons[Selected].Visible = true;
            lastSelected = Selected;
        };

        lastInfoParent = ChildButtons[Selected];

        for (int i = 0; i < ChildButtons.Length; i++)
        {
            // This has to be done because just using 'i' raw in a lambda
            // uses a copy of the final state of i in this scope. Don't understand *how*,
            // but it just is.
            childButtonContainer cbp = new(i);
            ChildButtons[i].ItemSelected += (_) =>
            {
                // Refs are pointers
                cbp.run(ref ChildButtons, ref InfoContainer, ref lastIndex, ref lastInfoParent);
            };
        }
    }

    private class childButtonContainer(int _i) // <- this is an integral constructor
    {
        private int i = _i;

        // This is like... horrid. But it's neat and requires no editor assignments which is a bit more stable.
        public void run(ref OptionButton[] ChildButtons, ref HBoxContainer InfoContainer, ref int lastIndex, ref Node LastInfoParent)
        {
            // Hide and move the currently displayed DD and info to the position it was in previously
            (InfoContainer.GetChild(-1) as Control).Visible = false;
            InfoContainer.GetChild(-1).Reparent(LastInfoParent);
            LastInfoParent.MoveChild(LastInfoParent.GetChild(-1), lastIndex);

            // Ref the selected DDs selected item
            Control c = ChildButtons[i].GetChildren()[ChildButtons[i].Selected] as Control;
            // Copy its index so that we can move it back to its original position next time
            lastIndex = c.GetIndex();
            // Copy its parent so that we can reparent it correctly
            LastInfoParent = c.GetParent();
            // Move it to the info container
            c.Reparent(InfoContainer);
            c.Visible = true;
        }
    }
}
