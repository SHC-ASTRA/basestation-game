using Godot;

public partial class DraggableSizeSetter : FoldableContainer
{
    [Export]
    Draggable target;
    FoldableContainer tfc;

    [Export]
    Button enable;
    [Export]
    Button reset;

    [Export]
    TextEdit X;
    [Export]
    TextEdit Y;


    public override void _Ready()
    {
        tfc = (target.GetChild(0) as FoldableContainer);
        X.Text = tfc.Size.X.ToString();
        Y.Text = tfc.Size.Y.ToString();

        X.TextChanged += Resize;
        Y.TextChanged += Resize;
        enable.Toggled += SetEnabled;
        reset.Pressed += ResetPos;
    }

    void SetEnabled(bool t)
    {
        target.Visible = t;
        ResetPos();
    }

    void ResetPos()
    {
        target.Position = GetWindow().Size / 2;
    }

    void Resize()
    {
        if (!X.Text.IsValidFloat())
        {
            X.Text = "0";
            return;
        }
        else if (!Y.Text.IsValidFloat())
        {
            Y.Text = "0";
            return;
        }

        tfc.Size = new Vector2(float.Parse(X.Text), float.Parse(Y.Text));
    }
}
