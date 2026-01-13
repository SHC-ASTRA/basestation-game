using Godot;

public partial class DraggableSizeSetter : FoldableContainer
{
    [Export]
    Draggable target;

    [Export]
    Button enable;
    [Export]
    Button reset;

    TextEdit X, Y;

    public override void _Ready()
    {
        X = GetChild(0).GetChild(1) as TextEdit;
        Y = GetChild(0).GetChild(2) as TextEdit; 

        target.Size = new Vector2(500, 500);

        X.Text = target.Size.X.ToString();
        Y.Text = target.Size.Y.ToString();

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

    void ResetPos() => target.Position = (GetWindow().Size / 2) - (target.Size / 2);

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

        target.Size = new Vector2(float.Parse(X.Text), float.Parse(Y.Text));
    }
}
