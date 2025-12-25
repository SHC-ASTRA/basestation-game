using Godot;

public partial class ColoredIndicator : Control
{
    public bool Value
    {
        set
        {
            True.Visible = value;
            False.Visible = !value;
        }
    }
    private Panel True, False;
    public override void _Ready()
    {
        True = GetChild<Panel>(0);
        False = GetChild<Panel>(1);
        Value = false;
        base._Ready();
    }
}
