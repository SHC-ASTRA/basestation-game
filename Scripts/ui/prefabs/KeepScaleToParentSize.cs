using Godot;

public partial class KeepScaleToParentSize : Control
{
    public override void _Ready()
    {
        Vector2 psize = (GetParent() as Control).Size;
        if (psize.X < psize.Y)
            Scale = new Vector2(psize.Y, psize.Y);
        else Scale = new Vector2(psize.X, psize.X);
    }
}
