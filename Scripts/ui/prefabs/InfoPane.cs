using Godot;

public partial class InfoPane : Panel
{
    public override void _Ready()
    {
        ItemRectChanged += () => SetSize(new Vector2((GetParent() as Control).Size.Y, (GetParent() as Control).Size.Y));


    }
}
