using Godot;

public partial class Draggable : CollisionShape2D
{
    public RectangleShape2D s;
    private Control scon;
    public Vector2 hsize;
    private Vector2 originalDownPosition;
    private static Vector2 deadzone = new Vector2(0.15f, 0.15f);

    public override void _Ready()
    {
        s = Shape as RectangleShape2D;
        scon = this.GetChild(0) as Control;

        Resize();
    }

    public void Resize()
    {
        scon.Size = s.Size;
        hsize = scon.Size * 0.5f;
        scon.Size = new Vector2(scon.Size.X, 400);
        scon.Position = -hsize;
    }

    public override void _Process(double delta)
    {
        Vector2 p = GetLocalMousePosition();
        Vector2 gp = GetGlobalMousePosition();
        if ((p.X < hsize.X && p.X > -hsize.X) && (p.Y < hsize.Y && p.Y > -hsize.Y))
        {
            if (Input.IsActionJustPressed("LeftMouseDown"))
            {
                originalDownPosition = gp;
            }
            else if (Input.IsActionPressed("LeftMouseDown") && (originalDownPosition > gp + deadzone || originalDownPosition < gp - deadzone))
            {
                GlobalPosition = gp;
            }
            else
            {
                originalDownPosition = new Vector2(999999, 999999);
            }
        }
    }
}
