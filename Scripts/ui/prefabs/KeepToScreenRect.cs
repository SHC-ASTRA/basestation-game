using Godot;

namespace ui
{
    public partial class KeepToScreenRect : Control
    {
        public override void _Ready()
        {
            GetWindow().SizeChanged += () => SetSize(GetWindow().Size, true);
            base._Ready();
        }
    }
}
