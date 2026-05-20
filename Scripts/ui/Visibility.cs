using Godot;

namespace UI.Debug
{
    public partial class Visibility : Control
    {
        protected bool visible;
        public override void _Process(double delta)
        {
            visible = Visible;
            base._Process(delta);
        }
    }
}