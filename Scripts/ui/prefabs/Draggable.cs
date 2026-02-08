using Godot;

namespace UI
{
    public partial class Draggable : Control
    {
        [Export]
        private bool Global = true;
        private bool dragging = false;
        private Vector2 originalDownPosition;

        public override void _GuiInput(InputEvent @event)
        {
            if (@event is InputEventMouseButton)
            {
                if ((@event as InputEventMouseButton).IsReleased())
                    dragging = false;
                else if ((@event as InputEventMouseButton).IsPressed())
                {
                    dragging = true;
                    originalDownPosition = GetLocalMousePosition();
                }
                else base._Input(@event);
            }
            if (dragging)
                if (Global)
                    SetPosition(GetGlobalMousePosition() - originalDownPosition);
                else
                    SetPosition((GetParent() as CanvasItem).GetLocalMousePosition() - originalDownPosition);
        }
    }
}
