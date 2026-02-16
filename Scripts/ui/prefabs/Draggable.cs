using Godot;

namespace UI
{
    public partial class Draggable : Control
    {
        [Export]
        private bool Global = true;
        private bool Dragging = false;
        private Vector2 originalDownPosition;

        public override void _GuiInput(InputEvent @event)
        {
            if (@event is InputEventMouseButton)
            {
                if ((@event as InputEventMouseButton).IsReleased())
                    Dragging = false;
                else if ((@event as InputEventMouseButton).IsPressed())
                {
                    Dragging = true;
                    originalDownPosition = GetLocalMousePosition();
                }
                else base._Input(@event);
            }
            if (Dragging)
                if (Global)
                {
                    SetPosition(GetGlobalMousePosition() - originalDownPosition);
                    Dragged();
                }
                else
                {
                    SetPosition((GetParent() as CanvasItem).GetLocalMousePosition() - originalDownPosition);
                    Dragged();
                }
        }

        public virtual void Dragged() { }
    }
}
