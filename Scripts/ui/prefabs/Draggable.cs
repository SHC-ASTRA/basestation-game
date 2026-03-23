using Godot;

namespace UI
{
    public partial class Draggable : Control
    {
        [Export]
        private bool Global = true;
        private bool Dragging = false;
        private Vector2 originalDownPosition;
        private Vector2 dragOffset;
        private CanvasItem parentCanvas;

        public override void _Ready()
        {
            parentCanvas = GetParent() as CanvasItem;
            dragOffset = Size;
            dragOffset.X *= 0.5f;
        }

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
                    SetPosition(parentCanvas.GetLocalMousePosition() - dragOffset);
                    Dragged();
                }
        }

        public virtual void Dragged() { }
    }
}
