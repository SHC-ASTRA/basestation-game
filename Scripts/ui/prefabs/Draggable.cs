using Godot;

namespace UI
{
    public partial class Draggable : FoldableContainer
    {
        private bool dragging = false;
        private Vector2 originalDownPosition;

        public override void _GuiInput(InputEvent @event)
        {
            if (@event is InputEventMouseButton)
            {
                InputEventMouseButton IEMB = @event as InputEventMouseButton;
                if (IEMB.IsReleased())
                    dragging = false;
                else if (IEMB.IsPressed())
                {
                    dragging = true;
                    originalDownPosition = GetLocalMousePosition();
                }
                else base._Input(@event);
            }
            if (dragging)
                SetPosition(GetGlobalMousePosition() - originalDownPosition);
        }
    }
}
