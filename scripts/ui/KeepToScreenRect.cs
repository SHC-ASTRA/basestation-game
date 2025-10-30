using Godot;

namespace ui
{
    public partial class KeepToScreenRect : VSplitContainer
    {
        public override void _Ready()
        {
            SetSize(GetWindow().Size);
            Dragged += EnforceMinimumHeight;
            GetWindow().SizeChanged += SizeChanged;
            EnforceMinimumHeight(0);
            base._Ready();
        }

        public void SizeChanged()
        {
            int tenthheight = -(int)(GetWindow().Size.Y * 0.1f);
            if (SplitOffset > tenthheight)
                SplitOffset = tenthheight;
        }

        public void EnforceMinimumHeight(long offset)
        {
            int tenthheight = -(int)(GetWindow().Size.Y * 0.1f);
            if (offset > tenthheight)
                SplitOffset = tenthheight;
        }
    }
}
