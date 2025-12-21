using Godot;

namespace ui
{
    public partial class MidwayHProgressBar : Control
    {
        [Export] public float Max;
        [Export] public float Min;

        private Panel HBar;
        private float _parallax;
        public float Parallax
        {
            get => _parallax;
            set
            {
                _parallax = Mathf.Clamp(_parallax + value, Min, Max);
                HBar.Position = new Vector2(Mathf.Lerp(0f, Size.X - HBar.Size.X, (_parallax - Min) / (Max - Min)), HBar.Position.Y);
            }
        }

        public override void _Ready()
        {
            HBar = GetChild<Panel>(0);
        }
    }
}
