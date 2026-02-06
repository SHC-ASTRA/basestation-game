using Godot;

namespace UI
{
    public partial class MidwayHProgressBar : Control
    {
        [Export] public float Max;
        [Export] public float Min;

        private Panel HBar;
        private float _value;
        public float Value
        {
            get => _value;
            set
            {
                _value = Mathf.Clamp(value, Min, Max);
                HBar.Position = new Vector2(Mathf.Lerp(0f, Size.X - HBar.Size.X, (_value - Min) / (Max - Min)), HBar.Position.Y);
            }
        }

        public override void _Ready()
        {
            HBar = GetChild<Panel>(0);
            Value = (Max + Min) * 0.5f;
        }
    }
}
