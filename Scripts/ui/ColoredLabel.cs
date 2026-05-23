using Godot;
using static UI.Debug.Debug;

// I SWEAR this is NOT racist
namespace UI
{
    public partial class ColoredLabel : BoxContainer
    {
        [ExportGroup("Numbers")]
        [Export]
        public float High;
        [Export]
        public float Middle = 0;
        [Export]
        public float Low;

        [ExportGroup("Colors")]
        [Export]
        public Color HighColor;
        [Export]
        public Color MiddleColor;
        [Export]
        public Color LowColor;

        [ExportGroup("Labels")]
        [Export]
        public Label BaseLabel;
        [Export]
        public string BaseText;

        [Export]
        public Label ValueLabel;

        public void SetValue(string fmt, params float[] args)
        {
            args[0] = Mathf.Clamp(args[0], Low, High);

            float t = (args[0] - Low) / (High - Low);
            float tMid = (Middle - Low) / (High - Low);

            float closenessLow = (tMid >= t) ? 1f - (t - 0f) / tMid : 1f - (t - 0f);
            float closenessHigh = (tMid <= t) ? 1f - (1f - t) / 1f - tMid : 1f - (1f - t);
            float closenessMid = 1f - Mathf.Abs(t - tMid) / Mathf.Max(tMid, 1f - tMid);

            float sum = closenessLow + closenessMid + closenessHigh;
            if (sum <= 0f) sum = 1f;
            closenessLow /= sum;
            closenessMid /= sum;
            closenessHigh /= sum;

            // Blend
            ValueLabel.SelfModulate = new Color(
                closenessLow * LowColor.R + closenessMid * MiddleColor.R + closenessHigh * HighColor.R,
                closenessLow * LowColor.G + closenessMid * MiddleColor.G + closenessHigh * HighColor.G,
                closenessLow * LowColor.B + closenessMid * MiddleColor.B + closenessHigh * HighColor.B,
                closenessLow * LowColor.A + closenessMid * MiddleColor.A + closenessHigh * HighColor.A
            );
            if (args.Length > 1)
            {
                int c = 0;
                while (c < args.Length)
                {
                    fmt = ReplaceSubstring(fmt, $"[VALUE{c + 1}]", args[c].ToString());
                    c++;
                }
                SetLabelText(ValueLabel, fmt);
            }
            else SetLabelText(ValueLabel, ReplaceSubstring(fmt, "[VALUE]", args[0].ToString()));
        }

        private static string ReplaceSubstring(string fmt, string toReplace, string replaceWith)
        {
            int vpos = fmt.Find(toReplace);
            if (vpos == -1) return "";
            return string.Concat(fmt[..vpos], replaceWith, fmt[(vpos + toReplace.Length)..]);
        }

        public override void _Ready()
        {
            BaseLabel.Text = BaseText;
            if (Middle == 0)
                Middle = (High - Low) * 0.5f;

            base._Ready();
        }
    }
}
