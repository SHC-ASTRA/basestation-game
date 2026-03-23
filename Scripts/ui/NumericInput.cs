using Godot;

public partial class NumericInput : LineEdit
{
    [Export]
    bool allow_fractional;

    public override void _Ready()
    {
        this.TextChanged += (string s) =>
        {
            if (this.CaretColumn - 1 < 0)
                return;
            char c = s[this.CaretColumn - 1];
            if (c == '.')
            {
                if (!allow_fractional || s.Count(".") > 1)
                    this.DeleteCharAtCaret();
            }
            else if ((c < '0' || c > '9'))
                this.DeleteCharAtCaret();
        };
    }
}
