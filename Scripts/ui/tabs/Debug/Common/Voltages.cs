using Godot;

namespace UI.Debug
{
    public partial class Voltages : Control
    {
        [Export]
        public Label NameLabel;
        [Export]
        public ColoredLabel BatVoltage, Voltage12, Voltage5, Voltage3;

        public void Update(BoardVoltage v)
        {
            BatVoltage.SetValue("[VALUE1]v | [VALUE2]%", v.vbatt, Mathf.RoundToInt(v.vbatt * 5.952380952380952 /* 100 / 16.8 */));
            Voltage12.SetValue("[VALUE]v", v.v12);
            Voltage5.SetValue("[VALUE]v", v.v5);
            Voltage3.SetValue("[VALUE]v", v.v3);
        }
    }
}
