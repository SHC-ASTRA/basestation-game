using Godot;
using static UI.Debug.Debug;

namespace UI.Debug
{
    public partial class Voltages : Control
    {
        [Export]
        public Label BatVoltage, Voltage12, Voltage5, Voltage3;
        public void Update(BoardVoltage v)
        {
            SetLabelText(BatVoltage, $" Bat volt: {v.vbatt} | {Mathf.RoundToInt(100 * v.vbatt / 14.8f)}%");
            SetLabelText(Voltage12, $" v12: {v.v12}");
            SetLabelText(Voltage5, $" v5: {v.v5}");
            SetLabelText(Voltage3, $" v3: {v.v3}");
        }
    }
}
