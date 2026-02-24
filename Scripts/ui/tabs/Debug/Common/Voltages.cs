using Godot;
using RosSharp.RosBridgeClient.MessageTypes.Astra;

namespace UI.Debug
{
    public partial class Voltages : Control
    {
        [Export]
        public Label BatVoltage, Voltage12, Voltage5, Voltage3;
        public void Update(BoardVoltage v)
        {
            BatVoltage.Text = $" Bat volt: {v.vbatt} | {Mathf.RoundToInt(100 * v.vbatt / 14.8f).ToString()}%";
            Voltage12.Text = $" v12: {v.v12}";
            Voltage5.Text = $" v5: {v.v5}";
            Voltage3.Text = $" v3: {v.v3}";
        }
    }
}
