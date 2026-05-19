using Godot;
using static UI.Debug.Debug;

namespace UI.Debug
{
    public partial class RevMotor : Control
    {
        [Export]
        public Label name;
        [Export]
        Label id, position, velocity, voltage, current, temperature;
        public void Update(RevMotorState R)
        {
            SetLabelText(id, $"id: {R.id}");
            SetLabelText(position, $"pos: {R.position}");
            SetLabelText(velocity, $"vel: {R.velocity}");
            SetLabelText(voltage, $"vol: {R.voltage}");
            SetLabelText(current, $"cur: {R.current}");
            SetLabelText(temperature, $"temp: {R.temperature}");
        }
    }
}
