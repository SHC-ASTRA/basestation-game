using Godot;
using static UI.Debug.Debug;

namespace UI.Debug
{
    public partial class RevMotor : Control
    {
        [Export]
        public Label name;
        [Export]
        Label id, position, velocity;
        [Export]
        ColoredLabel voltage, current, temperature;

        public void Update(RevMotorState R)
        {
            SetLabelText(id, $"id: {R.id}");
            SetLabelText(position, $"pos: {R.position}");
            SetLabelText(velocity, $"vel: {R.velocity}");
            voltage.SetValue("[VALUE]v", R.voltage);
            current.SetValue("[VALUE]a", R.current);
            temperature.SetValue("[VALUE]c", R.temperature);
        }
    }
}
