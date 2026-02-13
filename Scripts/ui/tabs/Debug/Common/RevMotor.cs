using Godot;
using RosSharp.RosBridgeClient.MessageTypes.Astra;

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
            id.Text = $"id: {R.id}";
            position.Text = $"pos: {R.position}";
            velocity.Text = $"vel: {R.velocity}";
            voltage.Text = $"vol: {R.voltage}";
            current.Text = $"cur: {R.current}";
            temperature.Text = $"temp: {R.temperature}";
        }
    }
}
