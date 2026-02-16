using Godot;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Astra;

namespace UI.Debug
{
    public partial class ArmDebug : Debug<ArmFeedback>
    {
        [ExportGroup("Board")]
        [Export]
        HBoxContainer VoltagesContainer;
        [Export]
        PackedScene VoltagesDisplay;
        [Export]
        Voltages SocketVoltages;
        Voltages DigitVoltages;

        [ExportGroup("Motors")]
        [Export]
        RevMotor A0, A1, A2, A3;

        public override void _Ready()
        {
            A0.name.Text = nameof(A0); A1.name.Text = nameof(A1); A2.name.Text = nameof(A2); A3.name.Text = nameof(A3);

            base._Ready();

            VisibilityChanged += () =>
            {
                if (!Visible)
                    VoltagesContainer.GetChild(-1).QueueFree();
                else
                {
                    for (int i = VoltagesContainer.GetChildren().Count - 1; i >= 1; i--)
                        VoltagesContainer.GetChild(i).QueueFree();
                    DigitVoltages = (VoltagesDisplay.Instantiate() as Voltages);
                    VoltagesContainer.AddChild(DigitVoltages);
                }
            };
        }

        public override SubscriptionHandler<ArmFeedback> GetFeedbackHandler() => new((feedback) =>
        {
            if (!Visible)
                return;

            SocketVoltages.Update(feedback.socket_voltage);
            DigitVoltages.Update(feedback.digit_voltage);

            A0.Update(feedback.axis0_motor);
            A1.Update(feedback.axis1_motor);
            A2.Update(feedback.axis2_motor);
            A3.Update(feedback.axis3_motor);
        });
    }
}
