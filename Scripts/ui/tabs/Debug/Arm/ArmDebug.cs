using Godot;
using static UI.Debug.Debug;

namespace UI.Debug.Arm
{
    public partial class ArmDebug : FeedbackProvider<ArmFeedback>
    {
        public static RevMotorState Axis0, Axis1, Axis2, Axis3;

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
        public RevMotor A0, A1, A2, A3;

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
                    DigitVoltages = VoltagesDisplay.Instantiate() as Voltages;
                    VoltagesContainer.AddChild(DigitVoltages);
                }
            };
        }

        public override void FeedbackHandler()
        {
            Axis0 = feedback.axis0_motor;
            Axis1 = feedback.axis1_motor;
            Axis2 = feedback.axis2_motor;
            Axis3 = feedback.axis3_motor;
            if (!visible)
                return;

            SocketVoltages.Update(feedback.socket_voltage);
            DigitVoltages.Update(feedback.digit_voltage);

            A0.Update(feedback.axis0_motor);
            A1.Update(feedback.axis1_motor);
            A2.Update(feedback.axis2_motor);
            A3.Update(feedback.axis3_motor);
        }
    }
}
