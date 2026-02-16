using Godot;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Astra;

namespace UI.Debug
{
    public partial class CoreDebug : Debug<NewCoreFeedback>
    {
        [ExportGroup("Board")]
        [Export]
        Voltages Voltages;

        [ExportGroup("IMU")]
        [Export]
        IMU imu;

        [ExportGroup("Motors")]
        Control MotorContainer;
        [Export]
        RevMotor BL, BR, FL, FR;

        public override void _Ready()
        {
            FL.name.Text = nameof(FL); FR.name.Text = nameof(FR); BL.name.Text = nameof(BL); BR.name.Text = nameof(BR);

            MotorContainer = (BL.GetParent() as Control);
            base._Ready();
        }

        public override SubscriptionHandler<NewCoreFeedback> GetFeedbackHandler() => new(feedback =>
        {
            Voltages.Update(feedback.board_voltage);

            if (MotorContainer.Visible)
            {
                FL.Update(feedback.fl_motor);
                BL.Update(feedback.bl_motor);
                FR.Update(feedback.fr_motor);
                BR.Update(feedback.br_motor);
            }
        });
    }
}
