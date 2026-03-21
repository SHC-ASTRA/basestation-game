using Godot;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Astra;

namespace UI.Debug
{
    public partial class CoreDebug : Debug<NewCoreFeedback>
    {
        [ExportGroup("Board")]
        [Export]
        Voltages voltages;

        [ExportGroup("IMU")]
        [Export]
        IMU imu;

        [ExportGroup("Motors")]
        [Export]
        Motors motors;

        public override void FeedbackHandler()
        {
            voltages.Update(feedback.board_voltage);

            imu.Update(feedback.orientation, feedback.imu_calib);

            motors.Update(feedback.fl_motor, feedback.bl_motor, feedback.fr_motor, feedback.br_motor);
        }
    }
}
