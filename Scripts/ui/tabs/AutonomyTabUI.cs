using IPC;
using Godot;
using UI.Debug;
using System.Threading.Tasks;
using RosSharp.RosBridgeClient.MessageTypes.Astra;

namespace UI
{
    public partial class AutonomyTabUI : BaseTabUI
    {
        internal NewCoreFeedback feedback;

        [Export]
        Button RadioFeedback, RadioControl;
        [Export]
        Control Feedback, Control;

        [Export]
        Button SendControls;

        [ExportGroup("Board")]
        [Export]
        Voltages Voltages;

        [ExportGroup("IMU")]
        [Export]
        IMU imu;

        [ExportGroup("Motors")]
        Control MotorContainer;
        [Export]
        Motors motors;

        public override void _Ready()
        {
            base._Ready();
            Task.Run(async () =>
            {
                while (!ROS.ROSReady)
                    await Task.Delay(5);
                ROS.TopicSubscribe<NewCoreFeedback>("/core/feedback_new", (feedback) =>
                {
                    this.feedback = feedback;
                    if (Visible)
                        CallDeferred(nameof(FeedbackHandler));
                });
            });
            // RadioFeedback.Toggled += (bool t) =>
            // {
            //     RadioControl.SetPressedNoSignal(!t);
            //     Feedback.Visible = t;
            //     Control.Visible = !t;
            // };
            // RadioControl.Toggled += (bool t) =>
            // {
            //     RadioFeedback.SetPressedNoSignal(!t);
            //     Control.Visible = t;
            //     Feedback.Visible = !t;
            // };
        }

        public override void AdvertiseToROS()
        {
            // ROS.RequestTopic<AutoNav>("");
        }

        public override void _Process(double delta)
        {

        }

        public void FeedbackHandler()
        {
            Voltages.Update(feedback.board_voltage);

            motors.Update(feedback.fl_motor, feedback.bl_motor, feedback.fr_motor, feedback.br_motor);

            imu.Update(feedback.orientation, feedback.imu_calib);
        }

        public override void EmitToROS() { }
        public override void _ExitTree() { }
    }
}
