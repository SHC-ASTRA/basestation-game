using IPC;
using Godot;
using UI.Debug;
using UI.Debug.Core;
using System.Threading.Tasks;

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

        public override void _Process(double delta) { }

        public void FeedbackHandler()
        {
            if (!Visible)
                return;

            Voltages.Update(feedback.board_voltage);

            motors.Update(feedback.fl_motor, feedback.bl_motor, feedback.fr_motor, feedback.br_motor);

            imu.Update(feedback.orientation, feedback.imu_calib);
        }

        public override void EmitToROS() { }
        public override void STARTTAB()
        {
            // ROS.RequestTopic<AutoNav>("");

            ROS.TopicSubscribe<NewCoreFeedback>(FEEDBACK.COREMAIN, (feedback) =>
            {
                this.feedback = feedback;
                CallDeferred(nameof(FeedbackHandler));
            });
        }
        public override void STOPTAB()
        {
            ROS.TopicUnsubscribe(FEEDBACK.COREMAIN);
        }
    }
}
