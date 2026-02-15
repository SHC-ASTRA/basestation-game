using Godot;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.MessageTypes.Astra;

namespace UI.Debug
{
    public partial class Socket : Debug<SocketFeedback>
    {
        [ExportGroup("Axis0")]
        [Export]
        Label Axis0Angle, Axis0Current, Axis0Temp, Axis0Voltage;
        [ExportGroup("Axis1")]
        [Export]
        Label Axis1Angle, Axis1Current, Axis1Temp, Axis1Voltage;
        [ExportGroup("Axis2")]
        [Export]
        Label Axis2Angle, Axis2Current, Axis2Temp, Axis2Voltage;
        [ExportGroup("Axis3")]
        [Export]
        Label Axis3Angle, Axis3Current, Axis3Temp, Axis3Voltage;


        private const string AngleText = "_Angle: ", CurrentText = "_Curnt: ", TempText = "_Temp: ", VoltageText = "_Vltge: ";
        private Axis Axis0, Axis1, Axis2, Axis3;

        public override void _Ready()
        {
            Axis0 = new(0, Axis0Angle, Axis0Current, Axis0Temp, Axis0Voltage);
            Axis1 = new(1, Axis1Angle, Axis1Current, Axis1Temp, Axis1Voltage);
            Axis2 = new(2, Axis2Angle, Axis2Current, Axis2Temp, Axis2Voltage);
            Axis3 = new(3, Axis3Angle, Axis3Current, Axis3Temp, Axis3Voltage);
            base._Ready();
        }

        public override SubscriptionHandler<SocketFeedback> GetFeedbackHandler() => new((feedback) =>
        {
            if (!Visible)
                return;
            Axis0.Set(feedback.axis0_angle, feedback.axis0_current, feedback.axis0_temp, feedback.axis0_voltage);
            Axis1.Set(feedback.axis1_angle, feedback.axis1_current, feedback.axis1_temp, feedback.axis1_voltage);
            Axis2.Set(feedback.axis2_angle, feedback.axis2_current, feedback.axis2_temp, feedback.axis2_voltage);
            Axis3.Set(feedback.axis3_angle, feedback.axis3_current, feedback.axis3_temp, feedback.axis3_voltage);
        });

        sealed class Axis(int ID, Label A, Label C, Label T, Label V)
        {
            private int id = ID;

            private int AngleLength = AngleText.Length + 4,
            CurrentLength = CurrentText.Length + 4,
            TempLength = TempText.Length + 4,
            VoltageLength = VoltageText.Length + 4;

            public Label Angle = A, Current = C, Temp = T, Voltage = V;
            public void Set(float _Angle, float _Current, float _Temp, float _Voltage)
            {
                Angle.Text = $"{id}{AngleText}{_Angle.ToString()}".PadRight(AngleLength);
                Current.Text = $"{id}{CurrentText}{_Current.ToString()}".PadRight(CurrentLength);
                Temp.Text = $"{id}{TempText}{_Temp.ToString()}".PadRight(TempLength);
                Voltage.Text = $"{id}{VoltageText}{_Voltage.ToString()}".PadRight(VoltageLength);
            }
        }
    }
}
