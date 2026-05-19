using Godot;
using static UI.Debug.Debug;

namespace UI.Debug
{
    public partial class BioDebug : FeedbackProvider<NewBioFeedback>
    {
        [ExportGroup("Board")]
        [Export]
        HBoxContainer VoltagesContainer;
        [Export]
        PackedScene VoltagesDisplay;
        [Export]
        Voltages BioVoltages;

        [ExportGroup("Lance")]
        [Export]
        Label DrillTemp;
        [Export]
        Label DrillHumidity;

        [ExportGroup("")]
        [Export]
        TextureRect GroundContact;
        bool prevContact;

        public override void _Ready()
        {
            base._Ready();

            VisibilityChanged += () =>
            {
                if (!Visible)
                    VoltagesContainer.GetChild(-1).QueueFree();
                else
                {
                    for (int i = VoltagesContainer.GetChildren().Count - 1; i >= 1; i--)
                        VoltagesContainer.GetChild(i).QueueFree();
                    BioVoltages = VoltagesDisplay.Instantiate() as Voltages;
                    VoltagesContainer.AddChild(BioVoltages);
                }
            };
        }

        public override void FeedbackHandler()
        {
            if(!visible)
                return;
            
            BioVoltages.Update(feedback.board_voltage);

            SetLabelText(DrillTemp, $"Drill Temp: {feedback.drill_temp}");
            SetLabelText(DrillHumidity, $"Drill Humidity: {feedback.drill_humidity}");
            GroundContact.SetDeferred(CanvasItem.MethodName.IsVisible, prevContact = feedback.libs_grounded);
        }
    }
    
}