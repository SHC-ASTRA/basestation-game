using Godot;

namespace UI.Debug
{
    public partial class IMU : Control
    {
        [Export]
        public Label Orientation, Calibration;
        public void Update(float o, int calib)
        {
            Orientation.Text = $" Hdng: {o}";
            Calibration.Text = $" Clbr: {calib}";
        }
    }
}
