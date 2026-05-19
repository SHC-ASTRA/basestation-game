using Godot;
using static UI.Debug.Debug;

namespace UI.Debug
{
    public partial class IMU : Control
    {
        [Export]
        public Label Orientation, Calibration;
        public void Update(float o, int calib)
        {
            SetLabelText(Orientation, $" Hdng: {o}");
            SetLabelText(Calibration, $" Clbr: {calib}");
        }
    }
}
