using Godot;

namespace UI.Debug
{
    public partial class Motors : Visibility
    {
        Control MotorContainer;
        [Export]
        public RevMotor FL, BL, FR, BR;

        public override void _Ready()
        {
            FL.name.Text = nameof(FL);
            BL.name.Text = nameof(BL);
            FR.name.Text = nameof(FR);
            BR.name.Text = nameof(BR);

            MotorContainer = FL.GetParent() as Control;
        }

        public void Update(RevMotorState fl, RevMotorState bl, RevMotorState fr, RevMotorState br)
        {
            if (!visible)
                return;
                
            FL.Update(fl);
            BL.Update(bl);
            FR.Update(fr);
            BR.Update(br);
        }
    }
}
