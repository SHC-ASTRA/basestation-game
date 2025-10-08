using Godot;
namespace ui
{
	public partial class CoreTabUI : BaseTabUI
	{
		public const bool TankDriving = true;

		[Export]
		public ProgressBar LMotor, RMotor;

		public override void _Ready()
		{
			base._Ready();
		}

		public override void _Process(double delta)
		{
			base._Process(delta);

			if (TankDriving)
			{
				LMotor.Value = leftStick.Y;
				RMotor.Value = rightStick.Y;
			}
			else
			{
				double MotorDrive = leftStick.Y;
				LMotor.Value = MotorDrive + rightStick.X;
				RMotor.Value = MotorDrive - rightStick.X;
			}
		}
	}
}
