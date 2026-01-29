using IPC;
using Godot;
using System.Threading.Tasks;

namespace ui
{
    // Base tab UI. Han
    public abstract partial class BaseTabUI : Control
    {
        [Export]
        public bool ROSDependent;

        protected double Slow;

        protected static Vector2 WindowSize;

        protected static bool UpButtonDown, DownButtonDown, LeftButtonDown, RightButtonDown;
        protected static bool UpButton, DownButton, LeftButton, RightButton;

        protected static bool StartButton, BackButton;

        protected static bool LeftStickPress, RightStickPress;

        protected static float AButton, BButton, XButton, YButton;

        protected static float BlackButton, WhiteButton;

        protected static float LeftTrigger, RightTrigger;

        protected static float LeftBumper, RightBumper;

        protected static Vector2 LeftStick, RightStick;

        public override void _Ready()
        {
            Task.Run(async () =>
            {
                await ROS.AwaitRosReady();
                AdvertiseToROS();
            });
        }

        public override void _Process(double delta)
        {
            base._Process(delta);

            WindowSize = GetWindow().Size;

            UpButtonDown = Input.IsActionJustPressed("UpBtn");
            DownButtonDown = Input.IsActionJustPressed("DownBtn");
            LeftButtonDown = Input.IsActionJustPressed("LeftBtn");
            RightButtonDown = Input.IsActionJustPressed("RightBtn");

            UpButton = Input.IsActionPressed("UpBtn");
            DownButton = Input.IsActionPressed("DownBtn");
            LeftButton = Input.IsActionPressed("LeftBtn");
            RightButton = Input.IsActionPressed("RightBtn");

            StartButton = Input.IsActionJustPressed("StartBtn");
            BackButton = Input.IsActionJustPressed("BackBtn");

            LeftStickPress = Input.IsActionPressed("LStickPress");
            RightStickPress = Input.IsActionPressed("RStickPress");

            AButton = Input.GetActionStrength("ABtn");
            BButton = Input.GetActionStrength("BBtn");
            XButton = Input.GetActionStrength("XBtn");
            YButton = Input.GetActionStrength("YBtn");

            BlackButton = Input.GetActionStrength("BlackBtn");
            WhiteButton = Input.GetActionStrength("WhiteBtn");

            LeftTrigger = Input.GetActionStrength("LTrigger");
            RightTrigger = Input.GetActionStrength("RTrigger");

            LeftBumper = Input.GetActionStrength("LBumper");
            RightBumper = Input.GetActionStrength("RBumper");

            LeftStick = Input.GetVector("LStickHN", "LStickHP", "LStickVN", "LStickVP");
            RightStick = Input.GetVector("RStickHN", "RStickHP", "RStickVN", "RStickVP");

            Slow += delta;
            if (Slow < 0.05 || !ROS.ROSReady)
                return;
            else Slow = 0;

            EmitToROS();
        }

        public abstract void AdvertiseToROS();
        public abstract void EmitToROS();

        public override abstract void _ExitTree();
    }
}
