using IPC;
using Godot;
using DEBUG;

namespace ui
{
    public abstract partial class BaseTabUI : Control
    {
        [Export]
        public string TopicName;

        [Export]
        public bool ROSDependent;

        protected double Slow;

        protected static Vector2 WindowSize;

        protected static int? DebugID;

        protected static bool UpButton, DownButton, LeftButton, RightButton;

        protected static bool StartButton, BackButton;

        protected static bool LeftStickPress, RightStickPress;

        protected static float AButton, BButton, XButton, YButton;

        protected static float BlackButton, WhiteButton;

        protected static float LeftTrigger, RightTrigger;

        protected static float LeftBumper, RightBumper;

        protected static Vector2 LeftStick, RightStick;

        // Left bumper slows down driving, right bumper speeds it up
        // 'A' button (south) enables brake mode
        // Joysticks have deadzones (0.05) so the motors don't sound angry when it's sitting still
        // Controller rumbles when headless is loaded and ready for control
        // Default driving speed should be closer to walking speed now, pending testing
        // Turning inverts when going backwards to make reversing more intuitive (kinda jank rn tho)
        // Turning follows curve (angular.z ** 3) to help with minor adjustments when driving in straight line

        public override void _Ready()
        {
            DebugID = Debug.RegisterDebugData(Variant.From<string>(""), false);

            AdvertiseToROS();
        }

        public override void _Process(double delta)
        {
            WindowSize = GetWindow().Size;

            UpButton = Input.IsActionJustPressed("UpBtn");
            DownButton = Input.IsActionJustPressed("DownBtn");
            LeftButton = Input.IsActionJustPressed("LeftBtn");
            RightButton = Input.IsActionJustPressed("RightBtn");

            StartButton = Input.IsActionJustPressed("StartBtn");
            BackButton = Input.IsActionJustPressed("BackBtn");

            LeftStickPress = Input.IsActionJustPressed("LStickPress");
            RightStickPress = Input.IsActionJustPressed("RStickPress");

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
            if (Slow < 0.004 || !ROS.ROSReady)
                return;
            else Slow = 0;

            EmitToROS();
        }

        public abstract void AdvertiseToROS();
        public abstract void EmitToROS();
    }
}
