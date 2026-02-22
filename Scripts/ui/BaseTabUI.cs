using IPC;
using Godot;
using System.Threading;
using System.Threading.Tasks;

namespace UI
{
    // Base tab UI. Abstractly handles ROS functions and provides a controller input interface.
    public abstract partial class BaseTabUI : Control
    {
        // Crazy shit that allows sub-thread ROS emission with pausing and resuming
        public CancellationTokenSource CTS = new();

        // ROS stuff
        [Export]
        public bool ROSDependent;

        ///<summary> EmitToROS called every (Rate)ms, with a one-off delay of (Delay)ms </summary>
        protected int Rate = 150;
        ///<summary> EmitToROS called every (Rate)ms, with a one-off delay of (Delay)ms </summary>
        public int Delay;
        private Task Updater;

        // Controller
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
            // Start or stop the ROS emitter if this node is visible
            this.VisibilityChanged += () => { if (this.Visible) Resume(); else Pause(); };

            // Spool this tab in the ROS ecosystem
            Task.Run(async () =>
            {
                await ROS.AwaitRosReady();
                AdvertiseToROS();
                Updater = new Task(() => Update(CTS.Token), CTS.Token);
                if (CallDeferred("is_visible").AsBool())
                    Updater.Start();
            });

            base._Ready();
        }

        public override void _Process(double delta)
        {
            base._Process(delta);

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
        }

        // Runs ROS emitter async to the main thread, slightly
        // more performant & doesn't run until ROS is ready
        private async void Update(CancellationToken token)
        {
            while (true)
            {
                if (token.IsCancellationRequested)
                    return;
                await Task.Delay(Rate + Delay);
                Delay = 0;
                EmitToROS();
            }
        }

        public async void Pause()
        {
            await CTS.CancelAsync();
            if (Updater != null)
                Updater.Dispose();
        }

        public void Resume()
        {
            CTS.Dispose();
            CTS = new CancellationTokenSource();
            Updater = new Task(() => Update(CTS.Token), CTS.Token);
            Updater.Start();
        }

        /// <summary> Called on Ready, used to advertise associated interfaces to ROS </summary>
        public abstract void AdvertiseToROS();

        /// <summary> Called every (<paramref name="Slow"/>)ms, sends control data to ROS. Can be changed on a per-tab basis </summary>
        public abstract void EmitToROS();

        /// <summary> Called every <paramref name="DebugRate"/> ROS emits, sends logs ROS to console </summary>
        // public abstract string DebugLogROS();

        /// <summary> Cleanup our controls so that clucky doesn't do something stupid while we're offline </summary>
        public override abstract void _ExitTree();
    }
}
