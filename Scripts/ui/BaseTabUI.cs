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
        protected int Rate = 120;
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
                Updater = new Task(() => Update(CTS.Token), CTS.Token);
                if (CallDeferred("is_visible").AsBool())
                    Updater.Start();
            });

            base._Ready();
        }

        /// <summary> You MUST call this before doing anything if you intend to use controller inputs
        /// <inheritdoc/> </summary>
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
            while (ROS.run)
            {
                await Task.Delay(Rate + Delay);
                if (token.IsCancellationRequested)
                    return;
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

        /// <summary> Called every (<paramref name="Slow"/>)ms, sends control data to ROS. <paramref name="Slow"/> can
        /// be changed on a per-tab basis.
        /// Aim to reduce the amount of engine-side variables gathered here as possible, and instead store them in the tab.
        /// Doing so reduces the call overhead, as the C# has to do a translation to get data back from the engine.
        /// Do Keep in mind though that with higher <paramref name="Slow"/> values, _Process() will be called
        /// more frequently and thus will have higher overhead than this. Essentially, think about it.</summary>
        public abstract void EmitToROS();

        /// <summary> Cleanup our controls so that clucky doesn't do something stupid while we're offline </summary>
        public override abstract void _ExitTree();
    }
}
