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
        public bool ROSDependent, ShouldEmit;
        public bool Started;

        ///<summary> EmitToROS called every (Rate)ms, with a one-off delay of (Delay)ms </summary>
        protected int Rate = 180;
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

        /// <summary> You MUST call this before doing anything if you intend to use controller inputs on (this) frame
        ///  <inheritdoc/> </summary>
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
            while (ROS.ROSShouldBeUp)
            {
                try { await Task.Delay(Rate + Delay, token); }
                catch { Delay = 0; return; }
                Delay = 0;
                EmitToROS();
            }
        }

        public async void Pause()
        {
            await CTS.CancelAsync();
            Updater?.Dispose();
        }

        public void Resume()
        {
            if (!ShouldEmit)
                return;
            CTS?.Dispose();
            CTS = new CancellationTokenSource();
            Task.Run(() =>
            {
                // Uses a globally dynamic delay so that topics cannot
                // all advertise at once, which causes ROSBridge to fail
                ROS.RegsiterOnROSStart(() =>
                {
                    while (requestDelay < 10) Thread.Sleep(requestDelay++);
                    requestDelay = 1;
                    if (CTS.Token.IsCancellationRequested)
                    {
                        CTS?.Dispose();
                        CTS = new CancellationTokenSource();
                    }
                    Updater = new(() => Update(CTS.Token), CTS.Token);
                    Updater.Start();
                });
            });

        }
        static int requestDelay = 1;

        /// <summary> Called every (<paramref name="Slow"/>)ms, sends control data to ROS. <paramref name="Slow"/> can
        /// be changed on a per-tab basis.
        /// Aim to reduce the amount of engine-side variables gathered here as possible, and instead store them in the tab.
        /// Doing so reduces the call overhead, as the C# has to do a translation to get data back from the engine.
        /// Do Keep in mind though that with higher <paramref name="Slow"/> values, _Process() will be called
        /// more frequently and thus will have higher overhead than this. Essentially, think about it.</summary>
        public abstract void EmitToROS();

        /// <summary> Called when Tab is selected and on program startup </summary>
        public abstract void STARTTAB();

        /// <summary> Send zeroes for all control values on leaving tab </summary>
        public abstract void STOPTAB();

        /// <summary> Cleanup our controls so that clucky doesn't do something stupid while we're offline </summary>
        public override void _ExitTree() => STOPTAB();
    }
}
