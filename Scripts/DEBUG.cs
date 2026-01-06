using Godot;
using System.Threading;
using System.Collections;
using System.Collections.Generic;

namespace DEBUG
{
    public static class Debug
    {
        // Using Variant is inherently cringe, but it makes having
        // an anonymous data type a whole lot prettier & faster
        // instead of using <int, string> or elsewise
        private static Dictionary<int, Variant> DebugData = new Dictionary<int, Variant>();

        private static LogEmitter logEmitter = null;

        /// <summary>
        /// Creates a new key in the DebugData array. This key-value is typed
        /// with that of the Variable.CreateFrom(<paramref name="type"/>).
        /// </summary>
        /// <returns> The assigned debug position </returns>
        // This keeps the anonymized data overhead minimal (in my eyes at least)
        public static int RegisterDebugData(Variant type, bool array)
        {
            int r = DebugData.Keys.Count;
            DebugData.Add(r, type);
            if (logEmitter == null)
            {
                logEmitter = new LogEmitter();
                new Thread(logEmitter.init);
            }
            if (array)
                logEmitter.pushStream(type.AsGodotArray());
            return r;
        }

        public static void PullLogs()
        {
            foreach (string s in LogEmitter.outs)
                GD.Print(s);
            LogEmitter.callback = false;
        }

        /// <summary>
        /// Adds data <paramref name="d"/> to Key <paramref name="r"/>.
        /// You better have the right datatype for your key,
        /// or it'll all blow up in your face.
        /// </summary>
        public static void Log(int r, Variant d)
        {
            DebugData[r].AsGodotArray().Add(d);
            if (DebugData[r].AsGodotArray().Count > 1)
            {
                Variant t = DebugData[r].AsGodotArray()[^2];
                if (!t.Equals(d))
                    GD.Print(d);
            }
        }

        /// <summary>
        /// Adds data <paramref name="d"/> to Key <paramref name="r"/>.
        /// You better have the right datatype for your key,
        /// or it'll all blow up in your face.
        /// </summary>
        public static void Log(int? r, Variant d) => DebugData[r.Value].AsGodotArray().Add(d);

        /// <summary>
        /// Copies out the corresponding Variant data from <paramref name="r"/>
        /// Generally safe
        /// </summary>
        /// <returns> Safe copy of DebugData[<paramref name="r"/>] </returns>
        public static Variant GetCopy(int r) => Variant.From(DebugData[r]);

        /// <summary>
        /// Gets the corresponding Variant data from <paramref name="r"/>
        /// Don't use this unless you're the owner of the data or the
        /// class doesn't consume its own logs
        /// </summary>
        /// <returns> Raw DebugData[<paramref name="r"/>] </returns>
        public static Variant GetData(int r) => DebugData[r];

        protected class LogEmitter
        {
            private static List<IEnumerable> logStreams = new();
            public static List<string> outs;

            public static bool callback = false;

            public void init()
            {
                IEnumerable[] clogStreams = new IEnumerable[] { };
                while (true)
                {
                    logStreams.CopyTo(clogStreams, 0);

                    foreach (IEnumerable stream in clogStreams)
                    {
                        foreach (object o in stream)
                        {
                            outs.Add(o.ToString());
                        }
                        callback = true;
                        while (callback)
                            Thread.Sleep(1);
                        Thread.Sleep(5);
                    }
                }
            }

            public int pushStream(IEnumerable i)
            {
                logStreams.Add(i);
                return logStreams.Count - 1;
            }

            public void removeStream(int i)
            {
                logStreams.RemoveAt(i);
            }
        }
    }
}
