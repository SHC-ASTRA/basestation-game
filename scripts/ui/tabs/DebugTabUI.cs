using Godot;
using System.Collections.Generic;

namespace ui
{
    public partial class DebugTabUI : BaseTabUI
    {

    }
}

namespace DEBUG
{
    public static class Debug
    {
        // Using Variant is inherently cringe, but it makes having
        // an anonymous data type a whole lot prettier & faster
        // instead of using <int, string> or elsewise
        private static Dictionary<int, Variant> DebugData = new Dictionary<int, Variant>();

        /// <summary>
        /// Creates a new key in the DebugData array. This key-value is typed
        /// with that of the Variable.CreateFrom(<paramref name="type"/>).
        /// </summary>
        /// <returns> The assigned debug position </returns>
        // This keeps the anonymized data overhead minimal (in my eyes at least)
        public static int RegisterDebugData(Variant type)
        {
            int r = DebugData.Keys.Count;
            DebugData.Add(r, type);
            return r;
        }

        /// <summary>
        /// Adds data <paramref name="d"/> to Key <paramref name="r"/>.
        /// You better have the right datatype for your key,
        /// or it'll all blow up in your face.
        /// </summary>
        public static void Log(int r, Variant d) => DebugData[r].AsGodotArray().Add(d);

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
    }
}
