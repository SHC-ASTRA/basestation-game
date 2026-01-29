using Godot;
using static Godot.OS;

namespace IPC
{
    public static class CameraSystem
    {
        private static int? cameraPID;

        public static void Init()
        {
            cameraPID ??= CreateProcess("basestation-cameras", []);
        }

        public static void AddStreamSafe(int id, string url = "192.168.1.")
        {
            Init();
            AddStream(id, url);
        }

        public static void AddStream(int id, string url = "192.168.1.")
        {
            GD.Print($"Opening stream ID{id}");
            Execute("bash", ["-c", $"printf '{{\"cmd\":\"add_stream\",\"payload\":{{\"id\":%s,\"url\":%s}}\n' ${id} ${url} | socat - UNIX-CONNECT:/tmp/basestation-cameras-ipc"]);
        }

        public static void RemoveStreamSafe(int id)
        {
            if (cameraPID == null) return;
            RemoveStream(id);
        }

        public static void RemoveStream(int id)
        {
            GD.Print($"Closing stream ID{id}");
            Execute("bash", ["-c", $"printf '{{\"cmd\":\"remove_stream\",\"payload\":{{\"id\":%s}}\n' ${id} | socat - UNIX-CONNECT:/tmp/basestation-cameras-ipc"]);
        }

        /// <summary>
        ///    Makes sure the camera system is running before attempting to resize the grid.
        ///    Wouldn't cause any trouble if it wasn't running, but convenient for the user.
        /// </summary>
        public static void SetGridDimsSafe(byte rows, byte cols)
        {
            if (cameraPID == null) Init();
            SetGridDims(rows, cols);
        }

        /// <summary>
        ///     Sets the camera system's grid dimensions as <paramref name="rows"/> x <paramref name="cols"/>
        /// </summary>
        public static void SetGridDims(byte rows, byte cols)
        {
            GD.Print($"Setting grid size to {rows}x{cols}");
            Execute("bash", ["-c", $"printf '{{\"cmd\":\"grid\",\"payload\":{{\"rows\":%s,\"cols\":%s}}\n' ${rows} ${cols} | socat - UNIX-CONNECT:/tmp/basestation-cameras-ipc"]);
        }
    }
}
