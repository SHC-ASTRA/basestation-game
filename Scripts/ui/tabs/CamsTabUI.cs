using Godot;
using System.IO;
using System.Linq;
using System.Threading;
using System.Net.Sockets;
using System.Threading.Tasks;
using System.Collections.Generic;

namespace UI
{
    public partial class CamsTabUI : BaseTabUI
    {
        [Export]
        VBoxContainer Streams, Grid;

        [Export]
        MenuButton Mode;
        int mode = 0;

        [Export]
        public LineEdit ID;

        [Export]
        public SpinBox Rows, Columns;

        [Export]
        public Button Commit;
        [Export]
        public TextureRect Committing;

        [Export]
        public TextEdit ActiveCams;
        [Export]
        public Button Query;

        private static Socket Sock;

        private HashSet<string> CamTable = [];

        public override void _Ready()
        {
            PopupMenu p = Mode.GetPopup();
            p.IndexPressed += (id) =>
            {
                switch (id)
                {
                    case 0:
                        Streams.Visible = true;
                        Grid.Visible = false;
                        break;
                    case 1:
                        Streams.Visible = false;
                        Grid.Visible = true;
                        break;
                }
                Mode.Text = p.GetItemText((int)id);
                mode = (int)id;
            };
            Commit.Pressed += () =>
            {
                string payload = "";
                string idText = ID.Text;
                switch (mode)
                {
                    case 0: // Add Stream
                        payload = $"launch {idText}";
                        if (CamTable.Contains(idText))
                        {
                            GD.Print("Cannot add stream, ID already in camera table!");
                            return;
                        }
                        break;
                    case 1: // Grid
                        payload = $"grid {Rows.Value} {Columns.Value}";
                        break;
                }
                OS.CreateProcess("bash", ["-c", $"cameracli {payload}"]);
                // Sock.Send(Encoding.UTF8.GetBytes(payload));
                Committing.Visible = true;
            }
                        ;
            Commit.ButtonUp += () => Committing.Visible = false;
            Query.Pressed += async () =>
            {
                lock (CamTable)
                {
                    CamTable = ["Active Streams:\n"];
                    CamTable = QueryCams().Result.AsEnumerable() as HashSet<string>;
                }
                ActiveCams.Text = CamTable.Aggregate((a, b) =>
                {
                    return $"{a}\n{b}";
                });
            };
        }

        public static async Task<string[]> QueryCams()
        {
            OS.CreateProcess("bash", ["-c", "cameracli list > /tmp/cameralist"]);
            await Task.Delay(50);
            // Silly little aggregator statement takes the first two characters off each string in the read-in array
            return File.ReadAllLines("/tmp/cameralist")[..][2..];
        }

        public override void EmitToROS() { }

        private static bool tried = false;
        public override void STARTTAB()
        {
        retry:
            try
            {
                Sock = new Socket(AddressFamily.Unix, SocketType.Stream, ProtocolType.Unspecified);
                Sock.Connect(new UnixDomainSocketEndPoint("/tmp/basestation-cameras-ipc"));
            }
            catch
            {
                if (!tried)
                {
                    Sock.Dispose();
                    GD.PushWarning("Failed to open the IPC stream, attempting to open basestation-cameras?");
                    OS.CreateProcess("bash", ["-c", "basestation-cameras"]);
                    Thread.Sleep(250);
                    tried = true;
                    goto retry;
                }
                else GD.PushError("Failed to open the Basestation cameras and/or the IPC stream stream.");
            }
        }
        public override void STOPTAB() { }
    }
}
