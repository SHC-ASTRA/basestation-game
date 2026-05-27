using Godot;
using System.Linq;
using System.Text;
using System.Net.Sockets;
using System.Threading.Tasks;
using System.Collections.Generic;
using System.IO;

namespace UI
{
	public partial class CamsTabUI : BaseTabUI
	{
		[Export]
		VBoxContainer Streams, Addstream, Grid;

		[Export]
		MenuButton Mode;
		int mode = 0;

		[Export]
		public LineEdit ID, Address;

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
			try
			{
				Sock = new Socket(AddressFamily.Unix, SocketType.Stream, ProtocolType.Unspecified);
				Sock.Connect(new UnixDomainSocketEndPoint("/tmp/basestation-cameras-ipc"));
			}
			catch { GD.PushError("Failed to open the IPC stream"); }

			PopupMenu p = Mode.GetPopup();
			p.IndexPressed += (id) =>
			{
				switch (id)
				{
					case 0:
						Streams.Visible = true;
						Addstream.Visible = true;
						Grid.Visible = false;
						break;
					case 1:
						Streams.Visible = true;
						Addstream.Visible = false;
						Grid.Visible = false;
						break;
					case 2:
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
						payload = $"{{\"cmd\":\"add_stream\",\"payload\":{{\"id\":\"{idText}\",\"url\":\"rtsp://{Address.Text}\"}}";
						if (CamTable.Contains(idText))
						{
							GD.Print("Cannot add stream, ID already in camera table!");
							return;
						}
						break;
					case 1: // Remove Stream
						payload = $"{{\"cmd\":\"remove_stream\",\"payload\":{{\"id\":\"{idText}\"}}";
						if (!CamTable.Contains(idText))
						{
							GD.Print("Cannot remove stream, not in Camera table!");
							return;
						}
						break;
					case 2: // Grid
						payload = $"{{\"cmd\":\"grid\",\"payload\":{{\"rows\":{Rows.Value},\"cols\":{Columns.Value}}}";
						break;
				}
				Sock.Send(Encoding.UTF8.GetBytes(payload));
				Committing.Visible = true;
			};
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
			// Silly little aggregator statement takes the first two characters off each string in the read-in array
			return File.ReadAllLines("/tmp/cameralist")[..][2..];
		}

		public override bool AdvertiseToROS() { return false; }
		public override void EmitToROS() { }
		public override void _ExitTree() { }
	}
}
