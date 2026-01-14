using Godot;
using System.Collections.Generic;

namespace ui
{
    public class QueueButton
    {
        public readonly int id;

        public QueueButton(int i) => id = i;

        public void call(ref Queue<(int, float)> q, ref SpinBox[] v) =>
            q.Enqueue((id, (float)v[id - 1].Value));

        public void call(ref Queue<(int, int)> q, ref SpinBox[] v) =>
            q.Enqueue((id, (int)v[id - 1].Value));

        public void call(ref Queue<(int, bool)> q, ref Button[] v) =>
            q.Enqueue((id, v[id - 1].ButtonPressed));
    }
}
