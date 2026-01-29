using Godot;

public partial class MapShifter : Button
{
    Vector2 bl = new Vector2(-1, -1);
    Vector2 tl = new Vector2(-1, 1);

    public override async void _Pressed()
    {
        Node map = GetParent().GetParent().GetChild(1).GetChild(0).GetChild(0);
        SceneTree tree = GetTree();

        Vector2 originalPosition = new Vector2(map.Get("longitude").As<float>(), map.Get("latitude").As<float>());

        for (int z = 10; z <= 20; z++)
        {
            for (int p = 1; p < 10; p++)
            {
                // Lord have mercy on me for this demonic code...
                float pz = 1 + (float)p / (float)z;
                map.Call("setPosition", originalPosition + bl * pz);
                await ToSignal(tree.CreateTimer(0.1f, false, false, false), SceneTreeTimer.SignalName.Timeout);
                map.Call("setPosition", originalPosition - bl * pz);
                await ToSignal(tree.CreateTimer(0.1f, false, false, false), SceneTreeTimer.SignalName.Timeout);
                map.Call("setPosition", originalPosition + tl * pz);
                await ToSignal(tree.CreateTimer(0.1f, false, false, false), SceneTreeTimer.SignalName.Timeout);
                map.Call("setPosition", originalPosition - tl * pz);
                await ToSignal(tree.CreateTimer(0.1f, false, false, false), SceneTreeTimer.SignalName.Timeout);
                map.Call("setPosition", originalPosition);
                await ToSignal(tree.CreateTimer(0.1f, false, false, false), SceneTreeTimer.SignalName.Timeout);
            }
            map.Call("setPosition", originalPosition);
            map.Call("_on_zoom_changed", z);
            await ToSignal(tree.CreateTimer(0.1f, false, false, false), SceneTreeTimer.SignalName.Timeout);
        }
    }
}
