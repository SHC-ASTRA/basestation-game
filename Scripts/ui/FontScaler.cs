using Godot;

namespace UI
{
    public partial class FontScaler : Control
    {
        int baseFontSize;
        Vector2 baseSize;
        Callable c;
        Control par;

        public override void _EnterTree()
        {
            par = GetParent() as Control;
            baseSize = (par.GetParent() as Control).Size;
            baseFontSize = par.GetThemeFontSize("font_size", this.GetClass());
            c = Callable.From(() => Resize());
            this.Connect("resized", c);
            base._EnterTree();
            Resize();
        }

        void Resize()
        {
            float xscale = Size.X - baseSize.X;
            float yscale = Size.Y - baseSize.Y;
            int scaledSize = (int)Mathf.Floor(baseFontSize * xscale <= yscale ? xscale : yscale);

            if (scaledSize > 4096)
                return;

            par.AddThemeFontSizeOverride("font_size", scaledSize);
        }

        public override void _ExitTree()
        {
            this.Disconnect("resized", c);
            base._ExitTree();
        }
    }
}
