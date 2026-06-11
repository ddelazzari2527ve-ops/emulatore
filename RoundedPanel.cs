using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace DigitalTwinManager
{
    public class RoundedPanel : Panel
    {
        public int CornerRadius { get; set; } = 20;

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            var g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;

            var bounds = new Rectangle(1, 1, Width - 2, Height - 2);
            using var path = GetRoundPath(bounds, CornerRadius);

            // Subtle two-tone border: brighter top-left, dimmer bottom-right
            using var borderPen = new Pen(Color.FromArgb(55, 130, 150, 200), 1.2f);
            g.DrawPath(borderPen, path);
        }

        public static GraphicsPath GetRoundPath(Rectangle bounds, int radius)
        {
            int d    = radius * 2;
            var path = new GraphicsPath();
            path.AddArc(bounds.X,               bounds.Y,                d, d, 180, 90);
            path.AddArc(bounds.Right - d,        bounds.Y,                d, d, 270, 90);
            path.AddArc(bounds.Right - d,        bounds.Bottom - d,       d, d,   0, 90);
            path.AddArc(bounds.X,               bounds.Bottom - d,       d, d,  90, 90);
            path.CloseFigure();
            return path;
        }
    }
}