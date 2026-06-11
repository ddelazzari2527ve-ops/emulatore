using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace DigitalTwinManager
{
    public sealed class LedIndicator : Panel
    {
        public Color OffColor { get; set; } = Color.DimGray;
        public Color OnColor  { get; set; } = Color.LimeGreen;

        private bool _isActive;
        public bool IsActive
        {
            get => _isActive;
            set { _isActive = value; Invalidate(); }
        }

        public LedIndicator()
        {
            DoubleBuffered = true;
            BackColor      = Color.Transparent;
            Size           = new Size(20, 20);
        }

        protected override void OnResize(EventArgs e)
        {
            base.OnResize(e);
            Width = Height;
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            var g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;

            var color = IsActive ? OnColor : OffColor;
            var rect  = new RectangleF(1.5f, 1.5f, Width - 4, Height - 4);

            // Glow halo when active
            if (IsActive)
            {
                int glowSize = 4;
                var glowRect = new RectangleF(
                    rect.X - glowSize, rect.Y - glowSize,
                    rect.Width  + glowSize * 2,
                    rect.Height + glowSize * 2);
                using var glowBrush = new PathGradientBrush(new[]
                {
                    new PointF(glowRect.Left  + glowRect.Width  / 2f, glowRect.Top),
                    new PointF(glowRect.Right, glowRect.Top    + glowRect.Height / 2f),
                    new PointF(glowRect.Left  + glowRect.Width  / 2f, glowRect.Bottom),
                    new PointF(glowRect.Left,  glowRect.Top    + glowRect.Height / 2f)
                })
                {
                    CenterColor    = Color.FromArgb(80, color),
                    SurroundColors = new[] { Color.Transparent }
                };
                g.FillEllipse(glowBrush, glowRect);
            }

            // Body fill
                using var bodyBrush = new LinearGradientBrush(
                new PointF(rect.Left,  rect.Top),
                new PointF(rect.Right, rect.Bottom),
                    IsActive ? Color.FromArgb(Math.Min(255, color.R + 60),
                                            Math.Min(255, color.G + 60),
                                            Math.Min(255, color.B + 60))
                           : Color.FromArgb(color.R / 2, color.G / 2, color.B / 2),
                color);
            g.FillEllipse(bodyBrush, rect);

            // Specular highlight
            var hiRect = new RectangleF(rect.X + rect.Width * 0.20f,
                                         rect.Y + rect.Height * 0.12f,
                                         rect.Width * 0.40f,
                                         rect.Height * 0.30f);
            using var hiBrush = new SolidBrush(Color.FromArgb(IsActive ? 120 : 40, 255, 255, 255));
            g.FillEllipse(hiBrush, hiRect);

            // Border
            using var borderPen = new Pen(
                IsActive ? Color.FromArgb(180, color) : Color.FromArgb(80, 80, 80), 1f);
            g.DrawEllipse(borderPen, rect);
        }
    }
}
