using System.Drawing;
using System.Drawing.Drawing2D;
using System.Globalization;
using System.Windows.Forms;

namespace DigitalTwinManager
{
    public sealed class ValueCard : RoundedPanel
    {
        // Card che mostra un valore numerico grande, un titolo e l'unità.
        // Utilizzata nella dashboard dei parametri telemetrici.
        private readonly Label _titleLabel;
        private readonly Label _valueLabel;
        private readonly Label _unitLabel;

        public string Title
        {
            get => _titleLabel.Text;
            set => _titleLabel.Text = value?.ToUpperInvariant() ?? string.Empty;
        }

        public string Value
        {
            get => _valueLabel.Text;
            set
            {
                _valueLabel.Text = FormatValueText(value);
                AdjustFontSize();
            }
        }

        public string Unit
        {
            get => _unitLabel.Text;
            set => _unitLabel.Text = value;
        }

        private Color _accentColor = Color.FromArgb(0x00, 0xD4, 0xFF);
        public Color AccentColor
        {
            get => _accentColor;
            set
            {
                _accentColor  = value;
                _valueLabel.ForeColor = value;
                _unitLabel.ForeColor  = Color.FromArgb(160,
                    Math.Min(255, value.R + 20),
                    Math.Min(255, value.G + 20),
                    Math.Min(255, value.B + 20));
                Invalidate();
            }
        }

        public ValueCard()
        {
            BackColor    = Color.FromArgb(0x2A, 0x2A, 0x2A);
            CornerRadius = 20;
            Padding      = new Padding(14, 12, 14, 10);
            MinimumSize  = new Size(300, 170);
            Size         = new Size(340, 190);

            // Title — small, muted, letter-spaced feel via bold
            _titleLabel = new Label
            {
                AutoSize  = false,
                Dock      = DockStyle.Fill,
                Font      = new Font("Segoe UI", 8F, FontStyle.Bold),
                ForeColor = Color.FromArgb(0x8A, 0x9B, 0xBD),
                TextAlign = ContentAlignment.BottomCenter,
                BackColor = Color.Transparent,
                Padding   = new Padding(0, 0, 0, 2)
            };

            // Value — large, fills centre row
            _valueLabel = new Label
            {
                AutoSize = false,
                Dock     = DockStyle.Fill,
                Font     = new Font("Segoe UI", 48F, FontStyle.Bold),
                ForeColor = _accentColor,
                TextAlign = ContentAlignment.MiddleCenter,
                UseCompatibleTextRendering = true,
                BackColor = Color.Transparent,
                Padding   = new Padding(0)
            };

            // Unit — smaller, below value
            _unitLabel = new Label
            {
                AutoSize  = false,
                Dock      = DockStyle.Fill,
                Font      = new Font("Segoe UI", 10F, FontStyle.Regular),
                ForeColor = Color.FromArgb(160, _accentColor.R, _accentColor.G, _accentColor.B),
                TextAlign = ContentAlignment.TopCenter,
                BackColor = Color.Transparent,
                Padding   = new Padding(0, 2, 0, 0)
            };

            var layout = new TableLayoutPanel
            {
                Dock        = DockStyle.Fill,
                BackColor   = Color.Transparent,
                ColumnCount = 1,
                RowCount    = 3,
                Padding     = new Padding(0),
                Margin      = new Padding(0),
                AutoSize    = false
            };
            layout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
            // 18% title | 62% value | 20% unit
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 18F));
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 62F));
            layout.RowStyles.Add(new RowStyle(SizeType.Percent, 20F));

            layout.Controls.Add(_titleLabel, 0, 0);
            layout.Controls.Add(_valueLabel, 0, 1);
            layout.Controls.Add(_unitLabel,  0, 2);

            Controls.Add(layout);
        }

        protected override void OnResize(EventArgs e)
        {
            base.OnResize(e);
            if (IsHandleCreated && _valueLabel != null)
                AdjustFontSize();
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            var g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;

            // Accent bar at bottom — slightly thicker and rounded caps
            int margin = Padding.Left + 6;
            int left   = margin;
            int right  = Width - margin - 1;
            int y      = Height - 6;

            using var pen = new Pen(_accentColor, 3f) { StartCap = LineCap.Round, EndCap = LineCap.Round };
            g.DrawLine(pen, left, y, right, y);

            // Very faint top-left corner highlight for depth
            using var topBrush = new LinearGradientBrush(
                new Rectangle(0, 0, Width, 40),
                Color.FromArgb(18, 255, 255, 255),
                Color.Transparent,
                LinearGradientMode.Vertical);
            using var topPath = RoundedPanel.GetRoundPath(
                new Rectangle(1, 1, Width - 2, 38), CornerRadius);
            g.FillPath(topBrush, topPath);
        }

        private static string FormatValueText(string text)
        {
            if (decimal.TryParse(text, NumberStyles.Any, CultureInfo.InvariantCulture, out var value))
            {
                return value % 1 == 0
                    ? value.ToString("0",   CultureInfo.InvariantCulture)
                    : value.ToString("0.0", CultureInfo.InvariantCulture);
            }
            return text;
        }

        private void AdjustFontSize()
        {
            if (_valueLabel == null || string.IsNullOrEmpty(_valueLabel.Text)) return;
            try
            {
                const int padX = 10;
                const int padY = 6;

                int availW = _valueLabel.ClientSize.Width  - padX * 2;
                int availH = _valueLabel.ClientSize.Height - padY * 2;
                if (availW <= 0 || availH <= 0) return;

                string text   = _valueLabel.Text;
                var    family = _valueLabel.Font.FontFamily;
                var    style  = _valueLabel.Font.Style;

                float   fontSize = 72f;
                Font?   bestFont = null;

                while (fontSize >= 11f)
                {
                    using var testFont = new Font(family, fontSize, style);
                    var size = TextRenderer.MeasureText(text, testFont,
                        new Size(int.MaxValue, int.MaxValue),
                        TextFormatFlags.SingleLine | TextFormatFlags.NoPadding | TextFormatFlags.NoPrefix);

                    if (size.Width <= availW && size.Height <= availH)
                    {
                        bestFont = new Font(family, fontSize, style);
                        break;
                    }
                    fontSize -= 1.5f;
                }

                bestFont ??= new Font(family, 11f, style);

                if (_valueLabel.Font.Size != bestFont.Size)
                    _valueLabel.Font = bestFont;
                else
                    bestFont.Dispose();
            }
            catch
            {
                _valueLabel.Font = new Font("Segoe UI", 12f, FontStyle.Bold);
            }
        }
    }
}