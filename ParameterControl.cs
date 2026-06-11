
using System;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Windows.Forms;

namespace DigitalTwinManager
{
    public sealed class ParameterControl : RoundedPanel
    {
                // Controllo custom per impostare un parametro numerico.
                // Include slider, NumericUpDown, titolo, descrizione e unità.

                // ── Public surface ────────────────────────────────────────────────
                public TrackBar      Track   { get; private set; }
                public NumericUpDown Numeric { get; private set; }
                public event EventHandler? ValueChanged;

                private readonly Label _titleLabel;
                private readonly Label _descLabel;
                private readonly Label _unitLabel;

                private Color   _accentColor = Color.FromArgb(0x00, 0xB4, 0xD8);
                private decimal _minValue;
                private decimal _maxValue;
                private int     _scale = 1;

                // ── Layout constants ──────────────────────────────────────────────
                private const int PAD_H     = 14;
                private const int PAD_V     = 11;
                private const int NUMERIC_W = 96;
                private const int NUMERIC_H = 28;
                private const int TRACK_H   = 20;
                private const int ACCENT_W  =  4;
                private const int ACCENT_R  = ACCENT_W + 8;

                // Highlight flash
                private readonly System.Windows.Forms.Timer _hlTimer = new();
                private bool _highlighted;

                // ── Properties ───────────────────────────────────────────────────
                public string Title
                {
                    get => _titleLabel.Text;
                    set => _titleLabel.Text = value;
                }

                public string Description
                {
                    get => _descLabel.Text;
                    set => _descLabel.Text = value;
                }

                public string Unit
                {
                    get => _unitLabel.Text;
                    set => _unitLabel.Text = value;
                }

                public decimal Minimum
                {
                    get => _minValue;
                    set { _minValue = value; Numeric.Minimum = value; UpdateTrackRange(); }
                }

                public decimal Maximum
                {
                    get => _maxValue;
                    set { _maxValue = value; Numeric.Maximum = value; UpdateTrackRange(); }
                }

                public decimal Value
                {
                    get => Numeric.Value;
                    set
                    {
                        value = Math.Max(Minimum, Math.Min(Maximum, value));
                        if (Numeric.Value == value) return;
                        Numeric.Value = value;
                        Track.Value   = Clamp(ToScaledInt(value));
                    }
                }

                public int DecimalPlaces
                {
                    get => Numeric.DecimalPlaces;
                    set
                    {
                        Numeric.DecimalPlaces = value;
                        Numeric.Increment     = (decimal)Math.Pow(0.1, value);
                        UpdateTrackRange();
                    }
                }

                // ── Constructor ───────────────────────────────────────────────────
                public ParameterControl()
                {
                    CornerRadius = 14;
                    BackColor    = Color.FromArgb(0x16, 0x21, 0x3E);
                    MinimumSize  = new Size(320, 130);
                    AutoSize     = true;
                    AutoSizeMode = AutoSizeMode.GrowOnly;
                    Padding      = new Padding(ACCENT_R + PAD_H, PAD_V, PAD_H, PAD_V);

                    // ── Title label ──────────────────────────────────────────────
                    _titleLabel = new Label
                    {
                        AutoSize     = false,
                        Dock         = DockStyle.Top,
                        Height       = 20,
                        Font         = new Font("Segoe UI", 9F, FontStyle.Bold),
                        ForeColor    = Color.FromArgb(0xD8, 0xEA, 0xFF),
                        TextAlign    = ContentAlignment.MiddleLeft,
                        BackColor    = Color.Transparent,
                        AutoEllipsis = false,
                        Padding      = new Padding(0)
                    };

                    // ── Description label ────────────────────────────────────────
                    _descLabel = new Label
                    {
                        AutoSize  = true,
                        Dock      = DockStyle.Top,
                        Font      = new Font("Segoe UI", 7F, FontStyle.Italic),
                        ForeColor = Color.FromArgb(0x5A, 0x6E, 0x99),
                        BackColor = Color.Transparent,
                        Padding   = new Padding(0, 1, 0, 3)
                    };

                    // ── Track bar ────────────────────────────────────────────────
                    Track = new TrackBar
                    {
                        TickStyle = TickStyle.None,
                        Height    = TRACK_H,
                        Dock      = DockStyle.Top,
                        BackColor = Color.FromArgb(0x16, 0x21, 0x3E),
                        Margin    = new Padding(0, 2, 0, 0)
                    };

                    // ── Bottom row: numeric + unit ────────────────────────────────
                    var bottomRow = new TableLayoutPanel
                    {
                        Dock        = DockStyle.Top,
                        Height      = NUMERIC_H + 6,
                        ColumnCount = 3,
                        RowCount    = 1,
                        BackColor   = Color.Transparent,
                        Margin      = new Padding(0, 4, 0, 0),
                        Padding     = new Padding(0)
                    };
                    bottomRow.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
                    bottomRow.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, NUMERIC_W));
                    bottomRow.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 48));

                    Numeric = new NumericUpDown
                    {
                        Dock        = DockStyle.Fill,
                        TextAlign   = HorizontalAlignment.Right,
                        Font        = new Font("Segoe UI", 10F, FontStyle.Bold),
                        BackColor   = Color.FromArgb(0x0A, 0x11, 0x22),
                        ForeColor   = Color.FromArgb(0x48, 0xCA, 0xE4),
                        BorderStyle = BorderStyle.FixedSingle
                    };

                    _unitLabel = new Label
                    {
                        AutoSize  = false,
                        Dock      = DockStyle.Fill,
                        Font      = new Font("Segoe UI", 8F, FontStyle.Bold),
                        ForeColor = Color.FromArgb(0x44, 0x55, 0x77),
                        TextAlign = ContentAlignment.MiddleLeft,
                        Padding   = new Padding(5, 0, 0, 0),
                        BackColor = Color.Transparent
                    };

                    bottomRow.Controls.Add(new Label { BackColor = Color.Transparent }, 0, 0); // spacer
                    bottomRow.Controls.Add(Numeric,    1, 0);
                    bottomRow.Controls.Add(_unitLabel, 2, 0);

                    // ── Wire events ───────────────────────────────────────────────
                    _hlTimer.Interval = 450;
                    _hlTimer.Tick    += (_, _) => { _highlighted = false; _hlTimer.Stop(); Invalidate(); };

                    Track.Scroll += (_, _) =>
                    {
                        var v = ToUnscaledValue(Track.Value);
                        if (Numeric.Value == v) return;
                        Numeric.Value = v;
                        Flash();
                    };

                    Numeric.ValueChanged += (_, _) =>
                    {
                        int t = ToScaledInt(Numeric.Value);
                        if (Track.Value != t) Track.Value = t;
                        ValueChanged?.Invoke(this, EventArgs.Empty);
                        Flash();
                    };

                    // Add children (reverse DockStyle.Top stacking order)
                    Controls.Add(bottomRow);
                    Controls.Add(Track);
                    Controls.Add(_descLabel);
                    Controls.Add(_titleLabel);
                }

                // ── Painting ──────────────────────────────────────────────────────
                protected override void OnPaintBackground(PaintEventArgs e)
                {
                    var g  = e.Graphics;
                    g.SmoothingMode = SmoothingMode.AntiAlias;
                    var rc = ClientRectangle;

                    // Background fill
                    Color bg = _highlighted
                        ? Color.FromArgb(0x22, 0x18, 0x44)
                        : BackColor;
                    using var bgBrush = new SolidBrush(bg);
                    using var path    = RoundedPanel.GetRoundPath(
                        new Rectangle(0, 0, rc.Width - 1, rc.Height - 1), CornerRadius);
                    g.FillPath(bgBrush, path);

                    // Left accent bar — gradient fade bottom
                    int barTop = CornerRadius / 2;
                    int barH   = rc.Height - CornerRadius;
                    using var accentBrush = new LinearGradientBrush(
                        new Rectangle(0, barTop, ACCENT_W, Math.Max(1, barH)),
                        Color.FromArgb(210, _accentColor),
                        Color.FromArgb(40,  _accentColor),
                        LinearGradientMode.Vertical);
                    using var accentPath = new GraphicsPath();
                    accentPath.AddRoundedRectangle(
                        new RectangleF(ACCENT_W - 1, barTop, ACCENT_W, barH), 2);
                    g.FillPath(accentBrush, accentPath);

                    // Outer border
                    using var borderPen = new Pen(
                        _highlighted
                            ? Color.FromArgb(80, _accentColor)
                            : Color.FromArgb(30, 0xAA, 0xBB, 0xFF), 1f);
                    g.DrawPath(borderPen, path);

                    // Subtle inner top-highlight line for depth
                    using var shimPen = new Pen(Color.FromArgb(18, 255, 255, 255), 1f);
                    g.DrawLine(shimPen, CornerRadius, 1, rc.Width - CornerRadius, 1);
                }

                protected override void OnPaint(PaintEventArgs e) { /* suppress base border */ }

                // ── Accent color (public, used by BuildParametersPanel) ───────────
                public Color AccentColor
                {
                    get => _accentColor;
                    set
                    {
                        _accentColor = value;
                        _titleLabel.ForeColor = Color.FromArgb(
                            Math.Min(255, value.R + 80),
                            Math.Min(255, value.G + 80),
                            Math.Min(255, value.B + 80));
                        Invalidate();
                    }
                }

                // ── Helpers ───────────────────────────────────────────────────────
                private void Flash()
                {
                    _highlighted = true;
                    _hlTimer.Stop();
                    _hlTimer.Start();
                    Invalidate();
                }

                private void UpdateTrackRange()
                {
                    _scale = (int)Math.Pow(10, Numeric.DecimalPlaces);
                    if (_scale < 1) _scale = 1;
                    int scaledMin = (int)Math.Round(_minValue * _scale);
                    int scaledMax = (int)Math.Round(_maxValue * _scale);
                    Track.Minimum     = scaledMin;
                    Track.Maximum     = scaledMax;
                    Track.SmallChange = 1;
                    Track.LargeChange = Math.Max(1, (scaledMax - scaledMin) / 10);
                    Track.Value       = Clamp(ToScaledInt(Numeric.Value));
                }

                private int     ToScaledInt(decimal value)
                    => Clamp((int)Math.Round((value - _minValue) * _scale));

                private decimal ToUnscaledValue(int scaledValue)
                    => Math.Round(_minValue + scaledValue / (decimal)_scale, Numeric.DecimalPlaces);

                private int Clamp(int v)
                    => Math.Max(Track.Minimum, Math.Min(Track.Maximum, v));
            }

            // ── Extension helper ──────────────────────────────────────────────────
            internal static class GraphicsPathExtensions
            {
                public static void AddRoundedRectangle(this GraphicsPath path, RectangleF rect, float r)
                {
                    float d = r * 2f;
                    path.AddArc(rect.X,               rect.Y,                d, d, 180, 90);
                    path.AddArc(rect.Right - d,        rect.Y,                d, d, 270, 90);
                    path.AddArc(rect.Right - d,        rect.Bottom - d,       d, d,   0, 90);
                    path.AddArc(rect.X,               rect.Bottom - d,       d, d,  90, 90);
                    path.CloseFigure();
                }
            }
}