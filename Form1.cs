using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace DigitalTwinManager
{
    // Questo namespace contiene l'applicazione desktop per la gestione del digital twin.
    // DigitalTwinManager include la form principale, i controlli personalizzati
    // e la logica di traduzione per l'interfaccia utente.
    
    internal static class Program
    {
        // Punto di avvio dell'applicazione Windows Forms.
        // Configura DPI, rendering, gestione eccezioni e poi esegue la form principale.
        
        [STAThread]
        private static void Main()
        {
            Application.SetHighDpiMode(HighDpiMode.PerMonitorV2);
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.SetUnhandledExceptionMode(UnhandledExceptionMode.CatchException);
            Application.ThreadException += (_, e) => LogException(e.Exception, "ThreadException");
            AppDomain.CurrentDomain.UnhandledException += (_, e) => LogException(e.ExceptionObject as Exception, "UnhandledException");
            try
            {
                Application.Run(new MainForm());
            }
            catch (Exception ex)
            {
                LogException(ex, "StartupException");
                throw;
            }
        }

        private static void LogException(Exception? ex, string type)
        {
            try
            {
                var path = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "startup_error.log");
                File.WriteAllText(path, $"{type}: {ex?.ToString() ?? "null"}\r\n");
            }
            catch { }
        }
    }

    public sealed class MainForm : Form
    {
        // ---------- CORE ----------
        // La MainForm contiene tutta la UI moderna, la connessione seriale,
        // la lettura dei dati JSON, l'aggiornamento delle schede dashboard
        // e l'invio dei comandi verso il dispositivo remoto.

        private SerialPort? _serial;
        private readonly System.Windows.Forms.Timer _readTimer = new System.Windows.Forms.Timer();
        private readonly StringBuilder _rxBuffer = new StringBuilder();
        private readonly Dictionary<ParameterControl, string> _parameterDescriptionTranslationKeys = new();
        private bool _uiReady;
        private bool _isOilMode;
        private bool _connected;
        private bool _suppressParamEvents = false;

        private readonly JsonSerializerOptions _jsonOptions = new JsonSerializerOptions
        {
            PropertyNameCaseInsensitive = true,
            NumberHandling = JsonNumberHandling.AllowReadingFromString
        };

        private readonly Color _bg       = Color.FromArgb(0x1A, 0x1A, 0x2E);
        private readonly Color _panel    = Color.FromArgb(0x16, 0x21, 0x3E);
        private readonly Color _panel2   = Color.FromArgb(0x0F, 0x17, 0x2A);
        private readonly Color _card     = Color.FromArgb(0x1E, 0x2A, 0x45);
        private readonly Color _accentBlue   = Color.FromArgb(0x00, 0xB4, 0xD8);
        private readonly Color _accentCyan   = Color.FromArgb(0x48, 0xCA, 0xE4);
        private readonly Color _accentOrange = Color.FromArgb(0xFF, 0x9F, 0x1C);
        private readonly Color _accentRed    = Color.FromArgb(0xFF, 0x4D, 0x4D);
        private readonly Color _accentGreen  = Color.FromArgb(0x06, 0xD6, 0x6A);
        private readonly Color _accentPurple = Color.FromArgb(0xAA, 0x66, 0xFF);
        private readonly Color _accentTeal   = Color.FromArgb(0x26, 0xD0, 0xC2);
        private readonly Color _text  = Color.FromArgb(0xE8, 0xF1, 0xFF);
        private readonly Color _muted = Color.FromArgb(0x8A, 0x9B, 0xBD);

        // Left column (UNTOUCHED)
        private readonly ComboBox cmbPorts = new ComboBox();
        private readonly ComboBox cmbDevice = new ComboBox();
        private readonly TextBox txtLog = new TextBox();
        private Button btnConnect = null!;
        private Button btnDisconnect = null!;
        private Button btnScan = null!;
        private Label lblConnectionState = new Label();
        private Label lblFluidState = new Label();

        // Dashboard cards  — keys = those used in ApplyTelemetry
        private readonly Dictionary<string, ValueCard> _dashboardCards = new Dictionary<string, ValueCard>();

        // Parameters
        private readonly Dictionary<string, ParameterControl> _paramControls = new Dictionary<string, ParameterControl>();

        // LED Alarms
        private readonly Dictionary<int, LedIndicator> _alarmLeds = new Dictionary<int, LedIndicator>();
        private readonly string[] _alarmNameKeys =
        {
            "AlarmBoiling", "AlarmDry", "AlarmPumpFailure", "AlarmThermalShock",
            "AlarmCirculation", "AlarmEmptyTank", "AlarmCavitation", "AlarmOutletSensor",
            "AlarmResistorOverheat", "AlarmHighPressure", "AlarmAirInCircuit", "AlarmWaterLoss"
        };

        private Panel _activeAlarmPanel = null!;
        private LedIndicator _statusPumpLed  = null!;
        private LedIndicator _statusColdLed  = null!;
        private LedIndicator _statusFillLed  = null!;
        private Label _statusPumpLabel  = null!;
        private Label _statusColdLabel  = null!;
        private Label _statusFillLabel  = null!;
        private Label _clockLabel = null!;
        private Label _footerConnectionLabel  = null!;
        private Label _footerHandshakeLabel   = null!;
        private Label _headerConnectionStateLabel = null!;
        private Label _headerPortLabel = null!;
        private Label _headerHandshakeLabel = null!;
        private Label _lastJsonErrorLabel = null!;
        private string _handshakeStatus = "NONE";

        private Language _language = Language.Italian;
        private readonly Dictionary<Label, string> _labelTranslationKeys = new();
        private readonly Dictionary<Button, string> _buttonTranslationKeys = new();
        private readonly Dictionary<ValueCard, string> _cardTranslationKeys = new();
        private readonly Dictionary<ParameterControl, string> _parameterTranslationKeys = new();
        private readonly Dictionary<CheckBox, (string Command, string Key, Color CheckedColor)> _faultTranslationKeys = new();

        private Button btnLanguageToggle = null!;

        // Action buttons
        private Button btnPompaAuto      = null!;
        private Button btnPompaOn        = null!;
        private Button btnPompaOff       = null!;
        private Button btnRaffreddamentoOn  = null!;
        private Button btnRaffreddamentoOff = null!;
        private Button btnSwitchFluid    = null!;
        private Button btnWater          = null!;
        private Button btnLoss           = null!;
        private Button btnReset          = null!;
        private Button btnPumpWater      = null!;
        private Button btnStato          = null!;
        private Button btnMenu           = null!;
        private CheckBox[] faultCheckBoxes = null!;

        // Console
        private TextBox txtCommand      = null!;
        private Button btnSendCommand   = null!;
        // === NEW FIELDS FOR ADDED PARAMETERS ===
        private ParameterControl pcMassaTubi = null!;
        private ParameterControl pcEfficienzaPompa = null!;

        private Label lblManutenzioneInfo = null!;
        private Label lblOrePompa = null!;
        private Label lblOreResistenza = null!;
        private Label lblOreHx = null!;
        private Label lblUsuraPompa = null!;

        // Maintenance (optional but recommended)
        // UI Views
        private Panel _contentPanel    = null!;
        private Panel _dashboardView   = null!;
        private Panel _parametersView  = null!;
        private Panel _alarmsView      = null!;
        private Panel _actionsView     = null!;
        private Panel _consoleView     = null!;

        // Sidebar nav buttons (to manage active state)
        private Button? _activeNavBtn;
    
        public MainForm()
        {
            // Costruttore principale: inizializza l'interfaccia moderna,
            // carica le porte seriali disponibili e avvia il timer di lettura.
            var logPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "debug_startup.log");
            File.AppendAllText(logPath, "MainForm constructor start\r\n");
            InitializeModernUI();
            File.AppendAllText(logPath, "InitializeModernUI completed\r\n");
            ScanPorts();
            _readTimer.Interval = 100;
            _readTimer.Tick += (_, _) => ReadIncoming();
            File.AppendAllText(logPath, "MainForm constructor end\r\n");
        }

        // ---------- ORIGINAL UNALTERED METHODS ----------
        private async Task<bool> PerformHandshakeAsync(int timeoutMs = 4000)
        {
            // Esegue il handshake con il dispositivo remoto inviando NOME?
            // Attende la risposta e controlla se contiene il nome atteso.
            if (_serial == null || !_serial.IsOpen) { Log(T("LogHandshakePortNotOpen")); return false; }
            string expected = GetExpectedNameForDevice(cmbDevice.SelectedIndex);
            _serial.NewLine = "\n";
            for (int i = 0; i < 3; i++) { _serial.DiscardInBuffer(); _serial.DiscardOutBuffer(); await Task.Delay(50); }
            Log(T("LogSendingHandshake"));
            try
            {
                using var cts = new System.Threading.CancellationTokenSource(timeoutMs);
                string response = await Task.Run(async () =>
                {
                    _serial.ReadTimeout = timeoutMs;
                    for (int attempt = 0; attempt < 3; attempt++)
                    {
                        _serial.WriteLine("NOME?");
                        _serial.BaseStream.Flush();
                        try
                        {
                            string line = _serial.ReadLine()?.Trim() ?? string.Empty;
                            if (!string.IsNullOrEmpty(line))
                            {
                                Log(string.Format(T("LogResponseReceived"), line));
                                if (line.ToUpperInvariant().Contains(expected)) return line;
                            }
                        }
                        catch (TimeoutException) { }
                        catch (Exception ex) { Log(string.Format(T("LogErrorAttempt"), attempt + 1, ex.Message)); }
                        await Task.Delay(150);
                    }
                    return string.Empty;
                }, cts.Token);
                bool ok = !string.IsNullOrEmpty(response) && response.ToUpperInvariant().Contains(expected);
                if (ok) Log(T("LogHandshakeSuccessful"));
                else Log(T("LogHandshakeFailed"));
                return ok;
            }
            catch (Exception ex) { Log(string.Format(T("LogHandshakeError"), ex.Message)); return false; }
        }

        private async Task ConnectAsync()
        {
            // Apre la porta seriale selezionata e avvia il processo di handshake.
            // Se l'apertura o il handshake falliscono, disconnette e segnala l'errore.
            if (cmbPorts.SelectedItem == null)
            {
                MessageBox.Show(T("SelectComPort"), T("Warning"), MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }
            string portName = cmbPorts.SelectedItem.ToString()!;
            if (string.IsNullOrWhiteSpace(portName))
            {
                MessageBox.Show(T("InvalidPort"), T("Warning"), MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }
            try
            {
                Disconnect();
                _serial = new SerialPort(portName, 115200)
                {
                    Encoding = Encoding.ASCII,
                    NewLine = "\n",
                    DtrEnable = true,
                    RtsEnable = true,
                    ReadTimeout = SerialPort.InfiniteTimeout,
                    WriteTimeout = 1000
                };
                _serial.Open();
                Log(string.Format(T("LogPortOpened"), portName));
                await Task.Delay(300);
                Log(T("LogWaitingStabilization"));
                await Task.Delay(3500);
                _serial.DiscardInBuffer();
                _serial.DiscardOutBuffer();
                bool ok = await PerformHandshakeAsync(3500);
                _handshakeStatus = ok ? "OK" : "FAILED";
                if (ok)
                {
                    _serial.ReadTimeout = SerialPort.InfiniteTimeout;
                    _connected = true;
                    UpdateConnectionUi();
                    _readTimer.Start();
                    Log(T("LogConnectionActive"));
                }
                else Disconnect();
            }
            catch (Exception ex) { Log(string.Format(T("LogErrorOpeningPort"), ex.Message)); Disconnect(); }
        }

        private void ReadIncoming()
        {
            // Legge i dati in arrivo dalla porta seriale e costruisce righe complete.
            // Se trova un JSON valido, lo inoltra a ProcessLine.
            if (_serial?.IsOpen != true) return;
            try
            {
                string chunk = _serial.ReadExisting();
                if (string.IsNullOrEmpty(chunk)) return;
                _rxBuffer.Append(chunk);
                string accumulated = _rxBuffer.ToString();
                int idx;
                while ((idx = accumulated.IndexOf('\n')) >= 0)
                {
                    string rawLine = accumulated[..idx];
                    accumulated = accumulated[(idx + 1)..];
                    string line = rawLine.Trim('\r', '\n', ' ');
                    if (string.IsNullOrWhiteSpace(line)) continue;

                    int jsonStart = line.IndexOf('{');
                    // Se la riga non contiene '{' è testo diagnostico (es. stampaAllarmiSintetici):
                    // ignorare silenziosamente senza generare errori.
                    if (jsonStart < 0) continue;
                    if (jsonStart > 0)
                        line = line[jsonStart..].TrimStart();

                    if (!string.IsNullOrWhiteSpace(line))
                        ProcessLine(line);
                }

                _rxBuffer.Clear();
                _rxBuffer.Append(accumulated);
                if (_rxBuffer.Length > 8192)
                {
                    _rxBuffer.Clear();
                    SetLastJsonError(T("LogBufferOverflow"));
                }
            }
            catch (Exception ex)
            {
                Log(string.Format(T("LogSerialReadError"), ex.Message));
                SetLastJsonError(T("JsonStatusSerialError"));
            }
        }

        private void ProcessLine(string line)
        {
            // Analizza una singola riga di input seriale.
            // Ignora output non JSON e deserializza i messaggi validi.
            string trimmed = line.Trim();
            int jsonStart = trimmed.IndexOf('{');
            if (jsonStart > 0)
                trimmed = trimmed[jsonStart..].TrimStart();

            if (!trimmed.StartsWith("{", StringComparison.Ordinal))
            {
                // Riga non-JSON (es. testo diagnostico Arduino): ignora silenziosamente.
                // NON generare errori: non è un problema ma normale output del firmware.
                return;
            }

            try
            {
                var dto = JsonSerializer.Deserialize<TelemetryDto>(trimmed, _jsonOptions);
                if (dto != null)
                {
                    ApplyTelemetry(dto);
                    SetLastJsonError(T("JsonStatusOk"), false);
                }
                else
                {
                    Log(T("LogValidJsonNullObject"));
                    SetLastJsonError(T("JsonStatusNullDto"));
                }
                return;
            }
            catch (JsonException ex)
            {
                Log(T("JsonErrorPrefix") + ex.Message);
                SetLastJsonError(T("JsonErrorPrefix") + ex.Message);
            }
            catch (Exception ex)
            {
                Log(T("ParseErrorPrefix") + ex.Message);
                SetLastJsonError(T("ParseErrorPrefix") + ex.Message);
            }

            Log(T("LogJsonRawPrefix") + trimmed);
        }

        private void SetLastJsonError(string message, bool isError = true)
        {
            // Visualizza l'ultimo errore JSON ricevuto nella UI, con colore diverso
            // per successo o errore.
            if (InvokeRequired) { BeginInvoke(new Action(() => SetLastJsonError(message, isError))); return; }
            if (_lastJsonErrorLabel == null) return;
            _lastJsonErrorLabel.Text = T("LastJsonPrefix") + message;
            _lastJsonErrorLabel.ForeColor = isError ? _accentOrange : _accentGreen;
        }

        private void ApplyTelemetry(TelemetryDto dto)
        {
            // Aggiorna tutte le card dashboard in base ai valori ricevuti dal DTO JSON.
            if (InvokeRequired) { BeginInvoke(new Action(() => ApplyTelemetry(dto))); return; }

            // migliaia, ottenendo 200 invece di 20 — da qui i valori 10× errati.
            static string F1(double v)   => v.ToString("0.0",  CultureInfo.InvariantCulture);
            static string F2(double v)   => v.ToString("0.00", CultureInfo.InvariantCulture);
            static string F0(double v)   => v.ToString("0",    CultureInfo.InvariantCulture);

            void UpdateCard(string key, string value)
            {
                if (_dashboardCards.TryGetValue(key, out var c)) c.Value = value;
            }

            // Temperature — chiavi corrispondenti alle card aggiunte in BuildDashboardPanel
            if (dto.Serb.HasValue)   UpdateCard("serbatoio",  F1(dto.Serb.Value));
            if (dto.Mand.HasValue)   UpdateCard("mandata",    F1(dto.Mand.Value));
            if (dto.Rit.HasValue)    UpdateCard("ritorno",    F1(dto.Rit.Value));
            if (dto.Stampo.HasValue) UpdateCard("stampo",     F1(dto.Stampo.Value));
            if (dto.Res.HasValue)    UpdateCard("resistenza", F1(dto.Res.Value));
            if (dto.Met.HasValue)    UpdateCard("metallo",    F1(dto.Met.Value));
            if (dto.Nucl.HasValue)   UpdateCard("nucleo",     F1(dto.Nucl.Value));

            // Flusso
            if (dto.Port.HasValue)   UpdateCard("portata",   F1(dto.Port.Value));
            if (dto.Pres.HasValue)   UpdateCard("pressione", F2(dto.Pres.Value));
            if (dto.Acqua.HasValue)  UpdateCard("acqua",     F2(dto.Acqua.Value));

            // Energie
            if (dto.Qhx.HasValue)    UpdateCard("qhx",   F0(dto.Qhx.Value));
            if (dto.Qraff.HasValue)  UpdateCard("qraff", F0(dto.Qraff.Value));
            if (dto.Qfill.HasValue)  UpdateCard("qfill", F0(dto.Qfill.Value));
            if (dto.Qconv.HasValue)  UpdateCard("qconv", F0(dto.Qconv.Value));
            if (dto.Qproc.HasValue)  UpdateCard("qproc", F0(dto.Qproc.Value));
            if (dto.Qpump.HasValue)  UpdateCard("qpump", F0(dto.Qpump.Value));

            // Stati
            if (dto.Pompa.HasValue)  UpdateCard("s_pompa",  dto.Pompa.Value == 1 ? T("StatusOn") : T("StatusOff"));
            if (dto.Freddo.HasValue) UpdateCard("s_freddo", dto.Freddo.Value == 1 ? T("StatusOn") : T("StatusOff"));
            if (dto.Riemp.HasValue)  UpdateCard("s_riemp",  dto.Riemp.Value == 1 ? T("StatusOn") : T("StatusOff"));

            if (dto.Alm.HasValue) UpdateAlarmIndicators(dto.Alm.Value);

            _statusPumpLed.IsActive = dto.Pompa.GetValueOrDefault() != 0;
            _statusPumpLabel.Text = $"{T("RelayPump")} {(_statusPumpLed.IsActive ? T("StatusOn") : T("StatusOff"))}";
            _statusColdLed.IsActive = dto.Freddo.GetValueOrDefault() != 0;
            _statusColdLabel.Text = $"{T("RelayCold")} {(_statusColdLed.IsActive ? T("StatusOn") : T("StatusOff"))}";
            _statusFillLed.IsActive = dto.Riemp.GetValueOrDefault() != 0;
            _statusFillLabel.Text = $"{T("RelayFill")} {(_statusFillLed.IsActive ? T("StatusOn") : T("StatusOff"))}";

            // === MANUTENZIONE ===
            // Aggiorna le informazioni di manutenzione, se fornite dal dispositivo.
            if (dto.OrePompa.HasValue || dto.OreRes.HasValue || dto.OreHx.HasValue || dto.UsuraPompa.HasValue)
            {
                UpdateMaintenanceInfo(
                    dto.OrePompa ?? 0,
                    dto.OreRes ?? 0,
                    dto.OreHx ?? 0,
                    dto.UsuraPompa ?? 0
                );
            }
            UpdateTelemetryHeader();
        }

        private void UpdateTelemetryHeader()
        {
            // Aggiorna i label di stato connessione e tipo di fluido nella UI.
            lblConnectionState.Text = _connected ? T("ConnectionStatusOnline") : T("ConnectionStatusOffline");
            lblConnectionState.ForeColor = _connected ? _accentGreen : _accentRed;
            lblFluidState.Text = _isOilMode ? T("FluidOil") : T("FluidWater");
            lblFluidState.ForeColor = _isOilMode ? _accentOrange : _accentBlue;
        }

        private void UpdateAlarmIndicators(int mask)
        {
            // Accende o spegne i LED di allarme in base alla maschera bit.
            for (int i = 0; i < 12; i++)
                if (_alarmLeds.TryGetValue(i, out var led))
                    led.IsActive = (mask & (1 << i)) != 0;
            UpdateActiveAlarmPanel(mask);
        }

        private void SendCommand(string command)
        {
            // Invia una stringa di comando al dispositivo seriale, se connesso.
            if (_serial?.IsOpen != true) { Log(string.Format(T("LogCommandIgnored"), command)); return; }
            try
            {
                _serial.WriteLine(command);
                _serial.BaseStream.Flush();
                Log(string.Format(T("LogTx"), command));
            }
            catch (Exception ex) { Log(string.Format(T("LogErrorSendingCommand"), command, ex.Message)); }
        }

        private void SendParameterCommand(string letter, decimal value)
        {
            // Costruisce il comando di parametro usando la lettera identificativa
            // e il valore numerico formattato su base del parametro.
            string format = letter == "P" ? "0.00" : "0.###";
            string payload = letter + value.ToString(format, CultureInfo.InvariantCulture);
            SendCommand(payload);

            if (letter == "P" && value == 0m)
                UpdateDashboardValueCard("pressione", value.ToString("0.00", CultureInfo.InvariantCulture));
        }

        private void UpdateDashboardValueCard(string key, string value)
        {
            // Aggiorna il valore testuale di una singola card dashboard in modo thread-safe.
            if (InvokeRequired) { BeginInvoke(new Action(() => UpdateDashboardValueCard(key, value))); return; }
            if (_dashboardCards.TryGetValue(key, out var card))
                card.Value = value;
        }

        private void ParameterChanged(object? sender, EventArgs e)
        {
            // Gestisce l'evento di cambio parametro e invia il comando corrispondente.
            if (!_uiReady || _suppressParamEvents) return;
            if (sender is ParameterControl pc && pc.Tag is string letter)
                SendParameterCommand(letter, pc.Value);
        }

        private void BtnSwitchFluid_Click(object? sender, EventArgs e)
        {
            // Gestisce il passaggio da acqua a olio e viceversa.
            // Aggiorna lo stato visivo e invia il comando al dispositivo.
            _isOilMode = !_isOilMode;
            SendCommand(_isOilMode ? "OLIO" : "H2O");
            UpdateTelemetryHeader();
            btnSwitchFluid.Text = T(_isOilMode ? "ButtonSwitchFluidWater" : "ButtonSwitchFluidOil");
            ApplyPressureLock();
        }

        // Blocca/sblocca il controllo pressione in base alla modalità fluido.
        // Con olio: pressione fissa a 0 e slider disabilitato.
        // Con acqua: riportata al valore minimo ammesso e riabilitata.
        private void ApplyPressureLock()
        {
            // Se si usa olio, la pressione non è modificabile. Con acqua la pressione
            // torna disponibile e il parametro viene ripristinato.
            if (!_paramControls.TryGetValue("P", out var pcPressione)) return;

            if (_isOilMode)
            {
                _suppressParamEvents = true;
                pcPressione.Value   = 0m;
                _suppressParamEvents = false;
                pcPressione.Enabled  = false;
                pcPressione.Minimum  = 0m;    // allows showing 0
                pcPressione.Track.Enabled    = false;
                pcPressione.Numeric.Enabled  = false;
                pcPressione.Description = T("LogNotAvailableWithOil");
            }
            else
            {
                pcPressione.Minimum  = 0m;  // restore original minimum
                _suppressParamEvents = true;
                pcPressione.Value    = 0m;    // default water value
                _suppressParamEvents = false;
                pcPressione.Enabled  = true;
                pcPressione.Track.Enabled    = true;
                pcPressione.Numeric.Enabled  = true;
                // Restore original description from translation map
                if (_parameterDescriptionTranslationKeys.TryGetValue(pcPressione, out var descKey))
                    pcPressione.Description = T(descKey);
                SendParameterCommand("P", 0m); // communicate restore to firmware
            }
        }

        private void Disconnect()
        {
            // Chiude la connessione seriale in modo sicuro e libera le risorse.
            _readTimer.Stop();
            _rxBuffer.Clear();
            _connected = false;
            try
            {
                var port = _serial;
                _serial = null;
                if (port != null)
                {
                    if (port.IsOpen) { port.DiscardInBuffer(); port.DiscardOutBuffer(); port.Close(); }
                    port.Dispose();
                }
            }
            catch (Exception ex) { Log(string.Format(T("LogErrorDuringDisconnection"), ex.Message)); }
            UpdateConnectionUi();
            Log(T("LogDisconnectionExecuted"));
        }

        private void UpdateConnectionUi()
        {
            // Aggiorna lo stato di connessione nella UI dopo connect/disconnect.
            if (InvokeRequired) { BeginInvoke(new Action(UpdateConnectionUi)); return; }
            lblConnectionState.Text = _connected ? T("ConnectionStatusOnline") : T("ConnectionStatusOffline");
            lblConnectionState.ForeColor = _connected ? _accentGreen : _accentRed;
            UpdateFooterStatus();
        }

        private void ScanPorts()
        {
            cmbPorts.Items.Clear();
            var ports = SerialPort.GetPortNames().OrderBy(p => p).ToArray();
            cmbPorts.Items.AddRange(ports);
            if (ports.Length > 0) { cmbPorts.SelectedIndex = 0; Log(string.Format(T("LogPortsFound"), string.Join(", ", ports))); }
            else Log(T("LogNoComPortFound"));
        }

        private void Log(string text)
        {
            // Scrive messaggi nel log interno della UI in modo thread-safe.
            if (InvokeRequired) { BeginInvoke(new Action(() => Log(text))); return; }
            txtLog.AppendText($"[{DateTime.Now:HH:mm:ss}] {text}{Environment.NewLine}");
            txtLog.ScrollToCaret();
        }

        protected override void OnFormClosing(FormClosingEventArgs e)
        {
            Disconnect();
            base.OnFormClosing(e);
        }

        // ---------- MODERN UI ----------
        private void InitializeModernUI()
        {
            // Costruisce l'intera interfaccia utente moderna a runtime.
            // In questa funzione si creano i layout, le sezioni, i pulsanti e le card.
            var logPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "debug_startup.log");
            File.AppendAllText(logPath, "InitializeModernUI start\r\n");
            SuspendLayout();

            Text = $"{T("AppTitle")} - {T("AppSubtitle")}";
            Size = new Size(1820, 1020);
            MinimumSize = new Size(1500, 860);
            StartPosition = FormStartPosition.CenterScreen;
            BackColor = _bg;
            Font = new Font("Segoe UI", 10F, FontStyle.Regular);
            ForeColor = _text;
            AutoScroll = false;

            // ── Icona applicazione (titlebar + taskbar) ───────────────
            try
            {
                var iconPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "icona.ico");
                if (File.Exists(iconPath))
                    Icon = new Icon(iconPath);
            }
            catch { /* if the file doesn't exist, simply use the default icon */ }

            var appGrid = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                BackColor = _bg,
                ColumnCount = 3,
                RowCount = 3,
                Padding = new Padding(0),
                Margin = new Padding(0)
            };
            appGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 250));
            appGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f));
            appGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 300));
            appGrid.RowStyles.Add(new RowStyle(SizeType.Absolute, 100)); 
            appGrid.RowStyles.Add(new RowStyle(SizeType.Percent, 100f));
            appGrid.RowStyles.Add(new RowStyle(SizeType.Absolute, 220));

            // ── HEADER ───────────────────────────────────────────────────
             var topBar = new Panel
            {
                 Dock = DockStyle.Fill,
                 BackColor = Color.FromArgb(0x0A, 0x0F, 0x1F),
                 Height = 98
             };


            topBar.Paint += (s, e) =>
            {
                 using var pen = new Pen(_accentCyan, 3);
                 e.Graphics.DrawLine(pen, 0, topBar.Height - 3, topBar.Width, topBar.Height - 3);
            };

            var headerLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                BackColor = Color.Transparent,
                ColumnCount = 3,
                RowCount = 1,
                Padding = new Padding(30, 22, 30, 18)
            };

            headerLayout.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
            headerLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
            headerLayout.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));

            // Title
            var titleLabel = new Label
            {
                Text = T("AppTitle"),
                Font = new Font("Segoe UI", 23F, FontStyle.Bold),
                ForeColor = _accentCyan,
                AutoSize = true,
                Padding = new Padding(0, 10, 0, 0)
            };

            // Pulsante Lingua
            btnLanguageToggle = new Button
            {
                Text = GetLanguageButtonText(),
                AutoSize = true,
                MinimumSize = new Size(130, 54),
                FlatStyle = FlatStyle.Flat,
                BackColor = Color.FromArgb(0x1E, 0x2A, 0x45),
                ForeColor = Color.White,
                Font = new Font("Segoe UI", 14F, FontStyle.Bold),
                Cursor = Cursors.Hand,
                Margin = new Padding(40, 8, 0, 8),
                Padding = new Padding(22, 8, 22, 8)
            };

            btnLanguageToggle.FlatAppearance.BorderSize = 0;
            btnLanguageToggle.FlatAppearance.MouseOverBackColor = _accentCyan;
            btnLanguageToggle.FlatAppearance.MouseDownBackColor = Color.FromArgb(0x00, 0x94, 0xB8);

            btnLanguageToggle.MouseEnter += (s, e) => 
            { 
                btnLanguageToggle.BackColor = _accentCyan; 
                btnLanguageToggle.ForeColor = Color.FromArgb(0x0A, 0x0F, 0x1F); 
            };

            btnLanguageToggle.MouseLeave += (s, e) => 
            { 
                btnLanguageToggle.BackColor = Color.FromArgb(0x1E, 0x2A, 0x45); 
                btnLanguageToggle.ForeColor = Color.White; 
            };

            btnLanguageToggle.Click += (_, _) => ToggleLanguage();

            // Clock
            _clockLabel = new Label
            {
                AutoSize = true,
                ForeColor = _accentBlue,
                Font = new Font("Segoe UI", 15F, FontStyle.Bold),
                Padding = new Padding(20, 18, 30, 0),
                Anchor = AnchorStyles.Right
            };

            // Add everything
            headerLayout.Controls.Add(titleLabel, 0, 0);
            headerLayout.Controls.Add(new Label { Width = 50 }, 1, 0); // spacer
            headerLayout.Controls.Add(btnLanguageToggle, 2, 0);
            headerLayout.Controls.Add(_clockLabel, 2, 0);

            topBar.Controls.Add(headerLayout);
            appGrid.Controls.Add(topBar, 0, 0);
            appGrid.SetColumnSpan(topBar, 3);

            var leftPanel = new Panel { Dock = DockStyle.Fill, BackColor = _bg, Margin = new Padding(0, 0, 15, 15) };
            var navPanel = new RoundedPanel
            {
                Dock = DockStyle.Fill,
                BackColor = _panel2,
                CornerRadius = 20,
                Padding = new Padding(20)
            };
            var navTitle = new Label
            {
                Text = T("NavTitle"),
                AutoSize = true,
                ForeColor = _muted,
                Font = new Font("Segoe UI", 9F, FontStyle.Bold),
                Dock = DockStyle.Top
            };
            navPanel.Controls.Add(navTitle);
            var menuFlow = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.TopDown,
                WrapContents = false,
                AutoScroll = false,                   
                BackColor = Color.Transparent,
                Padding = new Padding(0, 16, 0, 0)
            };
            var btnDashboard  = CreateNavButton("NavDashboard", _accentBlue);
            var btnParameters = CreateNavButton("NavParameters", _accentTeal);
            var btnActions    = CreateNavButton("NavActions", _accentOrange);
            var btnAlarms     = CreateNavButton("NavAlarms", _accentRed);
            var btnConsole    = CreateNavButton("NavConsole", _accentPurple);
            menuFlow.Controls.AddRange(new Control[] { btnDashboard, btnParameters, btnActions, btnAlarms, btnConsole });
            navPanel.Controls.Add(menuFlow);
            leftPanel.Controls.Add(navPanel);
            appGrid.Controls.Add(leftPanel, 0, 1);

            _contentPanel = new Panel { Dock = DockStyle.Fill, BackColor = _panel, Padding = new Padding(0), Margin = new Padding(0, 0, 15, 15) };
            File.AppendAllText(logPath, "Building dashboard panel\r\n");
            _dashboardView  = BuildDashboardPanel();
            File.AppendAllText(logPath, "Building parameters panel\r\n");
            _parametersView = BuildParametersPanel();
            File.AppendAllText(logPath, "Building alarms panel\r\n");
            _alarmsView     = BuildAlarmsPanel();
            File.AppendAllText(logPath, "Building actions panel\r\n");
            _actionsView    = BuildActionsPanel();
            File.AppendAllText(logPath, "Building console panel\r\n");
            _consoleView    = BuildConsolePanel();
            btnDashboard.Click  += (_, _) => { ShowView(_dashboardView);  SetActiveNav(btnDashboard); };
            btnParameters.Click += (_, _) => { ShowView(_parametersView); SetActiveNav(btnParameters); };
            btnActions.Click    += (_, _) => { ShowView(_actionsView);    SetActiveNav(btnActions); };
            btnAlarms.Click     += (_, _) => { ShowView(_alarmsView);     SetActiveNav(btnAlarms); };
            btnConsole.Click    += (_, _) => { ShowView(_consoleView);    SetActiveNav(btnConsole); };
            _contentPanel.Controls.Add(_dashboardView);
            _contentPanel.Controls.Add(_parametersView);
            _contentPanel.Controls.Add(_alarmsView);
            _contentPanel.Controls.Add(_actionsView);
            _contentPanel.Controls.Add(_consoleView);
            appGrid.Controls.Add(_contentPanel, 1, 1);

            var rightPanel = new Panel { Dock = DockStyle.Fill, BackColor = _bg, Margin = new Padding(0, 0, 0, 15) };
            var statusPanel = BuildStatusPanel();
            statusPanel.Dock = DockStyle.Fill;
            rightPanel.Controls.Add(statusPanel);
            appGrid.Controls.Add(rightPanel, 2, 1);

            ShowView(_dashboardView);
            SetActiveNav(btnDashboard);

            var bottomPanel = new Panel
            {
                Dock = DockStyle.Fill,
                BackColor = _panel2,
                Padding = new Padding(16),
                Margin = new Padding(0, 15, 0, 0)
            };
            var logHeader = new Label
            {
                Text = T("ConsoleLogHeader"),
                AutoSize = true,
                Font = new Font("Segoe UI", 11F, FontStyle.Bold),
                ForeColor = _text,
                Dock = DockStyle.Top
            };
            var logPanel = new Panel { Dock = DockStyle.Fill, BackColor = _panel, Padding = new Padding(14) };
            txtLog.Multiline = true;
            txtLog.Dock = DockStyle.Fill;
            txtLog.BackColor = Color.FromArgb(0x06, 0x09, 0x14);
            txtLog.ForeColor = Color.FromArgb(0x7A, 0xE2, 0xAD);
            txtLog.Font = new Font("Consolas", 9F);
            txtLog.ReadOnly = true;
            txtLog.ScrollBars = ScrollBars.Vertical;
            txtLog.BorderStyle = BorderStyle.None;
            logPanel.Controls.Add(txtLog);
            bottomPanel.Controls.Add(logPanel);
            bottomPanel.Controls.Add(logHeader);
            appGrid.Controls.Add(bottomPanel, 0, 2);
            appGrid.SetColumnSpan(bottomPanel, 3);

            topBar.Controls.Add(_clockLabel);
            var clockTimer = new System.Windows.Forms.Timer { Interval = 1000 };
            clockTimer.Tick += (_, _) => _clockLabel.Text = DateTime.Now.ToString("HH:mm:ss");
            clockTimer.Start();

            Controls.Add(appGrid);
            _uiReady = true;
            UpdateTelemetryHeader();
            UpdateFooterStatus();
            ResumeLayout(false);
            _uiReady = true;
            ApplyTranslations();        // ← AGGIUNGI QUESTA RIGA

            // Attach Shown handler to help debug visibility issues
            try
            {
                var logPath2 = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "debug_startup.log");
                File.AppendAllText(logPath2, "Attaching Shown handler\r\n");
                Shown += (s, e) =>
                {
                    File.AppendAllText(logPath2, "MainForm Shown\r\n");
                    try { BringToFront(); Activate(); } catch { }
                };
            }
            catch { }
        }

        // ── CONNECTION (UNTOUCHED) ──────────────────────────────────────
        private RoundedPanel BuildConnectionPanel()
        {
            // Costruisce la sezione di connessione seriale con porta, dispositivo
            // e pulsanti Connetti/Disconnetti.
            var panel = new RoundedPanel
            {
                Dock = DockStyle.Fill,
                BackColor = _panel,
                CornerRadius = 20,
                Padding = new Padding(20)
            };

            var content = new TableLayoutPanel
            {
                Dock = DockStyle.Fill,
                AutoSize = true,
                ColumnCount = 2,
                RowCount = 10,
                BackColor = Color.Transparent,
                CellBorderStyle = TableLayoutPanelCellBorderStyle.None,
                Margin = new Padding(0),
                Padding = new Padding(0)
            };
            content.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
            content.ColumnStyles.Add(new ColumnStyle(SizeType.AutoSize));
            for (int i = 0; i < content.RowCount; i++)
                content.RowStyles.Add(new RowStyle(SizeType.AutoSize));

            var titleLabel = CreateTranslatedLabel("ConnectionPanelTitle", new Font("Segoe UI", 16F, FontStyle.Bold), _text, false);
            titleLabel.Height = 34;
            titleLabel.Dock = DockStyle.Top;
            content.Controls.Add(titleLabel, 0, 0);
            content.SetColumnSpan(titleLabel, 2);

            var comPortLabel = CreateTranslatedLabel("LabelComPort", new Font("Segoe UI", 9F), _muted, false);
            comPortLabel.Height = 22;
            content.Controls.Add(comPortLabel, 0, 1);
            cmbPorts.DropDownStyle = ComboBoxStyle.DropDownList;
            cmbPorts.Font = new Font("Segoe UI", 11F, FontStyle.Bold);
            cmbPorts.Width = 280;
            cmbPorts.Margin = new Padding(0, 4, 0, 10);
            content.Controls.Add(cmbPorts, 0, 2);

            btnScan = CreateCmdButton("ButtonScan", _accentBlue, 140);
            btnScan.Click += (_, _) => ScanPorts();
            btnScan.Margin = new Padding(0, 4, 0, 10);
            content.Controls.Add(btnScan, 1, 2);

            var deviceLabel = CreateTranslatedLabel("LabelDevice", new Font("Segoe UI", 9F), _muted, false);
            deviceLabel.Height = 22;
            content.Controls.Add(deviceLabel, 0, 3);
            cmbDevice.DropDownStyle = ComboBoxStyle.DropDownList;
            cmbDevice.Items.AddRange(new object[] { T("DeviceEsp32"), T("DeviceCustom") });
            cmbDevice.SelectedIndex = 0;
            cmbDevice.Font = new Font("Segoe UI", 11F, FontStyle.Bold);
            cmbDevice.Width = 280;
            cmbDevice.Margin = new Padding(0, 4, 0, 10);
            cmbDevice.SelectedIndexChanged += (_, _) => { /* name automatically managed by GetExpectedNameForDevice */ };
            content.Controls.Add(cmbDevice, 0, 4);

            var actionsFlow = new FlowLayoutPanel
            {
                FlowDirection = FlowDirection.LeftToRight,
                WrapContents = false,
                AutoSize = true,
                Dock = DockStyle.Fill,
                BackColor = Color.Transparent,
                Margin = new Padding(0, 4, 0, 10)
            };
            btnConnect = CreateCmdButton("ButtonConnect", _accentGreen, 140);
            btnConnect.Click += async (_, _) => await ConnectAsync();
            btnDisconnect = CreateCmdButton("ButtonDisconnect", _accentRed, 140);
            btnDisconnect.Click += (_, _) => Disconnect();
            actionsFlow.Controls.Add(btnConnect);
            actionsFlow.Controls.Add(btnDisconnect);
            content.Controls.Add(actionsFlow, 0, 7);
            content.SetColumnSpan(actionsFlow, 2);

            lblConnectionState.Text = T("ConnectionStatusOffline");
            lblConnectionState.AutoSize = true;
            lblConnectionState.Font = new Font("Segoe UI", 12F, FontStyle.Bold);
            lblConnectionState.ForeColor = _accentRed;
            lblConnectionState.Margin = new Padding(0, 4, 0, 0);
            content.Controls.Add(lblConnectionState, 0, 8);
            content.SetColumnSpan(lblConnectionState, 2);

            lblFluidState.Text = T("FluidWater");
            lblFluidState.AutoSize = true;
            lblFluidState.Font = new Font("Segoe UI", 12F, FontStyle.Bold);
            lblFluidState.ForeColor = _accentBlue;
            lblFluidState.Margin = new Padding(0, 4, 0, 0);
            content.Controls.Add(lblFluidState, 0, 9);
            content.SetColumnSpan(lblFluidState, 2);

            panel.Controls.Add(content);
            return panel;
        }

        // ── RIGHT STATUS PANEL ──────────────────────────────────────
        private Panel BuildStatusPanel()
        {
            // Crea il pannello laterale di stato con carte di connessione,
            // indicatori relè e lista allarmi attivi.
            var panel = new RoundedPanel { Dock = DockStyle.Fill, BackColor = _panel2, CornerRadius = 20, Padding = new Padding(16) };

            var flow = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.TopDown,
                WrapContents = false,
                AutoScroll = true,
                BackColor = Color.Transparent,
                Padding = new Padding(0),
                Margin = new Padding(0)
            };

            var connectionCard = new RoundedPanel
            {
                BackColor = _card,
                CornerRadius = 16,
                Padding = new Padding(16),
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowOnly,
                MinimumSize = new Size(300, 190),
                Margin = new Padding(0, 0, 0, 12)
            };

            var connTitle = CreateTranslatedLabel("StatusConnectionTitle", new Font("Segoe UI", 10F, FontStyle.Bold), _text);
            lblConnectionState = new Label
            {
                Text = T("StatusDisconnected"),
                AutoSize = true,
                Font = new Font("Segoe UI", 11F, FontStyle.Bold),
                ForeColor = _accentRed,
                Margin = new Padding(0, 6, 0, 0)
            };
            lblFluidState = new Label
            {
                Text = T("FluidWater"),
                AutoSize = true,
                Font = new Font("Segoe UI", 9F, FontStyle.Bold),
                ForeColor = _accentBlue,
                Margin = new Padding(0, 4, 0, 0)
            };
            var connInfo = new FlowLayoutPanel
            {
                Dock = DockStyle.Top,
                FlowDirection = FlowDirection.TopDown,
                WrapContents = false,
                AutoSize = true,
                BackColor = Color.Transparent,
                Margin = new Padding(0, 12, 0, 0)
            };
            _footerConnectionLabel = new Label
            {
                Text = T("PortLabelPrefix") + T("StatusNotAvailable"),
                AutoSize = true,
                ForeColor = _muted,
                Font = new Font("Segoe UI", 9F),
                Margin = new Padding(0, 4, 0, 0)
            };
            _footerHandshakeLabel = new Label
            {
                Text = T("HandshakeLabelPrefix") + T("HandshakeNone"),
                AutoSize = true,
                ForeColor = _muted,
                Font = new Font("Segoe UI", 9F),
                Margin = new Padding(0, 2, 0, 0)
            };

            connInfo.Controls.Add(_footerConnectionLabel);
            connInfo.Controls.Add(_footerHandshakeLabel);
            _lastJsonErrorLabel = new Label
            {
                Text = T("LastJsonPrefix") + T("StatusNoError"),
                AutoSize = true,
                ForeColor = _muted,
                Font = new Font("Segoe UI", 8F, FontStyle.Italic),
                Margin = new Padding(0, 10, 0, 0)
            };

            connectionCard.Controls.Add(connTitle);
            connectionCard.Controls.Add(lblConnectionState);
            connectionCard.Controls.Add(lblFluidState);
            connectionCard.Controls.Add(connInfo);
            connectionCard.Controls.Add(_lastJsonErrorLabel);

            var connControls = new FlowLayoutPanel
            {
                Dock = DockStyle.Top,
                FlowDirection = FlowDirection.TopDown,
                WrapContents = false,
                AutoSize = true,
                BackColor = Color.Transparent,
                Margin = new Padding(0, 14, 0, 0)
            };
            cmbPorts.DropDownStyle = ComboBoxStyle.DropDownList;
            cmbPorts.Font = new Font("Segoe UI", 9F, FontStyle.Bold);
            cmbPorts.Width = 260;
            cmbPorts.Margin = new Padding(0, 6, 0, 0);
            cmbDevice.DropDownStyle = ComboBoxStyle.DropDownList;
            cmbDevice.Items.Clear();
            cmbDevice.Items.AddRange(new object[] { T("DeviceEsp32"), T("DeviceCustom") });
            cmbDevice.SelectedIndex = 0;
            cmbDevice.Font = new Font("Segoe UI", 9F, FontStyle.Bold);
            cmbDevice.Width = 260;
            cmbDevice.Margin = new Padding(0, 6, 0, 0);
            cmbDevice.SelectedIndexChanged += (_, _) => { /* name automatically managed by GetExpectedNameForDevice */ };

            btnScan = CreateCmdButton("ButtonScan", _accentBlue, 260);
            btnScan.Height = 36;
            btnScan.Margin = new Padding(0, 10, 0, 0);
            btnScan.Click += (_, _) => ScanPorts();

            btnConnect = CreateCmdButton("ButtonConnect", _accentGreen, 260);
            btnConnect.Height = 42;
            btnConnect.Margin = new Padding(0, 10, 0, 0);
            btnConnect.Click += async (_, _) => await ConnectAsync();

            btnDisconnect = CreateCmdButton("ButtonDisconnect", _accentRed, 260);
            btnDisconnect.Height = 42;
            btnDisconnect.Margin = new Padding(0, 8, 0, 0);
            btnDisconnect.Click += (_, _) => Disconnect();

            connControls.Controls.Add(CreateTranslatedLabel("LabelComPort", new Font("Segoe UI", 8F), _muted, true, new Padding(0, 12, 0, 0)));
            connControls.Controls.Add(cmbPorts);
            connControls.Controls.Add(CreateTranslatedLabel("LabelDevice", new Font("Segoe UI", 8F), _muted, true, new Padding(0, 10, 0, 0)));
            connControls.Controls.Add(cmbDevice);
            connControls.Controls.Add(btnScan);
            connControls.Controls.Add(btnConnect);
            connControls.Controls.Add(btnDisconnect);
            connectionCard.Controls.Add(connControls);

            flow.Controls.Add(connectionCard);

            var relayCard = new RoundedPanel
            {
                BackColor = _card,
                CornerRadius = 16,
                Padding = new Padding(14),
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowOnly,
                MinimumSize = new Size(300, 180),
                Margin = new Padding(0, 0, 0, 12)
            };
            relayCard.Controls.Add(CreateTranslatedLabel("RelayStatusHeader", new Font("Segoe UI", 10F, FontStyle.Bold), _text));

            void AddRelay(string title, Color color, out LedIndicator led, out Label lbl)
            {
                var row = new FlowLayoutPanel
                {
                    Dock = DockStyle.Top,
                    AutoSize = true,
                    FlowDirection = FlowDirection.LeftToRight,
                    WrapContents = false,
                    Margin = new Padding(0, 12, 0, 0),
                    BackColor = Color.Transparent
                };
                led = new LedIndicator { Size = new Size(22, 22), OffColor = Color.FromArgb(0x3A, 0x3A, 0x5A), OnColor = color };
                lbl = new Label { Text = title + " " + T("StatusOff"), AutoSize = true, ForeColor = _text, Font = new Font("Segoe UI", 10F, FontStyle.Bold), Margin = new Padding(8, 4, 0, 0) };
                row.Controls.Add(led);
                row.Controls.Add(lbl);
                relayCard.Controls.Add(row);
            }

            AddRelay(T("RelayPump"), _accentGreen, out _statusPumpLed, out _statusPumpLabel);
            AddRelay(T("RelayCold"), _accentCyan, out _statusColdLed, out _statusColdLabel);
            AddRelay(T("RelayFill"), _accentBlue, out _statusFillLed, out _statusFillLabel);
            flow.Controls.Add(relayCard);

            var alarmHeader = CreateTranslatedLabel("AlarmHeader", new Font("Segoe UI", 8F, FontStyle.Bold), _muted, true, new Padding(0, 0, 0, 6));
            flow.Controls.Add(alarmHeader);

            var alarmCard = new RoundedPanel
            {
                BackColor = _card,
                CornerRadius = 16,
                Padding = new Padding(12),
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowOnly,
                MinimumSize = new Size(300, 240),
                Margin = new Padding(0, 0, 0, 0)
            };

            _activeAlarmPanel = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                FlowDirection = FlowDirection.TopDown,
                WrapContents = false,
                AutoScroll = true,
                BackColor = Color.Transparent,
                Margin = new Padding(0)
            };
            alarmCard.Controls.Add(_activeAlarmPanel);
            flow.Controls.Add(alarmCard);

            panel.Controls.Add(flow);
            UpdateActiveAlarmPanel(0);
            return panel;
        }

        // ── DASHBOARD ───────────────────────────────────────────────────
        private Panel BuildDashboardPanel()
        {
            // Crea il pannello dashboard con le card che mostrano i valori telemetrici.
            var logPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "debug_startup.log");
            File.AppendAllText(logPath, "BuildDashboardPanel start\r\n");
            var panel = new Panel { Dock = DockStyle.Fill, AutoScroll = true, BackColor = _panel };
            var outer = new FlowLayoutPanel
            {
                Dock = DockStyle.Fill,
                AutoScroll = true,
                FlowDirection = FlowDirection.LeftToRight,
                WrapContents = true,
                Padding = new Padding(2, 2, 2, 2),
                BackColor = _panel,
                Margin = new Padding(0)
            };

            Panel MakeSection(string emoji, string titleKey, Color titleColor,
                              (string key, string labelKey, string unit, Color accent)[] cards)
            {
                File.AppendAllText(logPath, $"MakeSection start {titleKey}\r\n");
                var sec = new RoundedPanel
                {
                    AutoSize = true,
                    BackColor = _panel2,
                    CornerRadius = 16,
                    Padding = new Padding(4),
                    Margin = new Padding(2),
                    MinimumSize = new Size(260, 130)
                };

                var hdr = CreateTranslatedLabel(titleKey, 
                    new Font("Segoe UI", 9F, FontStyle.Bold), 
                    titleColor, 
                    false, 
                    new Padding(0, 0, 0, 8));

                hdr.Height = 26;
                hdr.Dock = DockStyle.Top;
                hdr.Text = $"{emoji} {hdr.Text}";   // adds emoji
                hdr.TextAlign = ContentAlignment.MiddleLeft;

                var cardFlow = new FlowLayoutPanel
                {
                    Dock = DockStyle.Fill,
                    AutoSize = true,
                    FlowDirection = FlowDirection.LeftToRight,
                    WrapContents = true,
                    BackColor = Color.Transparent,
                    Margin = new Padding(0),
                    Padding = new Padding(0)
                };

                foreach (var (key, labelKey, unit, accent) in cards)
                {
                    File.AppendAllText(logPath, $"MakeSection {titleKey}: adding card {labelKey}\r\n");
                    var card = new ValueCard
                    {
                        Title = T(labelKey),
                        Unit = unit,
                        AccentColor = accent,
                        Value = "--",
                        Width = 320,
                        Height = 180,
                        Margin = new Padding(6),
                        AccessibleName = $"{T(labelKey)} card",
                        AccessibleDescription = $"Valore corrente di {T(labelKey)} ({unit})"
                    };
                    _dashboardCards[key] = card;
                    _cardTranslationKeys[card] = labelKey;
                    cardFlow.Controls.Add(card);
                }

                File.AppendAllText(logPath, $"MakeSection {titleKey}: cards added\r\n");
                sec.Controls.Add(cardFlow);
                sec.Controls.Add(hdr);
                File.AppendAllText(logPath, $"MakeSection end {titleKey}\r\n");
                return sec;
            }

            outer.Controls.Add(MakeSection("🌡", "SectionTemperatureFluid", _accentBlue,
            new[]
            {
                ("mandata", "DashboardMandata", "°C", _accentBlue),
                ("ritorno", "DashboardRitorno", "°C", _accentCyan)
            }));

            outer.Controls.Add(MakeSection("🏭", "SectionTemperatureTank", _accentOrange,
            new[]
            {
                ("serbatoio", "DashboardSerbatoio", "°C", _accentOrange)
            }));

            outer.Controls.Add(MakeSection("⚙", "SectionTemperatureProcess", _accentPurple,
            new[]
            {
                ("metallo", "DashboardMetallo", "°C", _accentOrange),
                ("nucleo", "DashboardNucleo", "°C", _accentPurple)
            }));

            outer.Controls.Add(MakeSection("🛠", "SectionTemperatureMold", _accentGreen,
            new[]
            {
                ("stampo", "DashboardStampo", "°C", _accentGreen),
                ("resistenza", "DashboardResistenza", "°C", _accentRed)
            }));

            outer.Controls.Add(MakeSection("💧", "SectionDashboardFlow", _accentBlue,
            new[]
            {
                ("portata",   "DashboardPortata",     "L/min", _accentCyan),
                ("pressione", "DashboardPressione",   "bar",   _accentGreen),
                ("acqua",     "DashboardMassaAcqua", "kg",    _accentBlue),
            }));

            outer.Controls.Add(MakeSection("⚡", "SectionDashboardEnergy", _accentOrange,
            new[]
            {
                ("qhx",   "DashboardQhx", "W", _accentCyan),
                ("qraff", "DashboardQraff",      "W", _accentBlue),
                ("qfill", "DashboardQfill",        "W", _accentGreen),
                ("qconv", "DashboardQconv",       "W", _accentPurple),
                ("qproc", "DashboardQproc",    "W", _accentOrange),
                ("qpump", "DashboardQpump",       "W", _accentRed),
            }));

            outer.Controls.Add(MakeSection("🔄", "SectionDashboardState", _accentGreen,
            new[]
            {
                ("s_pompa",  "DashboardPompa",       "", _accentGreen),
                ("s_freddo", "DashboardFreddo",      "", _accentCyan),
                ("s_riemp",  "DashboardRiempimento", "", _accentBlue),
            }));

            panel.Controls.Add(outer);
            File.AppendAllText(logPath, "BuildDashboardPanel end\r\n");
            return panel;
        }

        private void UpdateFooterStatus()
        {
            // Aggiorna la riga di stato inferiore con porta e handshake.
            if (InvokeRequired) { BeginInvoke(new Action(UpdateFooterStatus)); return; }
            if (_headerConnectionStateLabel != null)
            {
                _headerConnectionStateLabel.Text = _connected ? T("ConnectionStatusOnline") : T("ConnectionStatusOffline");
                _headerConnectionStateLabel.ForeColor = _connected ? _accentGreen : _accentRed;
            }
            if (_headerPortLabel != null)
                _headerPortLabel.Text = T("PortLabelPrefix") + (cmbPorts.SelectedItem?.ToString() ?? T("StatusNotAvailable"));
            if (_headerHandshakeLabel != null)
            {
                var handshakeText = GetTranslatedHandshakeStatus();
                _headerHandshakeLabel.Text = T("HandshakeLabelPrefix") + handshakeText;
                _headerHandshakeLabel.ForeColor = _handshakeStatus == "OK" ? _accentGreen :
                                                (_handshakeStatus == "FAILED" ? _accentRed : _muted);
            }
            _footerConnectionLabel.Text = T("PortLabelPrefix") + (cmbPorts.SelectedItem?.ToString() ?? T("StatusNotAvailable"));
            if (_footerHandshakeLabel != null)
            {
                var handshakeText = GetTranslatedHandshakeStatus();
                _footerHandshakeLabel.Text = T("HandshakeLabelPrefix") + handshakeText;
                _footerHandshakeLabel.ForeColor = _handshakeStatus == "OK" ? _accentGreen :
                                                  (_handshakeStatus == "FAILED" ? _accentRed : _muted);
            }
        }

        private void UpdateActiveAlarmPanel(int mask)
        {
            // Mostra la lista di allarmi attivi basata sulla maschera bit.
            if (_activeAlarmPanel == null) return;
            if (InvokeRequired) { BeginInvoke(new Action(() => UpdateActiveAlarmPanel(mask))); return; }
            _activeAlarmPanel.Controls.Clear();
            bool any = false;
            for (int i = 0; i < _alarmNameKeys.Length; i++)
            {
                if ((mask & (1 << i)) != 0)
                {
                    any = true;
                    var alarmRow = new Panel { AutoSize = true, Margin = new Padding(2), BackColor = Color.Transparent };
                    var dot = new Panel { Size = new Size(8, 8), Location = new Point(2, 5), BackColor = _accentRed };
                    var alarmLabel = new Label
                    {
                        Text = T(_alarmNameKeys[i]),
                        AutoSize = true,
                        Location = new Point(16, 0),
                        ForeColor = _accentRed,
                        Font = new Font("Segoe UI", 9F, FontStyle.Bold)
                    };
                    alarmRow.Controls.Add(dot);
                    alarmRow.Controls.Add(alarmLabel);
                    _activeAlarmPanel.Controls.Add(alarmRow);
                }
            }
            if (!any)
            {
                _activeAlarmPanel.Controls.Add(new Label
                {
                    Text = T("NoActiveAlarm"),
                    AutoSize = true,
                    ForeColor = _accentGreen,
                    Font = new Font("Segoe UI", 9F, FontStyle.Italic),
                    Margin = new Padding(4)
                });
            }
        }
            private void UpdateMaintenanceInfo(double orePompa, double oreRes, double oreHx, double usura)
    {
        // Aggiorna le etichette di manutenzione con i valori ricevuti.
        if (InvokeRequired)
        {
            BeginInvoke(new Action(() => UpdateMaintenanceInfo(orePompa, oreRes, oreHx, usura)));
            return;
        }

        if (lblManutenzioneInfo != null)
        {
            lblManutenzioneInfo.Text = string.Format(T("MaintenanceInfo"), orePompa, oreRes, oreHx, usura);
        }

        if (lblOrePompa != null) lblOrePompa.Text = string.Format(T("MaintenancePumpHours"), orePompa);
        if (lblOreResistenza != null) lblOreResistenza.Text = string.Format(T("MaintenanceResistorHours"), oreRes);
        if (lblOreHx != null) lblOreHx.Text = string.Format(T("MaintenanceHxHours"), oreHx);
        if (lblUsuraPompa != null) lblUsuraPompa.Text = string.Format(T("MaintenancePumpWear"), usura);
    }
        // ── PARAMETERS ───────────────────────────────────────────────────
private Panel BuildParametersPanel()
{
    // Crea il pannello dei parametri con gruppi suddivisi per categorie.
    // Contiene anche i nuovi parametri aggiunti (massa tubi, efficienza pompa).
    var panel = new Panel { Dock = DockStyle.Fill, AutoScroll = true, BackColor = _panel };
    var headerRow = new Panel 
    { 
        Dock = DockStyle.Top, 
        Height = 52, 
        BackColor = _panel2, 
        Padding = new Padding(20, 14, 20, 8) 
    };
    headerRow.Controls.Add(new Label
    {
        Text = T("ParamHeader"),
        AutoSize = true,
        Font = new Font("Segoe UI", 13F, FontStyle.Bold),
        ForeColor = _text
    });
    headerRow.Controls.Add(new Label
    {
        Text = T("ParamHeaderDesc"),
        AutoSize = true,
        Font = new Font("Segoe UI", 8F),
        ForeColor = _muted,
        Location = new Point(20, 34)
    });

    var groupsContainer = new FlowLayoutPanel
    { 
        Dock = DockStyle.Fill, 
        AutoScroll = true,
        AutoSize = false,
        BackColor = _panel,
        FlowDirection = FlowDirection.TopDown,
        WrapContents = false,
        Padding = new Padding(18, 14, 18, 14),
        Margin = new Padding(0)
    };

    FlowLayoutPanel CreateGroup(string titleKey, Color accent)
    {
        var lbl = CreateTranslatedLabel(titleKey, new Font("Segoe UI", 10F, FontStyle.Bold), accent, true, new Padding(0, 12, 0, 6));
        lbl.Dock = DockStyle.Top;
        groupsContainer.Controls.Add(lbl);

        var groupFlow = new FlowLayoutPanel
        {
            Dock = DockStyle.Top,
            AutoSize = true,
            AutoSizeMode = AutoSizeMode.GrowAndShrink,
            FlowDirection = FlowDirection.LeftToRight,
            WrapContents = true,
            Padding = new Padding(8),
            BackColor = _panel,
            Margin = new Padding(0, 0, 0, 12)
        };

        groupFlow.Resize += (_, _) =>
        {
            int containerWidth = Math.Max(280, groupFlow.ClientSize.Width - groupFlow.Padding.Horizontal);
            int cols = containerWidth / 260;
            cols = Math.Max(1, Math.Min(5, cols));
            int cardWidth = Math.Max(260, (containerWidth / cols) - 12);
            foreach (var c in groupFlow.Controls.OfType<ParameterControl>())
            {
                c.Width = cardWidth;
                c.Height = 130;
                c.Margin = new Padding(6);
            }
        };

        groupsContainer.Controls.Add(groupFlow);
        return groupFlow;
    }

    void AddParamTo(FlowLayoutPanel container, string tag, string labelKey, string descKey, decimal min, decimal max, decimal def, string unit, int decimals)
    {
        var pc = new ParameterControl
        {
            Tag = tag,
            Title = T(labelKey),
            Description = T(descKey),
            Minimum = min,
            Maximum = max,
            Value = def,
            Unit = unit,
            DecimalPlaces = decimals,
            Margin = new Padding(6),
            Width = 260,
            Height = 130
        };
        pc.MinimumSize = new Size(220, 110);
        pc.ValueChanged += ParameterChanged;
        _parameterTranslationKeys[pc] = labelKey;
        _parameterDescriptionTranslationKeys[pc] = descKey;
        _paramControls[tag] = pc;
        container.Controls.Add(pc);
    }

    // === EXISTING GROUPS ===
    var hydraulic = CreateGroup("ParamSectionHydraulic", _accentBlue);
    AddParamTo(hydraulic, "T", "ParamT", "ParamTDesc", 0m, 250m, 15m, "°C", 1);
    AddParamTo(hydraulic, "E", "ParamE", "ParamEDesc", -20m, 80m, 20m, "°C", 1);
    AddParamTo(hydraulic, "L", "ParamL", "ParamLDesc", 0m, 100m, 20m, "L/min", 1);
    AddParamTo(hydraulic, "V", "ParamV", "ParamVDesc", 0m, 30m, 8m, "L/min", 1);

    var systemGroup = CreateGroup("ParamSectionSystem", _accentTeal);
    AddParamTo(systemGroup, "P", "ParamP", "ParamPDesc", 0m, 5m, 0m, "bar", 2);
    AddParamTo(systemGroup, "O", "ParamO", "ParamODesc", 0m, 1m, 0m, "", 2);

    var heat = CreateGroup("ParamSectionHeat", _accentOrange);
    AddParamTo(heat, "R", "ParamR", "ParamRDesc", 0m, 50000m, 12000m, "W", 0);
    AddParamTo(heat, "F", "ParamF", "ParamFDesc", 0m, 50000m, 3000m, "W", 0);
    AddParamTo(heat, "S", "ParamS", "ParamSDesc", -50000m, 50000m, 0m, "W", 0);
    AddParamTo(heat, "D", "ParamPumpDeltaT", "ParamPumpDeltaTDesc", 0.0m, 5.0m, 0.5m, "°C/h", 2);

    var mass = CreateGroup("ParamSectionMass", _accentCyan);
    AddParamTo(mass, "A", "ParamA", "ParamADesc", 0m, 1000m, 25m, "kg", 1);
    AddParamTo(mass, "M", "ParamM", "ParamMDesc", 0.1m, 100m, 15m, "kg", 1);

    var losses = CreateGroup("ParamSectionLosses", _accentPurple);
    AddParamTo(losses, "X", "ParamX", "ParamXDesc", 0m, 100m, 2.5m, "W/K", 1);
    AddParamTo(losses, "Y", "ParamY", "ParamYDesc", 0m, 100m, 5m, "W/K", 1);

    // === NEW PARAMETERS (Pipe Mass, Pump Efficiency, Air) ===
    var advanced = CreateGroup("ParamSectionAdvanced", _accentPurple);

    pcMassaTubi = new ParameterControl
    {
        Title = T("ParamMassaTubi"),
        Description = T("ParamMassaTubiDesc"),
        Unit = "kg",
        Minimum = 1,
        Maximum = 20,
        DecimalPlaces = 1,
        Value = 6m,
        AccentColor = _accentTeal
    };
    pcMassaTubi.Tag = "MT";
    pcMassaTubi.ValueChanged += ParameterChanged;
    advanced.Controls.Add(pcMassaTubi);

    pcEfficienzaPompa = new ParameterControl
    {
        Title = T("ParamEfficienzaPompa"),
        Description = T("ParamEfficienzaPompaDesc"),
        Unit = "%",
        Minimum = 30,
        Maximum = 100,
        DecimalPlaces = 0,
        Value = 100m,
        AccentColor = _accentGreen
    };
    pcEfficienzaPompa.Tag = "EP";
    pcEfficienzaPompa.ValueChanged += ParameterChanged;
    advanced.Controls.Add(pcEfficienzaPompa);

    panel.Controls.Add(headerRow);
    panel.Controls.Add(groupsContainer);
    return panel;
}

        private static Label MakeSectionLabel(string text, Color color)
        {
            // Crea un'etichetta di sezione per i pannelli di parametro.
            return new Label
            {
                Text = text,
                AutoSize = true,
                Font = new Font("Segoe UI", 10F, FontStyle.Bold),
                ForeColor = color,
                Margin = new Padding(0, 12, 0, 6),
                Padding = new Padding(0, 4, 0, 4)
            };
        }

        // ── ALARMS ─────────────────────────────────────────────────────
        private Panel BuildAlarmsPanel()
        {
            // Costruisce la vista dedicata agli allarmi e alle loro descrizioni.
            var panel = new Panel { Dock = DockStyle.Fill, BackColor = _panel };

            var headerRow = new Panel { Dock = DockStyle.Top, Height = 52, BackColor = _panel2, Padding = new Padding(20, 14, 20, 8) };
            headerRow.Controls.Add(CreateTranslatedLabel("AlarmMonitorTitle", new Font("Segoe UI", 13F, FontStyle.Bold), _text));
            var alarmInfoLabel = CreateTranslatedLabel("AlarmUpdateInfo", new Font("Segoe UI", 8F), _muted);
            alarmInfoLabel.Margin = new Padding(20, 34, 0, 0);
            headerRow.Controls.Add(alarmInfoLabel);

            var scrollArea = new Panel { Dock = DockStyle.Fill, AutoScroll = true, BackColor = Color.Transparent, Padding = new Padding(0), Margin = new Padding(0) };

            var grid = new TableLayoutPanel
            {
                Dock = DockStyle.Top,
                AutoSize = true,
                ColumnCount = 2,
                Padding = new Padding(20),
                BackColor = _panel,
                CellBorderStyle = TableLayoutPanelCellBorderStyle.None,
                GrowStyle = TableLayoutPanelGrowStyle.AddRows,
                Margin = new Padding(0)
            };
            grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50f));
            grid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 50f));

            var alarmDescriptions = new[]
            {
                ("AlarmBoiling",        "AlarmDescBoiling",        _accentRed),
                ("AlarmDry",            "AlarmDescDry",            _accentOrange),
                ("AlarmPumpFailure",    "AlarmDescPumpFailure",    _accentRed),
                ("AlarmThermalShock",   "AlarmDescThermalShock",   _accentOrange),
                ("AlarmCirculation",    "AlarmDescCirculation",    _accentOrange),
                ("AlarmEmptyTank",      "AlarmDescEmptyTank",      _accentRed),
                ("AlarmCavitation",     "AlarmDescCavitation",     _accentOrange),
                ("AlarmOutletSensor",   "AlarmDescOutletSensor",   _accentRed),
                ("AlarmResistorOverheat","AlarmDescResistorOverheat", _accentRed),
                ("AlarmHighPressure",   "AlarmDescHighPressure",   _accentOrange),
                ("AlarmAirInCircuit",   "AlarmDescAirInCircuit",   _accentOrange),
                ("AlarmWaterLoss",      "AlarmDescWaterLoss",      _accentRed),
            };

            for (int i = 0; i < alarmDescriptions.Length; i++)
            {
                var (nameKey, descKey, alertColor) = alarmDescriptions[i];
                int bit = i;

                var cell = new RoundedPanel
                {
                    Dock = DockStyle.Fill,
                    BackColor = _card,
                    CornerRadius = 16,
                    Padding = new Padding(14, 12, 14, 12),
                    Margin = new Padding(6),
                    AutoSize = false,
                    MinimumSize = new Size(360, 100)
                };

                var cellContent = new TableLayoutPanel
                {
                    Dock = DockStyle.Fill,
                    ColumnCount = 2,
                    RowCount = 1,
                    AutoSize = false,
                    BackColor = Color.Transparent,
                    Padding = new Padding(0),
                    Margin = new Padding(0)
                };
                cellContent.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 30F));
                cellContent.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
                cellContent.RowStyles.Add(new RowStyle(SizeType.AutoSize));

                var led = new LedIndicator
                {
                    Size = new Size(18, 18),
                    OffColor = Color.FromArgb(0x3A, 0x3A, 0x5A),
                    OnColor = alertColor,
                    Margin = new Padding(0, 6, 12, 0)
                };

                var textFlow = new TableLayoutPanel
                {
                    Dock = DockStyle.Fill,
                    AutoSize = true,
                    AutoSizeMode = AutoSizeMode.GrowAndShrink,
                    ColumnCount = 1,
                    RowCount = 3,
                    BackColor = Color.Transparent,
                    Margin = new Padding(0),
                    Padding = new Padding(0)
                };
                textFlow.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100F));
                textFlow.RowStyles.Add(new RowStyle(SizeType.AutoSize));
                textFlow.RowStyles.Add(new RowStyle(SizeType.AutoSize));
                textFlow.RowStyles.Add(new RowStyle(SizeType.AutoSize));

                var nameLabel = CreateTranslatedLabel(nameKey, new Font("Segoe UI", 10F, FontStyle.Bold), _text);
                var descLabel = CreateTranslatedLabel(descKey, new Font("Segoe UI", 8F, FontStyle.Regular), _muted);
                descLabel.AutoSize = true;
                descLabel.MaximumSize = new Size(500, 0);

                var stateLabel = new Label
                {
                    Text = T("StatusHandshakeOk"),
                    AutoSize = true,
                    ForeColor = _accentGreen,
                    Font = new Font("Segoe UI", 8F, FontStyle.Bold)
                };

                led.Tag = stateLabel; // collega LED a label stato

                textFlow.Controls.Add(nameLabel);
                textFlow.Controls.Add(descLabel);
                textFlow.Controls.Add(stateLabel);
                cellContent.Controls.Add(led, 0, 0);
                cellContent.Controls.Add(textFlow, 1, 0);
                cell.Controls.Add(cellContent);

                _alarmLeds[bit] = led;
                grid.Controls.Add(cell, bit % 2, bit / 2);
            }

            scrollArea.Controls.Add(grid);
            panel.Controls.Add(scrollArea);
            panel.Controls.Add(headerRow);
            return panel;
        }

        // ── COMMANDS ─────────────────────────────────────────────────────
private Panel BuildActionsPanel()
{
    // Costruisce la pagina dei comandi con azioni rapide e diagnostica.
    var panel = new Panel { Dock = DockStyle.Fill, AutoScroll = true, BackColor = _panel };
    try
    {
        var headerRow = new Panel { Dock = DockStyle.Top, Height = 64, BackColor = _panel2, Padding = new Padding(20, 14, 20, 8) };
        headerRow.Controls.Add(CreateTranslatedLabel("ActionHeader", new Font("Segoe UI", 13F, FontStyle.Bold), _text));
        panel.Controls.Add(headerRow);

        var modulesGrid = new TableLayoutPanel
        {
            Dock = DockStyle.Top,
            AutoSize = true,
            ColumnCount = 3,
            RowCount = 1,
            Padding = new Padding(24),
            BackColor = _panel
        };
        modulesGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.33f));
        modulesGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.33f));
        modulesGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.33f));
        modulesGrid.RowStyles.Add(new RowStyle(SizeType.AutoSize));

        // Helper to create module card
        RoundedPanel CreateModuleCard(string titleKey, Color accentColor)
        {
            var card = new RoundedPanel
            {
                BackColor = _card,
                CornerRadius = 16,
                Padding = new Padding(16),
                Margin = new Padding(12),
                AutoSize = true,
                AutoSizeMode = AutoSizeMode.GrowOnly,
                MinimumSize = new Size(300, 520),
                MaximumSize = new Size(400, 0)
            };
            var accent = new Panel
            {
                Height = 3,
                Dock = DockStyle.Top,
                BackColor = accentColor,
                Margin = new Padding(0, 0, 0, 10)
            };
            var titleLbl = CreateTranslatedLabel(titleKey, new Font("Segoe UI", 10F, FontStyle.Bold), accentColor, true, new Padding(0, 10, 0, 12));
            card.Controls.Add(accent);
            card.Controls.Add(titleLbl);
            return card;
        }

        // ==================== WATER CARD ====================
        var waterCard = CreateModuleCard("ActionsWaterTitle", _accentGreen);
        btnWater = CreateCmdButton("ButtonWater", _accentGreen, 280);
        btnLoss = CreateCmdButton("ButtonLoss", _accentOrange, 280);
        btnReset = CreateCmdButton("ButtonReset", _accentRed, 280);
        btnSwitchFluid = CreateCmdButton("ButtonSwitchFluidWater", Color.FromArgb(0x4D, 0x5D, 0x9D), 280);
        btnPumpWater = CreateCmdButton("ButtonPumpWater", Color.FromArgb(0x2A, 0xA1, 0xA8), 280);

        btnWater.Click += (_, _) => SendCommand("ACQUA");
        btnLoss.Click += (_, _) => SendCommand("PERDITA");
        btnReset.Click += (_, _) => SendCommand("RIPRISTINA");
        btnSwitchFluid.Click += BtnSwitchFluid_Click;
        btnPumpWater.Click += (_, _) => SendCommand("H2O");

        var waterStack = new FlowLayoutPanel
        {
            Dock = DockStyle.Top,
            AutoSize = true,
            FlowDirection = FlowDirection.TopDown,
            WrapContents = false,
            Margin = new Padding(0, 0, 0, 6),
            BackColor = Color.Transparent
        };
        void addWaterBtn(Button? wb) { if (wb == null) return; wb.Dock = DockStyle.None; wb.Width = 260; wb.Height = 42; waterStack.Controls.Add(wb); }
        addWaterBtn(btnWater);
        addWaterBtn(btnLoss);
        addWaterBtn(btnReset);
        addWaterBtn(btnSwitchFluid);
        addWaterBtn(btnPumpWater);
        waterCard.Controls.Add(waterStack);
        modulesGrid.Controls.Add(waterCard, 0, 0);

        // ==================== DIAGNOSTIC CARD (FINAL WITH TABLELAYOUT) ====================
        var diagCard = CreateModuleCard("ActionsDiagTitle", _accentCyan);

        // Main layout: single column table
        var diagLayout = new TableLayoutPanel
        {
            Dock = DockStyle.Top,
            AutoSize = true,
            ColumnCount = 1,
            RowCount = 0,
            BackColor = Color.Transparent,
            Margin = new Padding(0, 0, 0, 12),
            Padding = new Padding(0)
        };
        diagLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f));

        // Create buttons (if not already initialized)
        btnPompaAuto = CreateCmdButton("ButtonAutoPump", Color.FromArgb(0x2D, 0x6A, 0xA0), 260);
        btnRaffreddamentoOn = CreateCmdButton("ButtonCoolOn", _accentCyan, 260);
        btnRaffreddamentoOff = CreateCmdButton("ButtonCoolOff", _accentRed, 260);
        btnStato = CreateCmdButton("ButtonStatus", _accentPurple, 260);
        btnMenu = CreateCmdButton("ButtonMenu", Color.FromArgb(0x55, 0x55, 0x99), 260);
        btnPompaOn = CreateCmdButton("ButtonPumpOn", _accentGreen, 125);
        btnPompaOff = CreateCmdButton("ButtonPumpOff", _accentRed, 125);

        // Events
        btnPompaAuto.Click += (_, _) => SendCommand("POMPA AUTO");
        btnRaffreddamentoOn.Click += (_, _) => SendCommand("RAFFREDDAMENTO");
        btnRaffreddamentoOff.Click += (_, _) => SendCommand("STOP");
        btnStato.Click += (_, _) => SendCommand("STATO");
        btnMenu.Click += (_, _) => SendCommand("MENU");
        btnPompaOn.Click += (_, _) => SendCommand("POMPA ON");
        btnPompaOff.Click += (_, _) => SendCommand("POMPA OFF");

        void AddButtonRow(Button btn)
        {
            btn.Width = 260;
            btn.Height = 42;
            btn.Margin = new Padding(0, 4, 0, 4);
            diagLayout.RowCount++;
            diagLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            diagLayout.Controls.Add(btn, 0, diagLayout.RowCount - 1);
        }

        AddButtonRow(btnPompaAuto);
        AddButtonRow(btnRaffreddamentoOn);
        AddButtonRow(btnRaffreddamentoOff);
        AddButtonRow(btnStato);
        AddButtonRow(btnMenu);

        // Pump ON/OFF row
        var pumpRow = new FlowLayoutPanel
        {
            FlowDirection = FlowDirection.LeftToRight,
            WrapContents = false,
            AutoSize = true,
            Margin = new Padding(0, 8, 0, 12),
            BackColor = Color.Transparent
        };
        btnPompaOn.Height = 42;
        btnPompaOff.Height = 42;
        pumpRow.Controls.Add(btnPompaOn);
        pumpRow.Controls.Add(btnPompaOff);
        diagLayout.RowCount++;
        diagLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        diagLayout.Controls.Add(pumpRow, 0, diagLayout.RowCount - 1);

        // Separator
        var maintSeparator = new Panel
        {
            Height = 2,
            BackColor = Color.FromArgb(55, 55, 75),
            Margin = new Padding(0, 16, 0, 8),
            Dock = DockStyle.None
        };
        diagLayout.RowCount++;
        diagLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        diagLayout.Controls.Add(maintSeparator, 0, diagLayout.RowCount - 1);

        // Maintenance Title
        var maintTitle = CreateTranslatedLabel("MaintenanceTitle", new Font("Segoe UI", 11F, FontStyle.Bold), _accentPurple);
        maintTitle.Margin = new Padding(6, 8, 0, 4);
        diagLayout.RowCount++;
        diagLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        diagLayout.Controls.Add(maintTitle, 0, diagLayout.RowCount - 1);

        // Maintenance info
        lblManutenzioneInfo = new Label
        {
            Text = T("MaintenanceTotalHours"),
            ForeColor = _muted,
            Font = new Font("Segoe UI", 9.5F),
            AutoSize = true,
            Margin = new Padding(6, 0, 0, 4)
        };
        diagLayout.RowCount++;
        diagLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        diagLayout.Controls.Add(lblManutenzioneInfo, 0, diagLayout.RowCount - 1);

        // Maintenance details
        var maintDetails = new FlowLayoutPanel
        {
            FlowDirection = FlowDirection.TopDown,
            WrapContents = false,
            AutoSize = true,
            BackColor = Color.Transparent,
            Margin = new Padding(6, 4, 0, 4)
        };
        lblOrePompa = new Label { Text = T("MaintenancePumpDefault"), ForeColor = _text, AutoSize = true, Margin = new Padding(0, 2, 0, 2) };
        lblOreResistenza = new Label { Text = T("MaintenanceResistorDefault"), ForeColor = _text, AutoSize = true, Margin = new Padding(0, 2, 0, 2) };
        lblOreHx = new Label { Text = T("MaintenanceHxDefault"), ForeColor = _text, AutoSize = true, Margin = new Padding(0, 2, 0, 2) };
        lblUsuraPompa = new Label
        {
            Text = T("MaintenanceWearDefault"),
            ForeColor = _accentOrange,
            Font = new Font("Segoe UI", 10F, FontStyle.Bold),
            AutoSize = true,
            Margin = new Padding(0, 8, 0, 2)
        };
        maintDetails.Controls.Add(lblOrePompa);
        maintDetails.Controls.Add(lblOreResistenza);
        maintDetails.Controls.Add(lblOreHx);
        maintDetails.Controls.Add(lblUsuraPompa);
        diagLayout.RowCount++;
        diagLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
        diagLayout.Controls.Add(maintDetails, 0, diagLayout.RowCount - 1);

        diagCard.Controls.Add(diagLayout);
        modulesGrid.Controls.Add(diagCard, 1, 0);

        // ==================== FAULT CARD ====================
        var faultCard = CreateModuleCard(T("FaultTitle"), _accentOrange);
        var faultFlow = new FlowLayoutPanel
        {
            Dock = DockStyle.Top,
            AutoSize = true,
            FlowDirection = FlowDirection.TopDown,
            WrapContents = false,
            Margin = new Padding(0, 12, 0, 0),
            BackColor = Color.Transparent
        };

        faultCheckBoxes = new CheckBox[12];
        var faultDefs = new[]
        {
            ("1", "FaultError1", _accentOrange),
            ("2", "FaultError2", _accentOrange),
            ("3", "FaultError3", _accentRed),
            ("4", "FaultError4", _accentOrange),
            ("5", "FaultError5", _accentRed),
            ("6", "FaultError6", _accentRed),
            ("7", "FaultError7", _accentRed),
            ("8", "FaultError8", _accentOrange),
            ("9", "FaultError9", _accentRed),
            ("10", "FaultError10", _accentRed),
            ("11", "FaultError11", _accentRed),
            ("12", "FaultError12", _accentRed),
        };
        for (int i = 0; i < faultDefs.Length; i++)
        {
            var (cmd, nameKey, fColor) = faultDefs[i];
            var chk = new CheckBox
            {
                Text = $"  {cmd} — {T(nameKey)}",
                ForeColor = _text,
                Font = new Font("Segoe UI", 9F),
                AutoSize = true,
                Margin = new Padding(0, 6, 0, 6)
            };
            chk.CheckedChanged += (_, _) =>
            {
                chk.ForeColor = chk.Checked ? fColor : _text;
                SendCommand($"{cmd} {(chk.Checked ? "ON" : "OFF")}");
            };
            _faultTranslationKeys[chk] = (cmd, nameKey, fColor);
            faultCheckBoxes[i] = chk;
            faultFlow.Controls.Add(chk);
        }
        faultCard.Controls.Add(faultFlow);
        modulesGrid.Controls.Add(faultCard, 2, 0);

        panel.Controls.Add(modulesGrid);
    }
    catch (Exception ex)
    {
        try
        {
            var log = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "actions_error.log");
            File.AppendAllText(log, ex.ToString() + "\r\n");
        }
        catch { }
    }
    return panel;
}

        // ── CONSOLE ─────────────────────────────────────────────────────
        private Panel BuildConsolePanel()
        {
            // Costruisce la pagina della console seriale con input personalizzato
            // e riferimento ai comandi disponibili.
            var panel = new Panel { Dock = DockStyle.Fill, BackColor = _panel };

            var headerRow = new Panel { Dock = DockStyle.Top, Height = 52, BackColor = _panel2, Padding = new Padding(20, 14, 20, 8) };
            headerRow.Controls.Add(CreateTranslatedLabel("ConsoleTitle", new Font("Segoe UI", 13F, FontStyle.Bold), _text));

            var inputCard = new RoundedPanel
            {
                BackColor = _card,
                CornerRadius = 16,
                Padding = new Padding(20),
                Dock = DockStyle.Top,
                AutoSize = true,
                Margin = new Padding(20, 16, 20, 12)
            };

            var inputLayout = new TableLayoutPanel
            {
                Dock = DockStyle.Top,
                AutoSize = true,
                ColumnCount = 2,
                RowCount = 3,
                BackColor = Color.Transparent,
                Padding = new Padding(0),
            };
            inputLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100f));
            inputLayout.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 140f));
            inputLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            inputLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            inputLayout.RowStyles.Add(new RowStyle(SizeType.AutoSize));

            var cmdLabel = CreateTranslatedLabel("CommandLabel", new Font("Segoe UI", 9F, FontStyle.Bold), _muted);
            txtCommand = new TextBox
            {
                Height = 36,
                Font = new Font("Consolas", 11F),
                BackColor = _panel2,
                ForeColor = Color.FromArgb(0x7A, 0xE2, 0xAD),
                BorderStyle = BorderStyle.FixedSingle,
                Dock = DockStyle.Fill,
                Margin = new Padding(0, 6, 8, 0)
            };
            txtCommand.KeyDown += (s, e) => { if (e.KeyCode == Keys.Enter) { SendCustomCommand(); e.SuppressKeyPress = true; } };

            btnSendCommand = CreateCmdButton("ButtonSend", _accentGreen, 120);
            btnSendCommand.Height = 36;
            btnSendCommand.Dock = DockStyle.Fill;
            btnSendCommand.Margin = new Padding(0, 6, 0, 0);
            btnSendCommand.Click += (_, _) => SendCustomCommand();

            var helpLabel = CreateTranslatedLabel("ConsoleHelp", new Font("Segoe UI", 8F), _muted);
            helpLabel.AutoSize = true;
            helpLabel.Margin = new Padding(0, 10, 0, 0);
            helpLabel.Padding = new Padding(0);

            inputLayout.Controls.Add(cmdLabel, 0, 0);
            inputLayout.SetColumnSpan(cmdLabel, 2);
            inputLayout.Controls.Add(txtCommand, 0, 1);
            inputLayout.Controls.Add(btnSendCommand, 1, 1);
            inputLayout.Controls.Add(helpLabel, 0, 2);
            inputLayout.SetColumnSpan(helpLabel, 2);

            inputCard.Controls.Add(inputLayout);

            // Quick reference card
            var refCard = new RoundedPanel
            {
                BackColor = _card,
                CornerRadius = 16,
                Padding = new Padding(20),
                Dock = DockStyle.Top,
                AutoSize = true,
                Margin = new Padding(20, 0, 20, 0)
            };
            var refTitle = CreateTranslatedLabel("CommandReferenceTitle", new Font("Segoe UI", 10F, FontStyle.Bold), _accentBlue);

            var refGrid = new TableLayoutPanel
            {
                Dock = DockStyle.Top,
                AutoSize = true,
                ColumnCount = 3,
                RowCount = 1,
                BackColor = Color.Transparent
            };
            refGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.3f));
            refGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.3f));
            refGrid.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.3f));

            static Label RefCol(string text, Color c) => new Label
            {
                Text = text,
                AutoSize = false,
                Dock = DockStyle.Fill,
                ForeColor = c,
                Font = new Font("Consolas", 9F),
                TextAlign = ContentAlignment.TopLeft,
                Padding = new Padding(0, 4, 0, 0)
            };

            var refColumn1 =
                T("ConsoleHelpMenu") + "\n" +
                T("ConsoleHelpStatus") + "\n" +
                T("ConsoleHelpHandshake") + "\n\n" +
                T("ConsoleHelpPumpAuto") + "\n" +
                T("ConsoleHelpPumpOn") + "\n" +
                T("ConsoleHelpPumpOff") + "\n\n" +
                T("ConsoleHelpCooling") + "\n" +
                T("ConsoleHelpStop");

            var refColumn2 =
                T("ConsoleHelpWater") + "\n" +
                T("ConsoleHelpLoss") + "\n" +
                T("ConsoleHelpReset") + "\n\n" +
                T("ConsoleHelpOil") + "\n" +
                T("ConsoleHelpH2O") + "\n\n" +
                T("ConsoleHelpFault");

            var refColumn3 =
                T("ConsoleHelpNumericParams") + "\n" +
                T("ConsoleHelpT") + "\n" +
                T("ConsoleHelpE") + "\n" +
                T("ConsoleHelpL") + "\n" +
                T("ConsoleHelpA") + "\n" +
                T("ConsoleHelpM") + "\n" +
                T("ConsoleHelpR") + "\n" +
                T("ConsoleHelpF") + "\n" +
                T("ConsoleHelpS") + "\n" +
                T("ConsoleHelpP") + "\n" +
                T("ConsoleHelpO") + "\n" +
                T("ConsoleHelpHX") + "\n" +
                T("ConsoleHelpK") + "\n" +
                T("ConsoleHelpG");

            refGrid.Controls.Add(RefCol(refColumn1, _accentCyan), 0, 0);
            refGrid.Controls.Add(RefCol(refColumn2, _accentOrange), 1, 0);
            refGrid.Controls.Add(RefCol(refColumn3, _accentGreen), 2, 0);

            refCard.Controls.Add(refTitle);
            refCard.Controls.Add(refGrid);

            panel.Controls.Add(refCard);
            panel.Controls.Add(inputCard);
            panel.Controls.Add(headerRow);
            return panel;
        }

        private void SendCustomCommand()
        {
            // Invia il comando digitato nell'input della console.
            string cmd = txtCommand.Text.Trim();
            if (string.IsNullOrEmpty(cmd)) return;
            SendCommand(cmd);
            txtCommand.Clear();
            txtCommand.Focus();
        }

        // ── HELPERS ─────────────────────────────────────────────────────
        // Ritorna la stringa tradotta per la chiave specificata nella lingua corrente.
        private string T(string key) => Translations.Get(_language, key);

        private Label CreateTranslatedLabel(string key, Font font, Color foreColor, bool autoSize = true, Padding? margin = null)
        {
            // Crea un label che utilizza la traduzione parziale registrata, e memorizza
            // la chiave per poter aggiornare l'interfaccia quando cambia lingua.
            var label = new Label
            {
                Text = T(key),
                AutoSize = autoSize,
                Font = font,
                ForeColor = foreColor,
                Margin = margin ?? new Padding(0),
                AutoEllipsis = true,
                TextAlign = ContentAlignment.MiddleLeft
            };
            if (!autoSize)
            {
                label.Dock = DockStyle.Top;
            }
            _labelTranslationKeys[label] = key;
            return label;
        }

        private Button CreateTranslatedButton(string key, Color backColor, int width, int height = 44)
        {
            // Crea un pulsante con testo tradotto e grafica personalizzata.
            var dimBack = Color.FromArgb(
                (int)(backColor.R * 0.18f + 0x0A),
                (int)(backColor.G * 0.18f + 0x0A),
                (int)(backColor.B * 0.18f + 0x12));
            var borderCol = Color.FromArgb(
                Math.Min(255, (int)(backColor.R * 0.55f)),
                Math.Min(255, (int)(backColor.G * 0.55f)),
                Math.Min(255, (int)(backColor.B * 0.55f)));

            var btn = new Button
            {
                Text = T(key),
                Width  = width,
                Height = height,
                BackColor = dimBack,
                ForeColor = backColor,          
                FlatStyle = FlatStyle.Flat,
                FlatAppearance = { BorderSize = 1, BorderColor = borderCol },
                Font = new Font("Segoe UI", 9.5F, FontStyle.Bold),
                Margin = new Padding(0, 4, 0, 4),
                TextAlign = ContentAlignment.MiddleLeft,
                Padding = new Padding(14, 0, 0, 0)
            };
            _buttonTranslationKeys[btn] = key;
            return btn;
        }

        private Button CreateNavButton(string key, Color backColor)
        {
            var btn = new Button
            {
                Text = T(key),
                Width = 264,
                Height = 48,
                BackColor = Color.FromArgb(0x1A, 0x28, 0x45),
                ForeColor = Color.FromArgb(0xAA, 0xBB, 0xCC),
                FlatStyle = FlatStyle.Flat,
                FlatAppearance = { BorderSize = 0 },
                Font = new Font("Segoe UI", 10F, FontStyle.Bold),
                Margin = new Padding(0, 2, 0, 2),
                TextAlign = ContentAlignment.MiddleLeft,
                Padding = new Padding(12, 0, 0, 0),
                Tag = backColor
            };
            _buttonTranslationKeys[btn] = key;
            btn.MouseEnter += (s, _) => { if (_activeNavBtn != btn) btn.BackColor = Color.FromArgb(0x22, 0x35, 0x58); };
            btn.MouseLeave += (s, _) => { if (_activeNavBtn != btn) btn.BackColor = Color.FromArgb(0x1A, 0x28, 0x45); };
            return btn;
        }

        private void ApplyTranslations()
        {
            // Aggiorna tutte le etichette, pulsanti e controlli registrati
            // con la traduzione corrente della lingua.
            // Aggiorna tutte le Label registrate con la traduzione corrente.
            foreach (var pair in _labelTranslationKeys)
                if (pair.Key != null)
                    pair.Key.Text = T(pair.Value);

            // Aggiorna tutti i Button registrati con la traduzione corrente.
            foreach (var pair in _buttonTranslationKeys)
                if (pair.Key != null)
                    pair.Key.Text = T(pair.Value);

            // Aggiorna tutte le ValueCard con i titoli tradotti.
            foreach (var pair in _cardTranslationKeys)
                if (pair.Key != null)
                    pair.Key.Title = T(pair.Value);

            // Aggiorna i titoli dei ParameterControl.
            foreach (var pair in _parameterTranslationKeys)
                if (pair.Key != null)
                    pair.Key.Title = T(pair.Value);

            // Aggiorna le descrizioni dei ParameterControl.
            foreach (var pair in _parameterDescriptionTranslationKeys)
                if (pair.Key != null)
                    pair.Key.Description = T(pair.Value);

            // Aggiorna i checkbox dei guasti con testo e colore corretti.
            foreach (var pair in _faultTranslationKeys)
            {
                if (pair.Key == null) continue;
                pair.Key.Text = $"  {pair.Value.Command} — {T(pair.Value.Key)}";
                pair.Key.ForeColor = pair.Key.Checked ? pair.Value.CheckedColor : _text;
            }

            // Switch Fluid Button
            if (btnSwitchFluid != null)
                btnSwitchFluid.Text = T(_isOilMode ? "ButtonSwitchFluidWater" : "ButtonSwitchFluidOil");

            // ComboBox dispositivo
            if (cmbDevice.Items.Count >= 2)
            {
                int index = cmbDevice.SelectedIndex;
                cmbDevice.Items[0] = T("DeviceEsp32");
                cmbDevice.Items[1] = T("DeviceCustom");
                cmbDevice.SelectedIndex = Math.Max(0, Math.Min(index, cmbDevice.Items.Count - 1));
            }

            // Pulsante cambio lingua
        if (btnLanguageToggle != null)
        {
            btnLanguageToggle.Text = GetLanguageButtonText();
        }
            UpdateTelemetryHeader();
            UpdateFooterStatus();

            // Force redraw of cards
            foreach (var card in _dashboardCards.Values)
                card?.Invalidate();
        }

        private void UpdateLanguage(Language language)
        {
            // Cambia la lingua dell'interfaccia e riapplica tutte le traduzioni.
            _language = language;
            ApplyTranslations();
        }

        private void ToggleLanguage()
        {
            // Alterna tra italiano e inglese.
            UpdateLanguage(_language == Language.Italian ? Language.English : Language.Italian);
        }

        private string GetLanguageButtonText()
        {
            // Ritorna il testo corretto per il pulsante di cambio lingua.
            return _language == Language.Italian ? T("LangButtonItalian") : T("LangButtonEnglish");
        }

        private string GetTranslatedHandshakeStatus()
        {
            // Converte lo stato handshake interno nel testo tradotto.
            return _handshakeStatus switch
            {
                "OK" => T("StatusHandshakeOk"),
                "FAILED" => T("StatusHandshakeFailed"),
                _ => T("HandshakeNone")
            };
        }

        private Button CreateCmdButton(string key, Color backColor, int width)
        {
            return CreateTranslatedButton(key, backColor, width, 44);
        }

        private void SetActiveNav(Button btn)
        {
            // Cambia lo stato attivo della navigazione nella sidebar.
            if (_activeNavBtn != null)
            {
                _activeNavBtn.BackColor = Color.FromArgb(0x1A, 0x28, 0x45);
                _activeNavBtn.ForeColor = Color.FromArgb(0xAA, 0xBB, 0xCC);
                _activeNavBtn.FlatAppearance.BorderSize = 0;
                _activeNavBtn.Padding = new Padding(12, 0, 0, 0);
            }
            _activeNavBtn = btn;
            if (btn.Tag is Color c)
            {
                btn.BackColor = Color.FromArgb(
                    (int)(c.R * 0.12f + 0x10),
                    (int)(c.G * 0.12f + 0x1A),
                    (int)(c.B * 0.12f + 0x30));
                btn.ForeColor = c;
                // Colored left border as visual indicator of active element
                btn.FlatAppearance.BorderColor = c;
                btn.FlatAppearance.BorderSize  = 0;
                btn.Padding = new Padding(9, 0, 0, 0); // space for simulated left border
            }
        }

        private RoundedPanel MakeCommandCard(int height = 90)
        {
            // Crea un contenitore con stile a card per i comandi.
            return new RoundedPanel
            {
                BackColor = _card,
                CornerRadius = 14,
                Height = height,
                Margin = new Padding(0, 0, 0, 8),
                Dock = DockStyle.Top
            };
        }

        /// <summary>
        /// Restituisce il nome atteso del dispositivo da verificare durante l'handshake.
        /// </summary>
        private static string GetExpectedNameForDevice(int deviceIndex) => deviceIndex switch
        {
            0 => "TERMOREGOLATORE",   // Temperature Controller (ESP32)
            // 1 => "OTHER_NAME",     // ← add next device here
            _ => "TERMOREGOLATORE"
        };

        private void ShowView(Panel view)
        {
            // Mostra una sola vista / pagina nella zona contenuti e nasconde tutte le altre.
            foreach (Control child in _contentPanel.Controls)
                child.Visible = false;
            view.Visible = true;
        }

        // ── TELEMETRY DTO ────────────────────────────────────────────────
        // Classe per deserializzare i dati JSON provenienti dal dispositivo.
        private sealed class TelemetryDto
        {
            [JsonPropertyName("pompa")] public int? Pompa { get; set; }
            [JsonPropertyName("freddo")] public int? Freddo { get; set; }
            [JsonPropertyName("riemp")] public int? Riemp { get; set; }
            [JsonPropertyName("serb")] public double? Serb { get; set; }
            [JsonPropertyName("met")] public double? Met { get; set; }
            [JsonPropertyName("mand")] public double? Mand { get; set; }
            [JsonPropertyName("rit")] public double? Rit { get; set; }
            [JsonPropertyName("stampo")] public double? Stampo { get; set; }
            [JsonPropertyName("res")] public double? Res { get; set; }
            [JsonPropertyName("nucl")] public double? Nucl { get; set; }
            [JsonPropertyName("acqua")] public double? Acqua { get; set; }
            [JsonPropertyName("port")] public double? Port { get; set; }
            [JsonPropertyName("pres")] public double? Pres { get; set; }
            [JsonPropertyName("qhx")] public double? Qhx { get; set; }
            [JsonPropertyName("qraff")] public double? Qraff { get; set; }
            [JsonPropertyName("qfill")] public double? Qfill { get; set; }
            [JsonPropertyName("qconv")] public double? Qconv { get; set; }
            [JsonPropertyName("qproc")] public double? Qproc { get; set; }
            [JsonPropertyName("qpump")] public double? Qpump { get; set; }
            [JsonPropertyName("alm")] public int? Alm { get; set; }
            [JsonPropertyName("ore_pompa")] public double? OrePompa { get; set; }
            [JsonPropertyName("ore_res")]   public double? OreRes { get; set; }
            [JsonPropertyName("ore_hx")]    public double? OreHx { get; set; }
            [JsonPropertyName("usura")]     public double? UsuraPompa { get; set; }
        }
    }
}