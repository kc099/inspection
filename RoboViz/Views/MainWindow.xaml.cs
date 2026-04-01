using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Threading;
using Microsoft.Win32;

namespace RoboViz
{
    public partial class MainWindow : Window
    {
        private InspectionService? _service;
        private ModbusService? _modbus;
        private TriggerService? _triggerService;
        private Bitmap? _currentImage;
        private CameraManager? _cameraManager;
        private DispatcherTimer? _streamTimer;
        private bool _isStreaming;

        private System.Windows.Controls.Image[] _imageDisplays = null!;
        private TextBlock[] _verdictLabels = null!;
        private TextBlock[] _scoreLabels = null!;
        private TextBlock[] _camLabels = null!;
        private const int FrameCount = 4;
        private int _activeFrameCount = 4;

        // Per-camera configuration (set via CameraSetupDialog)
        private CameraSlotConfig[] _cameraSlots =
        [
            new() { Slot = 0, TriggerGroup = 1, Detector = "MaskRCNN",  CaptureDelayMs = 50 },
            new() { Slot = 1, TriggerGroup = 1, Detector = "MaskRCNN",  CaptureDelayMs = 50 },
            new() { Slot = 2, TriggerGroup = 1, Detector = "PatchCore", CaptureDelayMs = 50 },
            new() { Slot = 3, TriggerGroup = 2, Detector = "PatchCore", CaptureDelayMs = 50 },
        ];

        // Frame sequence tracking for stream preview optimization
        private readonly long[] _lastDisplayedSequence = new long[4];
        private long _streamFrameCount;
        private long _streamSkipCount;

        private static readonly SolidColorBrush PassGreen = new(System.Windows.Media.Color.FromRgb(76, 175, 80));
        private static readonly SolidColorBrush FailRed = new(System.Windows.Media.Color.FromRgb(244, 67, 54));
        private static readonly SolidColorBrush NormalGray = new(System.Windows.Media.Color.FromRgb(200, 200, 200));
        private static readonly SolidColorBrush DimGray = new(System.Windows.Media.Color.FromRgb(110, 110, 118));

        private static readonly SolidColorBrush VerdictPassBg = new(System.Windows.Media.Color.FromRgb(27, 94, 32));
        private static readonly SolidColorBrush VerdictReworkBg = new(System.Windows.Media.Color.FromRgb(230, 126, 34));
        private static readonly SolidColorBrush VerdictRejectBg = new(System.Windows.Media.Color.FromRgb(211, 47, 47));
        private static readonly SolidColorBrush VerdictNeutralBg = new(System.Windows.Media.Color.FromRgb(55, 55, 64));
        private static readonly SolidColorBrush VerdictErrorBg = new(System.Windows.Media.Color.FromRgb(120, 60, 60));
        private static readonly SolidColorBrush ReadyGreen = new(System.Windows.Media.Color.FromRgb(100, 220, 100));
        private static readonly SolidColorBrush WarningAmber = new(System.Windows.Media.Color.FromRgb(255, 183, 77));
        private static readonly SolidColorBrush ErrorRed = new(System.Windows.Media.Color.FromRgb(255, 80, 80));
        private static readonly SolidColorBrush StreamStopRed = new(System.Windows.Media.Color.FromRgb(198, 40, 40));
        private static readonly SolidColorBrush StreamPurple = new(System.Windows.Media.Color.FromRgb(106, 27, 154));

        private static readonly Dictionary<string, SolidColorBrush> VerdictColorMap = new()
        {
            ["PASS"] = PassGreen,
            ["REWORK"] = VerdictReworkBg,
            ["REJECT"] = FailRed,
            ["ERROR"] = VerdictErrorBg,
        };

        static MainWindow()
        {
            PassGreen.Freeze(); FailRed.Freeze(); NormalGray.Freeze(); DimGray.Freeze();
            VerdictPassBg.Freeze(); VerdictReworkBg.Freeze(); VerdictRejectBg.Freeze();
            VerdictNeutralBg.Freeze(); VerdictErrorBg.Freeze();
            ReadyGreen.Freeze(); WarningAmber.Freeze(); ErrorRed.Freeze();
            StreamStopRed.Freeze(); StreamPurple.Freeze();
        }

        private readonly DispatcherTimer _clockTimer;

        public MainWindow()
        {
            InitializeComponent();
            _imageDisplays = [ImageDisplay0, ImageDisplay1, ImageDisplay2, ImageDisplay3];
            _verdictLabels = [VerdictLabel0, VerdictLabel1, VerdictLabel2, VerdictLabel3];
            _scoreLabels = [ScoreLabel0, ScoreLabel1, ScoreLabel2, ScoreLabel3];
            _camLabels = [CamLabel0, CamLabel1, CamLabel2, CamLabel3];

            _clockTimer = new DispatcherTimer { Interval = TimeSpan.FromSeconds(1) };
            _clockTimer.Tick += (_, _) => DateTimeText.Text = DateTime.Now.ToString("yyyy-MM-dd  HH:mm:ss");
            _clockTimer.Start();
            DateTimeText.Text = DateTime.Now.ToString("yyyy-MM-dd  HH:mm:ss");
        }

        private async void Window_Loaded(object sender, RoutedEventArgs e)
        {
            StatusText.Text = "Model: loading...";
            var progress = new Progress<string>(status => 
            {
                StatusText.Text = status;
                // Force UI update during long GPU warmup
                Dispatcher.Invoke(() => { }, System.Windows.Threading.DispatcherPriority.Render);
            });

            try
            {
                // Use longer timeout for PCIe Gen2 systems (up to 3 minutes)
                var loadTask = Task.Run(() =>
                {
                    string modelPath = Path.Combine(
                        AppDomain.CurrentDomain.BaseDirectory, "Assets", "maskrcnn_oring.onnx");

                    if (!File.Exists(modelPath))
                        throw new FileNotFoundException("ONNX model not found at: " + modelPath);

                    _service = new InspectionService(modelPath, scoreThreshold: 0.5f,
                        useGpu: true, progress: progress, modelName: "Model 2");
                });

                // Wait with extended timeout for PCIe Gen2 + older CPU
                if (await Task.WhenAny(loadTask, Task.Delay(TimeSpan.FromMinutes(3))) == loadTask)
                {
                    await loadTask; // Propagate any exceptions
                }
                else
                {
                    throw new TimeoutException(
                        "Model loading timed out after 3 minutes. GPU may be too slow or CUDA libraries incompatible. Check gpu_init.log for details.");
                }

                StatusText.Text = $"Model: ready  [{_service!.ActiveProvider}]  ({_service.CurrentModel})";
                StatusText.Foreground = ReadyGreen;

                VerdictText.Text = "READY";
                VerdictText.Foreground = ReadyGreen;
                VerdictBorder.Background = VerdictNeutralBg;

                if (_service.GpuError != null)
                {
                    StatusText.Foreground = WarningAmber;
                    DetailsText.Text = $"GPU fallback: {_service.GpuError}";
                }

                PopulateMetricsTable();
                UpdateCameraConfigSummary();

                // Initialize cameras (enumerate devices) but don't start streaming yet
                await InitializeCamerasAsync(progress);

                // Auto-start trigger mode (cameras will start streaming on-demand when trigger fires)
                AutoStartTrigger();
            }
            catch (Exception ex)
            {
                StatusText.Text = "Model: failed to load";
                StatusText.Foreground = ErrorRed;
                DetailsText.Text = $"{ex.Message}\n\nCheck gpu_init.log in application directory for detailed diagnostics.";
                
                // Log the full exception
                var logPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "gpu_init.log");
                try
                {
                    File.AppendAllText(logPath, $"\n\n=== MainWindow Exception {DateTime.Now:O} ===\n{ex}\n");
                }
                catch { }
            }
        }

        /// <summary>
        /// Auto-start camera streaming at app launch. Failures are non-fatal (logged to DetailsText).
        /// </summary>
        private async Task AutoStartCamerasAsync(IProgress<string>? progress)
        {
            try
            {
                progress?.Report("Initializing cameras...");
                await StartCameraStreamAsync(progress);
            }
            catch (Exception ex)
            {
                DetailsText.Text += $"\nCamera auto-start failed: {ex.Message}";
                MaskRCNNDetector.LogDiag($"[AutoStart] Camera init failed: {ex.Message}");
            }
        }

        /// <summary>
        /// Initialize camera manager and enumerate devices WITHOUT starting streaming.
        /// Cameras will start on-demand when first trigger fires.
        /// </summary>
        private async Task InitializeCamerasAsync(IProgress<string>? progress)
        {
            try
            {
                progress?.Report("Enumerating cameras...");
                _cameraManager = new CameraManager();

                int found = await Task.Run(() => _cameraManager.Initialize());
                if (found == 0)
                {
                    progress?.Report("No cameras found");
                    DetailsText.Text = "No cameras detected. Trigger mode will run without cameras (signal only).";
                    return;
                }

                var descriptions = _cameraManager.GetCameraDescriptions();
                DetailsText.Text = $"Found {found} camera(s) (ready for triggers):\n" + string.Join("\n", descriptions);
                Debug.WriteLine($"[CameraInit] Found {found} camera(s), ready for on-demand capture.");
                progress?.Report($"Cameras ready: {found} device(s)");
            }
            catch (Exception ex)
            {
                DetailsText.Text += $"\nCamera enumeration failed: {ex.Message}";
                MaskRCNNDetector.LogDiag($"[CameraInit] Enumeration failed: {ex.Message}");
                Debug.WriteLine($"[CameraInit] ERROR: {ex.Message}");
            }
        }

        /// <summary>
        /// Auto-start trigger mode at app launch. Failures are non-fatal (logged to TriggerStatusText).
        /// </summary>
        private void AutoStartTrigger()
        {
            try
            {
                StartTriggerMode();
            }
            catch (Exception ex)
            {
                TriggerStatusText.Text = $"Auto-start failed: {ex.Message}";
                TriggerStatusText.Foreground = ErrorRed;
                DetailsText.Text += $"\n\nTRIGGER ERROR: {ex.Message}";
                MaskRCNNDetector.LogDiag($"[AutoStart] Trigger init failed: {ex}");
                Debug.WriteLine($"[AutoStart] Trigger FAILED: {ex.Message}");
            }
        }

        /// <summary>
        /// Shared logic: initialize cameras, start streaming, start live-preview timer.
        /// Returns true if cameras started successfully.
        /// </summary>
        private async Task<bool> StartCameraStreamAsync(IProgress<string>? progress)
        {
            if (_isStreaming) return true;

            _cameraManager ??= new CameraManager();

            int found = await Task.Run(() => _cameraManager.Initialize());
            if (found == 0)
            {
                progress?.Report("No cameras found");
                return false;
            }

            var descriptions = _cameraManager.GetCameraDescriptions();
            DetailsText.Text = $"Found {found} camera(s):\n" + string.Join("\n", descriptions);

            await Task.Run(() => _cameraManager.StartStreaming(progress));

            _isStreaming = true;
            BtnStream.Content = "Stop Stream";
            BtnStream.Background = StreamStopRed;
            BtnAnalyze.IsEnabled = _service != null;

            progress?.Report(_service != null
                ? $"Model: ready  [{_service.ActiveProvider}]  ({_service.CurrentModel}) | Cameras ready"
                : "Cameras ready (no model)");
            return true;
        }

        /// <summary>
        /// Shared logic: load trigger config, connect Modbus, start TriggerService.
        /// Throws on failure.
        /// </summary>
        private void StartTriggerMode()
        {
            if (_triggerService?.IsRunning == true) return;

            var config = TriggerConfig.Load();

            // Diagnostic: enumerate available COM ports
            var availablePorts = ModbusService.GetAvailablePorts();
            Debug.WriteLine($"[Trigger] Available COM ports: {string.Join(", ", availablePorts)}");
            Debug.WriteLine($"[Trigger] Attempting to connect to {config.ComPort} @ {config.BaudRate} baud, slave {config.SlaveId}");

            if (availablePorts.Length == 0)
            {
                throw new InvalidOperationException("No COM ports found on this system. Check USB-to-RS485 adapter connection.");
            }

            if (!availablePorts.Contains(config.ComPort))
            {
                string available = string.Join(", ", availablePorts);
                throw new InvalidOperationException(
                    $"COM port '{config.ComPort}' not found. Available ports: {available}. " +
                    $"Update trigger_config.json or check device connections.");
            }

            var triggerModbus = new ModbusService();
            if (!triggerModbus.Connect(config.ComPort, config.BaudRate, config.SlaveId))
            {
                string err = triggerModbus.LastError ?? "unknown error";
                triggerModbus.Dispose();
                throw new InvalidOperationException($"Modbus connect failed ({config.ComPort}): {err}");
            }

            _triggerService = new TriggerService(
                triggerModbus, _cameraManager, _service, config, _cameraSlots,
                result => Dispatcher.BeginInvoke(() => OnTriggerResult(result)),
                poll   => Dispatcher.BeginInvoke(() => OnPollStatus(poll)));

            _triggerService.Start();
            BtnTriggerStart.IsEnabled = false;
            BtnTriggerStop.IsEnabled = true;
            TriggerStatusText.Text = $"Running \u2014 {config.ComPort} @ {config.BaudRate} " +
                $"(slave {config.SlaveId}) | poll {config.PollIntervalMs}ms | " +
                $"coils {config.TriggerCoil_Cam13}/{config.TriggerCoil_Cam24}";
            TriggerStatusText.Foreground = ReadyGreen;
        }

        private void OnPollStatus(TriggerPollStatus poll)
        {
            if (poll.Success)
            {
                TriggerPollText.Text = $"READ OK | coil 3001={( poll.Coil1Value ? "HIGH" : "low" )}  " +
                    $"coil 3002={( poll.Coil2Value ? "HIGH" : "low" )}  [{DateTime.Now:HH:mm:ss}]";
                TriggerPollText.Foreground = poll.Coil1Value || poll.Coil2Value ? WarningAmber : DimGray;
            }
            else
            {
                string msg = poll.ErrorMessage ?? "unknown";
                string stage = poll.ConsecutiveFailures < 5
                    ? $"(flush at x5)"
                    : poll.ConsecutiveFailures < 15
                        ? $"(reconnect at x15)"
                        : $"(retrying reconnect)";
                TriggerPollText.Text = $"READ FAIL (x{poll.ConsecutiveFailures}) | {msg}  {stage}";
                TriggerPollText.Foreground = ErrorRed;
            }
        }

        private void Model1_Click(object sender, RoutedEventArgs e)
        {
            BtnModel2.IsChecked = false;
            BtnModel1.IsChecked = true;
            SwitchModel("Model 1");
        }

        private void Model2_Click(object sender, RoutedEventArgs e)
        {
            BtnModel1.IsChecked = false;
            BtnModel2.IsChecked = true;
            SwitchModel("Model 2");
        }

        private void Cam2_Click(object sender, RoutedEventArgs e)
        {
            Btn4Cam.IsChecked = false;
            Btn2Cam.IsChecked = true;
            SetCameraMode(2);
        }

        private void Cam4_Click(object sender, RoutedEventArgs e)
        {
            Btn2Cam.IsChecked = false;
            Btn4Cam.IsChecked = true;
            SetCameraMode(4);
        }

        private void SetCameraMode(int count)
        {
            _activeFrameCount = count;
            bool show4 = count == 4;
            CamBorder2.Visibility = show4 ? Visibility.Visible : Visibility.Collapsed;
            CamBorder3.Visibility = show4 ? Visibility.Visible : Visibility.Collapsed;
            BottomCamRow.Height = show4 ? new GridLength(1, GridUnitType.Star) : new GridLength(0);

            if (!show4)
            {
                _verdictLabels[2].Text = "";
                _verdictLabels[3].Text = "";
            }
        }

        private void UpdateCameraLabels()
        {
            if (_camLabels == null) return;
            for (int i = 0; i < FrameCount; i++)
            {
                var cfg = _cameraSlots[i];
                string det = cfg.Detector == "MaskRCNN" ? "MRCNN" : "PCore";
                string trig = $"T{cfg.TriggerGroup}";
                _camLabels[i].Text = $"CAM {i + 1} [{det}] [{trig}]";
            }
        }

        private string GetDetectorForCamera(int camIndex) =>
            _cameraSlots[camIndex].Detector;

        private void UpdateCameraConfigSummary()
        {
            var lines = new List<string>();
            for (int i = 0; i < 4; i++)
            {
                var cfg = _cameraSlots[i];
                string det = cfg.Detector == "MaskRCNN" ? "MRCNN" : "PCore";
                string dev = cfg.DeviceIndex >= 0 ? $"dev {cfg.DeviceIndex}" : "none";
                lines.Add($"CAM {i + 1}: {dev} | {det} | T{cfg.TriggerGroup} | {cfg.CaptureDelayMs}ms");
            }
            int t1 = _cameraSlots.Count(c => c.DeviceIndex >= 0 && c.TriggerGroup == 1);
            int t2 = _cameraSlots.Count(c => c.DeviceIndex >= 0 && c.TriggerGroup == 2);
            lines.Add($"Trigger 1: {t1} cam(s)  •  Trigger 2: {t2} cam(s)");

            if (_service != null && _service.PatchCoreThreshold > 0)
                lines.Add($"PatchCore threshold: {_service.PatchCoreThreshold:F2}  ({_service.CurrentModel})");

            CameraConfigText.Text = string.Join("\n", lines);
        }

        private async void SwitchModel(string model)
        {
            if (_service == null) return;
            BtnAnalyze.IsEnabled = false;
            StatusText.Text = $"Switching to {model}...";
            VerdictText.Text = "LOADING...";
            VerdictText.Foreground = DimGray;
            VerdictBorder.Background = VerdictNeutralBg;

            var progress = new Progress<string>(status => StatusText.Text = status);
            await Task.Run(() => _service.SwitchModel(model, progress));

            StatusText.Text = $"Model: ready  [{_service.ActiveProvider}]  ({_service.CurrentModel})";
            VerdictText.Text = "READY";
            VerdictText.Foreground = ReadyGreen;
            PopulateMetricsTable();
            UpdateCameraConfigSummary();
            BtnAnalyze.IsEnabled = _currentImage != null;
        }

        private void PopulateMetricsTable(List<MetricEvalResult>? metricResults = null)
        {
            if (_service == null) return;
            var thresholds = _service.CurrentThresholds;

            var rows = new List<MetricRowViewModel>();
            foreach (var def in ThresholdConfig.MetricDefs)
            {
                string fmt = def.Decimals switch { 1 => "F1", 2 => "F2", 3 => "F3", _ => "F1" };
                string unit = string.IsNullOrEmpty(def.Unit) ? "" : $" ({def.Unit})";
                string catTag = def.VerdictCategory == "rework" ? "[R]" : "[X]";

                thresholds.TryGetValue(def.Key, out var t);

                MetricEvalResult? evalResult = null;
                if (metricResults != null)
                {
                    foreach (var mr in metricResults)
                    {
                        if (mr.Key == def.Key) { evalResult = mr; break; }
                    }
                }

                var row = new MetricRowViewModel
                {
                    Label = $"{catTag} {def.DisplayName}{unit}",
                    LoText = t != null && t.Lo < 9999 ? t.Lo.ToString(fmt) : "\u2014",
                    HiText = t != null && t.Hi < 9999 ? t.Hi.ToString(fmt) : "\u2014",
                };

                if (evalResult != null)
                {
                    row.ValueText = evalResult.Value.ToString(fmt);
                    row.StatusText = evalResult.Passed ? "PASS" : "FAIL";
                    row.ValueColor = evalResult.Passed ? NormalGray : FailRed;
                    row.StatusColor = evalResult.Passed ? PassGreen : FailRed;
                }
                else
                {
                    row.ValueText = "\u2014";
                    row.StatusText = "\u2014";
                    row.ValueColor = DimGray;
                    row.StatusColor = DimGray;
                }

                rows.Add(row);
            }
            MetricsPanel.ItemsSource = rows;
        }

        private void LoadImage_Click(object sender, RoutedEventArgs e)
        {
            var dlg = new OpenFileDialog
            {
                Filter = "BMP Images|*.bmp|All Images|*.bmp;*.jpg;*.jpeg;*.png;*.tiff|All Files|*.*",
                Title = "Select Washer Image"
            };

            if (dlg.ShowDialog() == true)
            {
                _currentImage?.Dispose();
                _currentImage = new Bitmap(dlg.FileName);

                var source = InspectionService.BitmapToBitmapSource(_currentImage);
                foreach (var img in _imageDisplays)
                    img.Source = source;

                BtnAnalyze.IsEnabled = _service != null;
                VerdictText.Text = "";
                VerdictBorder.Background = VerdictNeutralBg;
                CycleTimeText.Text = "";
                foreach (var lbl in _verdictLabels)
                    lbl.Text = "";
                foreach (var lbl in _scoreLabels)
                    lbl.Text = "";
                PopulateMetricsTable();
                DetailsText.Text = $"Loaded: {Path.GetFileName(dlg.FileName)}  " +
                                   $"({_currentImage.Width}x{_currentImage.Height})  x{_activeFrameCount} frames";
            }
        }

        private async void Stream_Click(object sender, RoutedEventArgs e)
        {
            if (_isStreaming)
            {
                // Stop trigger first if running
                if (_triggerService?.IsRunning == true)
                {
                    _triggerService.Stop();
                    _triggerService.Dispose();
                    _triggerService = null;
                    BtnTriggerStart.IsEnabled = true;
                    BtnTriggerStop.IsEnabled = false;
                    TriggerStatusText.Text = "Stopped (stream stopped)";
                    TriggerStatusText.Foreground = DimGray;
                    TriggerPollText.Text = "";
                }

                // Stop streaming
                _streamTimer?.Stop();
                _streamTimer = null;
                _cameraManager?.StopStreaming();
                _isStreaming = false;
                BtnStream.Content = "Start Stream";
                BtnStream.Background = StreamPurple;
                StatusText.Text = _service != null
                    ? $"Model: ready  [{_service.ActiveProvider}]  ({_service.CurrentModel})"
                    : "Stream stopped";
                return;
            }

            try
            {
                // Enumerate cameras if not done yet
                _cameraManager ??= new CameraManager();
                int found = await Task.Run(() => _cameraManager.Initialize());

                var descriptions = _cameraManager.GetCameraDescriptions();
                if (found == 0)
                    descriptions = [];

                // Show Camera Setup Dialog
                var dialog = new CameraSetupDialog(descriptions, _cameraSlots) { Owner = this };
                if (dialog.ShowDialog() != true || dialog.ResultConfigs == null)
                    return;

                // Apply the configuration
                _cameraSlots = dialog.ResultConfigs;
                CameraManager.CameraIndices = _cameraSlots
                    .Select(c => c.DeviceIndex)
                    .ToArray();

                UpdateCameraLabels();
                UpdateCameraConfigSummary();

                var progress = new Progress<string>(s => StatusText.Text = s);
                await Task.Run(() => _cameraManager.StartStreaming(progress));

                _isStreaming = true;
                BtnStream.Content = "Stop Stream";
                BtnStream.Background = StreamStopRed;
                BtnAnalyze.IsEnabled = _service != null;

                ((IProgress<string>)progress).Report(_service != null
                    ? $"Model: ready  [{_service.ActiveProvider}]  ({_service.CurrentModel}) | Cameras ready"
                    : "Cameras ready (no model)");

                // Start live preview timer
                _streamTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(50) };
                _streamTimer.Tick += StreamTimer_Tick;
                _streamTimer.Start();
                StatusText.Text += " | Live preview ON";

                // Auto-start trigger when manually starting stream
                AutoStartTrigger();
            }
            catch (Exception ex)
            {
                StatusText.Text = $"Camera error: {ex.Message}";
            }
        }

        // Optional: Live preview for debugging/setup — only used if user manually clicks 'Start Stream'
        private void StreamTimer_Tick(object? sender, EventArgs e)
        {
            if (!_isStreaming || _cameraManager == null || _streamTimer == null) return;

            try
            {
                int frameCount = _activeFrameCount;
                int updatedCount = 0;

                for (int i = 0; i < frameCount; i++)
                {
                    var (frame, sequence) = _cameraManager.GetLatestFrameWithSequence(i);
                    if (frame == null) continue;

                    if (sequence == _lastDisplayedSequence[i])
                    {
                        _streamSkipCount++;
                        frame.Dispose();
                        continue;
                    }

                    _imageDisplays[i].Source = InspectionService.BitmapToBitmapSource(frame);
                    _lastDisplayedSequence[i] = sequence;
                    _streamFrameCount++;
                    updatedCount++;
                    frame.Dispose();
                }

                if (_streamFrameCount > 0 && _streamFrameCount % 50 == 0)
                {
                    long total = _streamFrameCount + _streamSkipCount;
                    double utilization = total > 0 ? (_streamFrameCount * 100.0 / total) : 0;
                    Debug.WriteLine($"[StreamPreview] FPS stats: displayed={_streamFrameCount}, skipped={_streamSkipCount}, utilization={utilization:F1}%, last_tick_updated={updatedCount}");
                }
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"[StreamPreview] Error: {ex.Message}");
            }
        }

        private async void Analyze_Click(object sender, RoutedEventArgs e)
        {
            if (_service == null) return;

            // During streaming, use latest camera frames; otherwise use loaded image
            bool useCamera = _isStreaming && _cameraManager != null;
            if (!useCamera && _currentImage == null) return;

            BtnAnalyze.IsEnabled = false;
            // Pause auto-analysis while manual analysis runs
            _streamTimer?.Stop();

            int frameCount = _activeFrameCount;
            var detectors = _cameraSlots.Select(c => c.Detector).Distinct().ToArray();
            string detectorDesc = detectors.Length == 1 ? detectors[0] : string.Join(" + ", detectors);
            StatusText.Text = $"Analyzing {frameCount} frames ({detectorDesc})...";
            VerdictText.Text = "PROCESSING...";
            VerdictText.Foreground = WarningAmber;
            VerdictBorder.Background = VerdictNeutralBg;

            InspectionResult[] results = new InspectionResult[frameCount];
            long batchMs = 0;

            // Snapshot per-camera detector assignments for the background task
            var slotConfigs = _cameraSlots.ToArray();

            await Task.Run(() =>
            {
                Bitmap[] copies;
                if (useCamera)
                {
                    var frames = _cameraManager!.GetAllLatestFrames();
                    copies = new Bitmap[frameCount];
                    for (int i = 0; i < frameCount; i++)
                        copies[i] = (i < frames.Length && frames[i] != null)
                            ? new Bitmap(frames[i]!)
                            : new Bitmap(720, 720);
                }
                else
                {
                    copies = new Bitmap[frameCount];
                    for (int i = 0; i < frameCount; i++)
                        copies[i] = new Bitmap(_currentImage!);
                }

                var batchSw = Stopwatch.StartNew();
                Parallel.For(0, frameCount, i =>
                {
                    string detector = slotConfigs[i].Detector;
                    results[i] = detector == "MaskRCNN"
                        ? _service.InspectMaskRCNN(copies[i])
                        : _service.InspectPatchCore(copies[i]);
                });
                batchMs = batchSw.ElapsedMilliseconds;

                for (int i = 0; i < frameCount; i++)
                {
                    if (!ReferenceEquals(copies[i], results[i].OverlayImage))
                        copies[i].Dispose();
                }
            });

            UpdateAnalysisResults(results, batchMs);

            StatusText.Text = $"Model: ready  [{_service!.ActiveProvider}]  ({_service.CurrentModel})";
            BtnAnalyze.IsEnabled = true;

            // Resume auto-analysis if still streaming
            if (_isStreaming)
                _streamTimer?.Start();
        }

        private void UpdateAnalysisResults(InspectionResult[] results, long batchMs)
        {
            int count = results.Length;
            bool showOverlay = ChkShowOverlay.IsChecked == true;

            for (int i = 0; i < count; i++)
            {
                var r = results[i];
                if (showOverlay && r.OverlayImage != null)
                    _imageDisplays[i].Source = InspectionService.BitmapToBitmapSource(r.OverlayImage);
                _verdictLabels[i].Text = r.Verdict;
                _verdictLabels[i].Foreground = VerdictColorMap.GetValueOrDefault(r.Verdict, NormalGray);

                // Show score vs threshold on PatchCore camera tiles
                if (GetDetectorForCamera(i) == "PatchCore" && r.DetectorType == "PatchCore")
                {
                    _scoreLabels[i].Text = $"Score: {r.AnomalyScore:F2}  |  Threshold: {r.AnomalyThreshold:F2}";
                    _scoreLabels[i].Foreground = r.HasDefect ? FailRed : PassGreen;
                }
                else
                {
                    _scoreLabels[i].Text = "";
                }
            }

            // Show READY in the banner (per-cam verdicts stay on each image tile)
            VerdictText.Text = "READY";
            VerdictText.Foreground = ReadyGreen;
            VerdictBorder.Background = VerdictNeutralBg;

            PopulateMetricsTable(results[0].MetricResults);

            string[] camTypes = new string[count];
            for (int i = 0; i < count; i++)
                camTypes[i] = GetDetectorForCamera(i) == "MaskRCNN" ? "MRCNN" : "PCore";
            var timing = new System.Text.StringBuilder();
            timing.Append($"BATCH ({count} frames): {batchMs} ms total  |  ");
            timing.AppendLine($"Avg: {batchMs / count} ms/frame");
            for (int i = 0; i < count; i++)
            {
                if (i > 0) timing.Append("  |  ");
                timing.Append($"CAM {i + 1} ({camTypes[i]}): {results[i].TotalMs} ms");
            }
            timing.AppendLine();
            timing.Append("PatchCore");
            for (int i = 0; i < count; i++)
            {
                if (GetDetectorForCamera(i) == "PatchCore")
                    timing.Append($"  CAM {i + 1}: {results[i].AnomalyScore:F2}  |");
            }
            for (int i = 0; i < count; i++)
            {
                if (GetDetectorForCamera(i) == "PatchCore")
                {
                    timing.Append($"  Threshold: {results[i].AnomalyThreshold:F2}");
                    break;
                }
            }
            CycleTimeText.Text = timing.ToString();

            // Write Modbus coils: CAM1+2 → coil 0, CAM3+4 → coil 1
            // Fire-and-forget on background thread — never block the UI
            if (_modbus?.IsConnected == true)
            {
                bool cam12Pass = results[0].Verdict == "PASS" && results[1].Verdict == "PASS";
                bool cam34Pass = count > 3
                    ? results[2].Verdict == "PASS" && results[3].Verdict == "PASS"
                    : true;
                ushort coilAddr = ushort.TryParse(TxtCoilAddress.Text.Trim(), out var ca) ? ca : (ushort)0;

                var modbus = _modbus;
                Task.Run(() => modbus.WriteRejectionCoils(!cam12Pass, !cam34Pass, coilAddr))
                    .ContinueWith(t => Dispatcher.BeginInvoke(() =>
                    {
                        if (t.Result)
                        {
                            ModbusStatusText.Text = $"Sent: C0={(!cam12Pass ? 1 : 0)} C1={(!cam34Pass ? 1 : 0)}";
                            ModbusStatusText.Foreground = ReadyGreen;
                        }
                        else
                        {
                            ModbusStatusText.Text = $"Write error: {modbus.LastError}";
                            ModbusStatusText.Foreground = ErrorRed;
                        }
                    }), TaskContinuationOptions.OnlyOnRanToCompletion);
            }
        }

        protected override void OnClosed(EventArgs e)
        {
            _triggerService?.Dispose();
            _streamTimer?.Stop();
            _cameraManager?.Dispose();
            _modbus?.Dispose();
            _service?.Dispose();
            _currentImage?.Dispose();
            base.OnClosed(e);
        }

        // ─── Modbus RS-485 ─────────────────────────────────────────────

        private void CmbComPort_DropDownOpened(object sender, EventArgs e)
        {
            var current = CmbComPort.SelectedItem as string;
            CmbComPort.Items.Clear();
            foreach (var port in ModbusService.GetAvailablePorts())
                CmbComPort.Items.Add(port);
            if (current != null && CmbComPort.Items.Contains(current))
                CmbComPort.SelectedItem = current;
            else if (CmbComPort.Items.Count > 0)
                CmbComPort.SelectedIndex = 0;
        }

        private void ModbusConnect_Click(object sender, RoutedEventArgs e)
        {
            string? comPort = CmbComPort.SelectedItem as string;
            if (string.IsNullOrEmpty(comPort))
            {
                ModbusStatusText.Text = "Select a COM port";
                ModbusStatusText.Foreground = WarningAmber;
                return;
            }

            var baudItem = CmbBaudRate.SelectedItem as ComboBoxItem;
            int baudRate = int.TryParse(baudItem?.Content?.ToString(), out var br) ? br : 9600;
            byte slaveId = byte.TryParse(TxtSlaveId.Text.Trim(), out var sid) ? sid : (byte)1;

            _modbus ??= new ModbusService();
            bool ok = _modbus.Connect(comPort, baudRate, slaveId);

            if (ok)
            {
                ModbusStatusText.Text = $"Connected: {comPort} @ {baudRate}";
                ModbusStatusText.Foreground = ReadyGreen;
                BtnModbusConnect.IsEnabled = false;
                BtnModbusDisconnect.IsEnabled = true;
            }
            else
            {
                ModbusStatusText.Text = $"Failed: {_modbus.LastError}";
                ModbusStatusText.Foreground = ErrorRed;
            }
        }

        private void ModbusDisconnect_Click(object sender, RoutedEventArgs e)
        {
            _modbus?.Disconnect();
            ModbusStatusText.Text = "Disconnected";
            ModbusStatusText.Foreground = DimGray;
            BtnModbusConnect.IsEnabled = true;
            BtnModbusDisconnect.IsEnabled = false;
        }

        // ─── Modbus Trigger Mode ──────────────────────────────────────

        private void TriggerStart_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                StartTriggerMode();
            }
            catch (Exception ex)
            {
                TriggerStatusText.Text = $"Failed: {ex.Message}";
                TriggerStatusText.Foreground = ErrorRed;
            }
        }

        private void TriggerStop_Click(object sender, RoutedEventArgs e)
        {
            _triggerService?.Stop();
            _triggerService?.Dispose();
            _triggerService = null;
            BtnTriggerStart.IsEnabled = true;
            BtnTriggerStop.IsEnabled = false;
            TriggerStatusText.Text = "Stopped";
            TriggerStatusText.Foreground = DimGray;
            TriggerPollText.Text = "";
        }

        private void OnTriggerResult(TriggerResultEvent evt)
        {
            string pair = evt.Type == TriggerType.Trigger1 ? "Trigger 1" : "Trigger 2";

            // Trigger-only mode (no cameras/model): just show that the coil fired
            if (evt.Results.Length == 0)
            {
                TriggerStatusText.Text = $"{pair}: TRIGGERED | {evt.ModbusError ?? "signal only"}";
                TriggerStatusText.Foreground = WarningAmber;
                return;
            }

            // Display frozen inspection results (always show overlay image from inference)
            bool showOverlay = ChkShowOverlay.IsChecked == true;

            for (int i = 0; i < evt.Results.Length && i < evt.Slots.Length; i++)
            {
                int slot = evt.Slots[i];
                if (slot < 0 || slot >= _imageDisplays.Length) continue;
                var r = evt.Results[i];

                // Always display the result image (frozen frame after inference)
                // If overlay checkbox is ON, show annotated image; otherwise show raw image
                if (r.OverlayImage != null)
                {
                    _imageDisplays[slot].Source = InspectionService.BitmapToBitmapSource(
                        showOverlay ? r.OverlayImage : r.OverlayImage);
                }

                _verdictLabels[slot].Text = r.Verdict;
                _verdictLabels[slot].Foreground = VerdictColorMap.GetValueOrDefault(r.Verdict, NormalGray);
            }

            // Update metrics table from first result
            if (evt.Results.Length > 0)
                PopulateMetricsTable(evt.Results[0].MetricResults);

            string modbusInfo = evt.ModbusWriteOk ? "coil written" : (evt.ModbusError ?? "no write");
            string verdicts = string.Join("/", evt.Results.Select(r => r.Verdict));
            TriggerStatusText.Text =
                $"{pair}: {verdicts} | {evt.BatchMs}ms | {modbusInfo}";
            TriggerStatusText.Foreground = ReadyGreen;
        }
    }
}
