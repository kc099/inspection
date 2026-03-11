using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Threading;
using Microsoft.Win32;

namespace RoboViz
{
    public class MetricRowViewModel
    {
        public string Label { get; set; } = "";
        public string ValueText { get; set; } = "";
        public string LoText { get; set; } = "";
        public string HiText { get; set; } = "";
        public string StatusText { get; set; } = "";
        public SolidColorBrush ValueColor { get; set; } = new(Colors.Gray);
        public SolidColorBrush StatusColor { get; set; } = new(Colors.Gray);
    }

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
        private TextBlock[] _camLabels = null!;
        private const int FrameCount = 4;
        private int _activeFrameCount = 4;
        private string _row1Detector = "MaskRCNN";
        private string _row2Detector = "PatchCore";

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
                        AppDomain.CurrentDomain.BaseDirectory, "maskrcnn_oring.onnx");

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

        private void Row1MaskRCNN_Click(object sender, RoutedEventArgs e)
        {
            if (BtnRow1PatchCore != null) BtnRow1PatchCore.IsChecked = false;
            BtnRow1MaskRCNN.IsChecked = true;
            _row1Detector = "MaskRCNN";
            UpdateCameraLabels();
        }

        private void Row1PatchCore_Click(object sender, RoutedEventArgs e)
        {
            if (BtnRow1MaskRCNN != null) BtnRow1MaskRCNN.IsChecked = false;
            BtnRow1PatchCore.IsChecked = true;
            _row1Detector = "PatchCore";
            UpdateCameraLabels();
        }

        private void Row2MaskRCNN_Click(object sender, RoutedEventArgs e)
        {
            if (BtnRow2PatchCore != null) BtnRow2PatchCore.IsChecked = false;
            BtnRow2MaskRCNN.IsChecked = true;
            _row2Detector = "MaskRCNN";
            UpdateCameraLabels();
        }

        private void Row2PatchCore_Click(object sender, RoutedEventArgs e)
        {
            if (BtnRow2MaskRCNN != null) BtnRow2MaskRCNN.IsChecked = false;
            BtnRow2PatchCore.IsChecked = true;
            _row2Detector = "PatchCore";
            UpdateCameraLabels();
        }

        private void UpdateCameraLabels()
        {
            if (_camLabels == null) return;
            string[] detectors = [_row1Detector, _row1Detector, _row2Detector, _row2Detector];
            for (int i = 0; i < FrameCount; i++)
                _camLabels[i].Text = $"CAM {i + 1} [{detectors[i]}]";
        }

        private string GetDetectorForCamera(int camIndex) =>
            camIndex < 2 ? _row1Detector : _row2Detector;

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
                PopulateMetricsTable();
                DetailsText.Text = $"Loaded: {Path.GetFileName(dlg.FileName)}  " +
                                   $"({_currentImage.Width}x{_currentImage.Height})  x{_activeFrameCount} frames";
            }
        }

        private async void Stream_Click(object sender, RoutedEventArgs e)
        {
            if (_isStreaming)
            {
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

            // Initialize cameras
            try
            {
                StatusText.Text = "Initializing cameras...";
                _cameraManager ??= new CameraManager();

                int found = await Task.Run(() => _cameraManager.Initialize());
                if (found == 0)
                {
                    StatusText.Text = "No cameras found";
                    return;
                }

                var descriptions = _cameraManager.GetCameraDescriptions();
                DetailsText.Text = $"Found {found} camera(s):\n" + string.Join("\n", descriptions);

                var progress = new Progress<string>(s => StatusText.Text = s);
                await Task.Run(() => _cameraManager.StartStreaming(progress));

                _isStreaming = true;
                BtnStream.Content = "Stop Stream";
                BtnStream.Background = StreamStopRed;
                BtnAnalyze.IsEnabled = _service != null;

                // Poll latest frames and auto-analyze
                _streamTimer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(500) };
                _streamTimer.Tick += StreamTimer_Tick;
                _streamTimer.Start();
            }
            catch (Exception ex)
            {
                StatusText.Text = $"Camera error: {ex.Message}";
            }
        }

        private async void StreamTimer_Tick(object? sender, EventArgs e)
        {
            if (!_isStreaming || _cameraManager == null || _service == null) return;

            // Prevent re-entrant ticks while analyzing
            _streamTimer?.Stop();

            try
            {
                int frameCount = _activeFrameCount;
                var frames = _cameraManager.GetAllLatestFrames();

                // Display raw frames
                for (int i = 0; i < frameCount && i < frames.Length; i++)
                {
                    if (frames[i] != null)
                        _imageDisplays[i].Source = InspectionService.BitmapToBitmapSource(frames[i]!);
                }

                // Run analysis if all frames are available
                bool allReady = true;
                for (int i = 0; i < frameCount && i < frames.Length; i++)
                {
                    if (frames[i] == null) { allReady = false; break; }
                }

                if (allReady)
                {
                    BtnAnalyze.IsEnabled = false;
                    StatusText.Text = "Analyzing stream frame...";
                    VerdictText.Text = "PROCESSING...";
                    VerdictText.Foreground = WarningAmber;

                    InspectionResult[] results = new InspectionResult[frameCount];
                    long batchMs = 0;

                    // Capture references for the background thread
                    var capturedFrames = frames;
                    string row1Det = _row1Detector;
                    string row2Det = _row2Detector;

                    await Task.Run(() =>
                    {
                        var batchSw = Stopwatch.StartNew();
                        Parallel.For(0, frameCount, i =>
                        {
                            string detector = i < 2 ? row1Det : row2Det;
                            results[i] = detector == "MaskRCNN"
                                ? _service.InspectMaskRCNN(capturedFrames[i]!)
                                : _service.InspectPatchCore(capturedFrames[i]!);
                        });
                        batchMs = batchSw.ElapsedMilliseconds;
                    });

                    // Update UI with results (same as Analyze_Click)
                    UpdateAnalysisResults(results, batchMs);

                    // Dispose captured frames
                    for (int i = 0; i < capturedFrames.Length; i++)
                    {
                        if (capturedFrames[i] != null
                            && (i >= frameCount || !ReferenceEquals(capturedFrames[i], results[i].OverlayImage)))
                            capturedFrames[i]!.Dispose();
                    }
                }
            }
            catch (Exception ex)
            {
                DetailsText.Text = $"Stream error: {ex.Message}";
            }
            finally
            {
                BtnAnalyze.IsEnabled = _service != null;
                if (_isStreaming)
                    _streamTimer?.Start();
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
            string detectorDesc = _row1Detector == _row2Detector
                ? _row1Detector
                : $"{_row1Detector} + {_row2Detector}";
            StatusText.Text = $"Analyzing {frameCount} frames ({detectorDesc})...";
            VerdictText.Text = "PROCESSING...";
            VerdictText.Foreground = WarningAmber;
            VerdictBorder.Background = VerdictNeutralBg;

            InspectionResult[] results = new InspectionResult[frameCount];
            long batchMs = 0;

            string row1Det = _row1Detector;
            string row2Det = _row2Detector;

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
                    string detector = i < 2 ? row1Det : row2Det;
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
            // Load config from file
            TriggerConfig config;
            try
            {
                config = TriggerConfig.Load();
            }
            catch (Exception ex)
            {
                TriggerStatusText.Text = $"Config error: {ex.Message}";
                TriggerStatusText.Foreground = ErrorRed;
                return;
            }

            // Connect Modbus using config file settings (separate from the manual output Modbus)
            var triggerModbus = new ModbusService();
            if (!triggerModbus.Connect(config.ComPort, config.BaudRate, config.SlaveId))
            {
                TriggerStatusText.Text = $"Modbus connect failed: {triggerModbus.LastError}";
                TriggerStatusText.Foreground = ErrorRed;
                triggerModbus.Dispose();
                return;
            }

            // Capture current detector assignments
            string row1Det = _row1Detector;
            string row2Det = _row2Detector;
            Func<int, string> getDetector = i => i < 2 ? row1Det : row2Det;

            _triggerService = new TriggerService(
                triggerModbus, _cameraManager, _service, config, getDetector,
                result => Dispatcher.BeginInvoke(() => OnTriggerResult(result)));

            _triggerService.Start();
            BtnTriggerStart.IsEnabled = false;
            BtnTriggerStop.IsEnabled = true;
            TriggerStatusText.Text = $"Running — {config.ComPort} @ {config.BaudRate} " +
                $"(slave {config.SlaveId}) | poll {config.PollIntervalMs}ms";
            TriggerStatusText.Foreground = ReadyGreen;
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
        }

        private void OnTriggerResult(TriggerResultEvent evt)
        {
            string pair = evt.Type == TriggerType.Cam13 ? "CAM 1+3" : "CAM 2+4";

            // Trigger-only mode (no cameras/model): just show that the coil fired
            if (evt.Results.Length == 0)
            {
                TriggerStatusText.Text = $"{pair}: TRIGGERED | {evt.ModbusError ?? "signal only"}";
                TriggerStatusText.Foreground = WarningAmber;
                return;
            }

            // Full mode: update images + verdicts
            int[] slots = evt.Type == TriggerType.Cam13 ? [0, 2] : [1, 3];
            bool showOverlay = ChkShowOverlay.IsChecked == true;

            for (int i = 0; i < evt.Results.Length && i < slots.Length; i++)
            {
                int slot = slots[i];
                var r = evt.Results[i];
                if (showOverlay && r.OverlayImage != null)
                    _imageDisplays[slot].Source = InspectionService.BitmapToBitmapSource(r.OverlayImage);
                _verdictLabels[slot].Text = r.Verdict;
                _verdictLabels[slot].Foreground = VerdictColorMap.GetValueOrDefault(r.Verdict, NormalGray);
            }

            string modbusInfo = evt.ModbusWriteOk ? "coil written" : (evt.ModbusError ?? "no write");
            TriggerStatusText.Text =
                $"{pair}: {evt.Results[0].Verdict}/{evt.Results[1].Verdict} | {evt.BatchMs}ms | {modbusInfo}";
            TriggerStatusText.Foreground = ReadyGreen;
        }
    }
}
