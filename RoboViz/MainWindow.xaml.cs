using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO;
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
        private Bitmap? _currentImage;

        private System.Windows.Controls.Image[] _imageDisplays = null!;
        private TextBlock[] _verdictLabels = null!;
        private const int FrameCount = 4;

        private static readonly SolidColorBrush PassGreen = new(System.Windows.Media.Color.FromRgb(76, 175, 80));
        private static readonly SolidColorBrush FailRed = new(System.Windows.Media.Color.FromRgb(244, 67, 54));
        private static readonly SolidColorBrush NormalGray = new(System.Windows.Media.Color.FromRgb(200, 200, 200));
        private static readonly SolidColorBrush DimGray = new(System.Windows.Media.Color.FromRgb(110, 110, 118));

        private static readonly SolidColorBrush VerdictPassBg = new(System.Windows.Media.Color.FromRgb(27, 94, 32));
        private static readonly SolidColorBrush VerdictReworkBg = new(System.Windows.Media.Color.FromRgb(230, 126, 34));
        private static readonly SolidColorBrush VerdictRejectBg = new(System.Windows.Media.Color.FromRgb(211, 47, 47));
        private static readonly SolidColorBrush VerdictNeutralBg = new(System.Windows.Media.Color.FromRgb(55, 55, 64));
        private static readonly SolidColorBrush VerdictErrorBg = new(System.Windows.Media.Color.FromRgb(120, 60, 60));

        static MainWindow()
        {
            PassGreen.Freeze(); FailRed.Freeze(); NormalGray.Freeze(); DimGray.Freeze();
            VerdictPassBg.Freeze(); VerdictReworkBg.Freeze(); VerdictRejectBg.Freeze();
            VerdictNeutralBg.Freeze(); VerdictErrorBg.Freeze();
        }

        private readonly DispatcherTimer _clockTimer;

        public MainWindow()
        {
            InitializeComponent();
            _imageDisplays = [ImageDisplay0, ImageDisplay1, ImageDisplay2, ImageDisplay3];
            _verdictLabels = [VerdictLabel0, VerdictLabel1, VerdictLabel2, VerdictLabel3];

            _clockTimer = new DispatcherTimer { Interval = TimeSpan.FromSeconds(1) };
            _clockTimer.Tick += (_, _) => DateTimeText.Text = DateTime.Now.ToString("yyyy-MM-dd  HH:mm:ss");
            _clockTimer.Start();
            DateTimeText.Text = DateTime.Now.ToString("yyyy-MM-dd  HH:mm:ss");
        }

        private async void Window_Loaded(object sender, RoutedEventArgs e)
        {
            StatusText.Text = "Model: loading...";
            var progress = new Progress<string>(status => StatusText.Text = status);

            try
            {
                await Task.Run(() =>
                {
                    string modelPath = Path.Combine(
                        AppDomain.CurrentDomain.BaseDirectory, "maskrcnn_oring.onnx");

                    if (!File.Exists(modelPath))
                        throw new FileNotFoundException("ONNX model not found at: " + modelPath);

                    _service = new InspectionService(modelPath, scoreThreshold: 0.5f,
                        useGpu: true, progress: progress, modelName: "Model 2");
                });

                StatusText.Text = $"Model: ready  [{_service!.ActiveProvider}]  ({_service.CurrentModel})";
                StatusText.Foreground = new SolidColorBrush(
                    System.Windows.Media.Color.FromRgb(100, 220, 100));

                if (_service.GpuError != null)
                {
                    StatusText.Foreground = new SolidColorBrush(
                        System.Windows.Media.Color.FromRgb(255, 183, 77));
                    DetailsText.Text = $"GPU fallback: {_service.GpuError}";
                }

                PopulateMetricsTable();
            }
            catch (Exception ex)
            {
                StatusText.Text = "Model: failed to load";
                StatusText.Foreground = new SolidColorBrush(
                    System.Windows.Media.Color.FromRgb(255, 80, 80));
                DetailsText.Text = ex.Message;
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

        private async void SwitchModel(string model)
        {
            if (_service == null) return;
            BtnAnalyze.IsEnabled = false;
            StatusText.Text = $"Switching to {model}...";

            var progress = new Progress<string>(status => StatusText.Text = status);
            await Task.Run(() => _service.SwitchModel(model, progress));

            StatusText.Text = $"Model: ready  [{_service.ActiveProvider}]  ({_service.CurrentModel})";
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
                                   $"({_currentImage.Width}x{_currentImage.Height})  x{FrameCount} frames";
            }
        }

        private async void Analyze_Click(object sender, RoutedEventArgs e)
        {
            if (_currentImage == null || _service == null) return;

            BtnAnalyze.IsEnabled = false;
            StatusText.Text = "Analyzing 4 frames (MaskRCNN + PatchCore)...";

            InspectionResult[] results = new InspectionResult[FrameCount];
            long batchMs = 0;

            await Task.Run(() =>
            {
                var copies = new Bitmap[FrameCount];
                for (int i = 0; i < FrameCount; i++)
                    copies[i] = new Bitmap(_currentImage);

                var batchSw = Stopwatch.StartNew();
                Parallel.For(0, FrameCount, i =>
                {
                    // CAM 1,3 (index 0,2) -> MaskRCNN; CAM 2,4 (index 1,3) -> PatchCore
                    results[i] = (i % 2 == 0)
                        ? _service.InspectMaskRCNN(copies[i])
                        : _service.InspectPatchCore(copies[i]);
                });
                batchMs = batchSw.ElapsedMilliseconds;

                for (int i = 0; i < FrameCount; i++)
                {
                    if (!ReferenceEquals(copies[i], results[i].OverlayImage))
                        copies[i].Dispose();
                }
            });

            // --- Update each frame's overlay + verdict label ---
            var verdictColors = new Dictionary<string, SolidColorBrush>
            {
                ["PASS"] = PassGreen,
                ["REWORK"] = VerdictReworkBg,
                ["REJECT"] = FailRed,
                ["ERROR"] = VerdictErrorBg,
            };

            for (int i = 0; i < FrameCount; i++)
            {
                var r = results[i];
                if (r.OverlayImage != null)
                    _imageDisplays[i].Source = InspectionService.BitmapToBitmapSource(r.OverlayImage);

                _verdictLabels[i].Text = r.Verdict;
                _verdictLabels[i].Foreground = verdictColors.GetValueOrDefault(r.Verdict, NormalGray);
            }

            // --- Determine worst verdict across all frames ---
            string batchVerdict = "PASS";
            foreach (var r in results)
            {
                if (r.Verdict == "ERROR") { batchVerdict = "ERROR"; break; }
                if (r.Verdict == "REJECT") batchVerdict = "REJECT";
                else if (r.Verdict == "REWORK" && batchVerdict == "PASS") batchVerdict = "REWORK";
            }

            VerdictText.Text = batchVerdict;
            VerdictBorder.Background = batchVerdict switch
            {
                "PASS" => VerdictPassBg,
                "REWORK" => VerdictReworkBg,
                "REJECT" => VerdictRejectBg,
                "ERROR" => VerdictErrorBg,
                _ => VerdictNeutralBg,
            };

            PopulateMetricsTable(results[0].MetricResults);

            // --- Batch timing summary ---
            string[] camTypes = ["MRCNN", "PCore", "MRCNN", "PCore"];
            CycleTimeText.Text =
                $"BATCH ({FrameCount} frames): {batchMs} ms total  |  " +
                $"Avg: {batchMs / FrameCount} ms/frame\n" +
                $"CAM 1 ({camTypes[0]}): {results[0].TotalMs} ms  |  " +
                $"CAM 2 ({camTypes[1]}): {results[1].TotalMs} ms  |  " +
                $"CAM 3 ({camTypes[2]}): {results[2].TotalMs} ms  |  " +
                $"CAM 4 ({camTypes[3]}): {results[3].TotalMs} ms\n" +
                $"PatchCore  CAM 2: {results[1].AnomalyScore:F2}  |  " +
                $"CAM 4: {results[3].AnomalyScore:F2}  |  " +
                $"Threshold: {results[1].AnomalyThreshold:F2}";

            StatusText.Text = $"Model: ready  [{_service!.ActiveProvider}]  ({_service.CurrentModel})";
            BtnAnalyze.IsEnabled = true;
        }

        protected override void OnClosed(EventArgs e)
        {
            _service?.Dispose();
            _currentImage?.Dispose();
            base.OnClosed(e);
        }
    }
}
