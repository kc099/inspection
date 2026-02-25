using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
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

        public MainWindow()
        {
            InitializeComponent();
            _imageDisplays = [ImageDisplay0, ImageDisplay1, ImageDisplay2, ImageDisplay3];
            _verdictLabels = [VerdictLabel0, VerdictLabel1, VerdictLabel2, VerdictLabel3];
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

        private void SwitchModel(string model)
        {
            if (_service == null) return;
            _service.SwitchModel(model);
            StatusText.Text = $"Model: ready  [{_service.ActiveProvider}]  ({_service.CurrentModel})";
            PopulateMetricsTable();
        }

        /// <summary>
        /// Show the metric table with Lo/Hi thresholds pre-populated.
        /// Value and Status show "—" until analysis is run.
        /// After analysis, pass metricResults to show actual values.
        /// </summary>
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

                // Check if we have analysis results for this metric
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
                    LoText = t != null && t.Lo < 9999 ? t.Lo.ToString(fmt) : "—",
                    HiText = t != null && t.Hi < 9999 ? t.Hi.ToString(fmt) : "—",
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
                    row.ValueText = "—";
                    row.StatusText = "—";
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
                FailReasonsText.Text = "";
                DefectInfoText.Text = "---";
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
            StatusText.Text = "Analyzing 4 frames...";

            InspectionResult[] results = new InspectionResult[FrameCount];
            long batchMs = 0;

            await Task.Run(() =>
            {
                // Create copies on the main task thread (Bitmap ctor is not concurrent-safe on same source)
                var copies = new Bitmap[FrameCount];
                for (int i = 0; i < FrameCount; i++)
                    copies[i] = new Bitmap(_currentImage);

                var batchSw = Stopwatch.StartNew();
                Parallel.For(0, FrameCount, i =>
                {
                    results[i] = _service.Inspect(copies[i]);
                });
                batchMs = batchSw.ElapsedMilliseconds;

                // Only dispose copies not retained as overlay images
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

            // --- Use frame 1 result for the detail panels ---
            var result = results[0];

            VerdictText.Text = result.Verdict;
            VerdictBorder.Background = result.Verdict switch
            {
                "PASS" => VerdictPassBg,
                "REWORK" => VerdictReworkBg,
                "REJECT" => VerdictRejectBg,
                "ERROR" => VerdictErrorBg,
                _ => VerdictNeutralBg,
            };

            if (result.ErrorMessage != null)
                FailReasonsText.Text = result.ErrorMessage;
            else if (result.FailReasons.Count > 0)
                FailReasonsText.Text = string.Join(", ", result.FailReasons);
            else
                FailReasonsText.Text = "";

            PopulateMetricsTable(result.MetricResults);

            if (result.Verdict is "REWORK")
            {
                DefectInfoText.Text = "Skipped (geometric rework)";
                DefectInfoText.Foreground = new SolidColorBrush(
                    System.Windows.Media.Color.FromRgb(255, 183, 77));
            }
            else if (result.Verdict is "REJECT" && result.GeoResult != null && !result.HasDefect
                     && result.InferenceMs == 0)
            {
                DefectInfoText.Text = "Skipped (geometric reject)";
                DefectInfoText.Foreground = FailRed;
            }
            else if (result.HasDefect)
            {
                DefectInfoText.Text = $"{result.Detections.Count} defect(s)  " +
                                      $"(top: {result.TopScore:P0})  " +
                                      $"Inference: {result.InferenceMs} ms";
                DefectInfoText.Foreground = FailRed;
            }
            else
            {
                DefectInfoText.Text = $"No defects  |  Inference: {result.InferenceMs} ms";
                DefectInfoText.Foreground = PassGreen;
            }

            // --- Batch timing summary ---
            long totalGeo = 0, totalPrep = 0, totalTensor = 0, totalInf = 0, totalOverlay = 0;
            for (int i = 0; i < FrameCount; i++)
            {
                totalGeo += results[i].GeoMs;
                totalPrep += results[i].PrepMs;
                totalTensor += results[i].TensorMs;
                totalInf += results[i].InferenceMs;
                totalOverlay += results[i].OverlayMs;
            }

            CycleTimeText.Text =
                $"BATCH ({FrameCount} frames): {batchMs} ms total  |  " +
                $"Avg: {batchMs / FrameCount} ms/frame\n" +
                $"Per-frame avg:  Geo: {totalGeo / FrameCount} ms  |  " +
                $"Prep: {totalPrep / FrameCount} ms  |  " +
                $"Tensor: {totalTensor / FrameCount} ms  |  " +
                $"Inference: {totalInf / FrameCount} ms  |  " +
                $"Overlay: {totalOverlay / FrameCount} ms\n" +
                $"CAM 1: {results[0].TotalMs} ms  |  " +
                $"CAM 2: {results[1].TotalMs} ms  |  " +
                $"CAM 3: {results[2].TotalMs} ms  |  " +
                $"CAM 4: {results[3].TotalMs} ms";

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