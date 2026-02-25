using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Windows.Media;
using System.Windows.Media.Imaging;

namespace RoboViz;

/// <summary>
/// Full inspection pipeline:
///   1. Geometric measurement on raw image (2448×2048)
///   2. Evaluate rework/reject thresholds
///   3. If geometric PASS ? bin+crop to 720×720 ? Mask R-CNN defect detection
///   4. Final verdict: REWORK ? REJECT ? PASS
/// </summary>
public class InspectionService : IDisposable
{
    private readonly MaskRCNNDetector _detector;
    private readonly float _scoreThreshold;
    private Dictionary<string, MetricThreshold> _thresholds;
    private string _currentModel;

    public string ActiveProvider => _detector.ActiveProvider;
    public string? GpuError => _detector.GpuError;
    public string CurrentModel => _currentModel;
    public Dictionary<string, MetricThreshold> CurrentThresholds => _thresholds;

    public InspectionService(string onnxModelPath, float scoreThreshold = 0.5f,
        bool useGpu = true, IProgress<string>? progress = null,
        string modelName = "Model 2")
    {
        _detector = new MaskRCNNDetector(onnxModelPath, useGpu, progress);
        _scoreThreshold = scoreThreshold;
        _currentModel = modelName;
        _thresholds = LoadThresholdsForModel(modelName);
    }

    public void SwitchModel(string modelName)
    {
        _currentModel = modelName;
        _thresholds = LoadThresholdsForModel(modelName);
    }

    private static Dictionary<string, MetricThreshold> LoadThresholdsForModel(string modelName)
    {
        string fileName = modelName == "Model 1"
            ? "model1_tuned_thresholds.json"
            : "model2_tuned_thresholds.json";
        string path = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, fileName);
        return ThresholdConfig.LoadThresholds(path);
    }

    public InspectionResult Inspect(Bitmap rawImage)
    {
        var totalSw = Stopwatch.StartNew();

        // --- Step 1: Geometric measurement on raw image ---
        var geoSw = Stopwatch.StartNew();
        var geoResult = OringMeasurement.Measure(rawImage);
        long geoMs = geoSw.ElapsedMilliseconds;

        if (geoResult == null)
        {
            totalSw.Stop();
            return new InspectionResult
            {
                Verdict = "ERROR",
                TotalMs = totalSw.ElapsedMilliseconds,
                GeoMs = geoMs,
                ErrorMessage = "Could not detect o-ring contours",
                OverlayImage = rawImage,
            };
        }

        // --- Step 2: Normalize & evaluate thresholds ---
        double scale = ThresholdConfig.ComputeResolutionScale(rawImage.Width, rawImage.Height);
        var rawMetrics = geoResult.ToDictionary();
        var normedMetrics = ThresholdConfig.NormalizeMeasurements(rawMetrics, scale);
        var (metricResults, geoVerdict, failReasons) = ThresholdConfig.Evaluate(normedMetrics, _thresholds);

        var result = new InspectionResult
        {
            GeoResult = geoResult,
            MetricResults = metricResults,
            GeoMs = geoMs,
        };

        // --- Step 3: Rework/Reject from geometry ? skip Mask R-CNN ---
        if (geoVerdict is "REWORK" or "REJECT")
        {
            result.Verdict = geoVerdict;
            result.FailReasons = failReasons;
            result.OverlayImage = OringMeasurement.DrawGeometricOverlay(rawImage, geoResult);
            totalSw.Stop();
            result.TotalMs = totalSw.ElapsedMilliseconds;
            return result;
        }

        // --- Step 4: Geometric PASS ? bin+crop to 720×720 ? Mask R-CNN ---
        var prepSw = Stopwatch.StartNew();
        using var img720 = OringMeasurement.BinCrop720(rawImage);
        long prepMs = prepSw.ElapsedMilliseconds;

        var detections = _detector.Detect(img720, _scoreThreshold,
            out long tensorMs, out long inferenceMs);

        result.TensorMs = tensorMs;
        result.InferenceMs = inferenceMs;
        result.PrepMs = prepMs;
        result.Detections = detections;
        result.HasDefect = detections.Count > 0;
        result.TopScore = result.HasDefect ? detections.Max(d => d.Score) : 0f;

        if (result.HasDefect)
        {
            result.Verdict = "REJECT";
            result.FailReasons = [$"Defect detected ({detections.Count}, top: {result.TopScore:P0})"];

            var overlaySw = Stopwatch.StartNew();
            result.OverlayImage = DrawDefectOverlay(img720, detections);
            result.OverlayMs = overlaySw.ElapsedMilliseconds;
        }
        else
        {
            result.Verdict = "PASS";
            result.OverlayImage = rawImage;
        }

        totalSw.Stop();
        result.TotalMs = totalSw.ElapsedMilliseconds;
        return result;
    }

    private static Bitmap DrawDefectOverlay(Bitmap image, List<Detection> detections)
    {
        var overlay = new Bitmap(image);
        int w = overlay.Width;
        int h = overlay.Height;

        var rect = new Rectangle(0, 0, w, h);
        var bmpData = overlay.LockBits(rect, ImageLockMode.ReadWrite,
            System.Drawing.Imaging.PixelFormat.Format24bppRgb);
        try
        {
            int stride = bmpData.Stride;
            byte[] pixels = new byte[stride * h];
            Marshal.Copy(bmpData.Scan0, pixels, 0, pixels.Length);

            foreach (var det in detections)
            {
                int maskH = Math.Min(det.Mask.GetLength(0), h);
                int maskW = Math.Min(det.Mask.GetLength(1), w);

                for (int y = 0; y < maskH; y++)
                {
                    int rowOffset = y * stride;
                    for (int x = 0; x < maskW; x++)
                    {
                        if (det.Mask[y, x] > 0.5f)
                        {
                            int idx = rowOffset + x * 3;
                            pixels[idx] = (byte)(pixels[idx] * 0.6);
                            pixels[idx + 1] = (byte)(pixels[idx + 1] * 0.6);
                            pixels[idx + 2] = (byte)Math.Min(pixels[idx + 2] * 0.6 + 255 * 0.4, 255);
                        }
                    }
                }
            }

            Marshal.Copy(pixels, 0, bmpData.Scan0, pixels.Length);
        }
        finally
        {
            overlay.UnlockBits(bmpData);
        }

        using (var g = Graphics.FromImage(overlay))
        {
            using var font = new Font("Segoe UI", 14, System.Drawing.FontStyle.Bold);
            foreach (var det in detections)
            {
                string label = $"{det.Score:P0}";
                float cx = (det.X1 + det.X2) / 2;
                float cy = (det.Y1 + det.Y2) / 2;
                g.DrawString(label, font, System.Drawing.Brushes.White, cx - 20, cy - 10);
            }
        }

        return overlay;
    }

    public static BitmapSource BitmapToBitmapSource(Bitmap bmp)
    {
        var bmpData = bmp.LockBits(
            new Rectangle(0, 0, bmp.Width, bmp.Height),
            ImageLockMode.ReadOnly,
            System.Drawing.Imaging.PixelFormat.Format24bppRgb);

        var source = BitmapSource.Create(
            bmp.Width, bmp.Height,
            96, 96,
            PixelFormats.Bgr24,
            null,
            bmpData.Scan0,
            bmpData.Stride * bmp.Height,
            bmpData.Stride);

        bmp.UnlockBits(bmpData);
        source.Freeze();
        return source;
    }

    public void Dispose()
    {
        _detector?.Dispose();
    }
}

public class InspectionResult
{
    public string Verdict { get; set; } = string.Empty;
    public string? ErrorMessage { get; set; }
    public List<string> FailReasons { get; set; } = [];

    // Geometric
    public GeometricResult? GeoResult { get; set; }
    public List<MetricEvalResult> MetricResults { get; set; } = [];

    // Defect detection
    public bool HasDefect { get; set; }
    public List<Detection> Detections { get; set; } = [];
    public float TopScore { get; set; }

    // Timing
    public long TotalMs { get; set; }
    public long GeoMs { get; set; }
    public long PrepMs { get; set; }
    public long TensorMs { get; set; }
    public long InferenceMs { get; set; }
    public long OverlayMs { get; set; }

    public Bitmap? OverlayImage { get; set; }
}
