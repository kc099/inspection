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
/// Full inspection pipeline supporting two detector types:
///   - Mask R-CNN for CAM 1, 3 (defect segmentation)
///   - PatchCore for CAM 2, 4 (anomaly detection)
///
/// Pipeline per frame:
///   Mask R-CNN (CAM 1, 3):
///     1. Geometric measurement on raw image
///     2. Evaluate rework/reject thresholds
///     3. If geometric PASS: bin+crop to 720x720 then detect
///   PatchCore (CAM 2, 4 — side view, no hole visible):
///     1. 4x4 bin to 512x384, YOLO seg crop, resize 384x384
///     2. PatchCore anomaly detection (no geometric measurement)
///   Final verdict: REWORK / REJECT / PASS
/// </summary>
public class InspectionService : IDisposable
{
    private readonly MaskRCNNDetector _maskrcnn;
    private readonly float _scoreThreshold;
    private Dictionary<string, MetricThreshold> _thresholdsCam3001;
    private Dictionary<string, MetricThreshold> _thresholdsCam3002;
    private string _currentModel;

    public string ActiveProvider => _maskrcnn.ActiveProvider;
    public string? GpuError => _maskrcnn.GpuError;
    public string CurrentModel => _currentModel;
    public Dictionary<string, MetricThreshold> CurrentThresholds => _thresholdsCam3001;
    public Dictionary<string, MetricThreshold> ThresholdsCam3001 => _thresholdsCam3001;
    public Dictionary<string, MetricThreshold> ThresholdsCam3002 => _thresholdsCam3002;

    public InspectionService(string maskrcnnModelPath, float scoreThreshold = 0.5f,
        bool useGpu = true, IProgress<string>? progress = null,
        string modelName = "Model 2")
    {
        _maskrcnn = new MaskRCNNDetector(maskrcnnModelPath, useGpu, progress);
        _scoreThreshold = scoreThreshold;
        _currentModel = modelName;
        (_thresholdsCam3001, _thresholdsCam3002) = LoadThresholdsForModel(modelName);
    }

    public void SwitchModel(string modelName, IProgress<string>? progress = null)
    {
        if (_currentModel == modelName) return;
        _currentModel = modelName;
        (_thresholdsCam3001, _thresholdsCam3002) = LoadThresholdsForModel(modelName);
    }

    public float PatchCoreThreshold => 0f;

    private static (Dictionary<string, MetricThreshold> cam3001, Dictionary<string, MetricThreshold> cam3002)
        LoadThresholdsForModel(string modelName)
    {
        string assetsDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Assets");
        string suffix = modelName == "Model 1" ? "model1" : "model2";

        string csv3001 = Path.Combine(assetsDir, $"{suffix}_3001_measurements_stats.csv");
        string csv3002 = Path.Combine(assetsDir, $"{suffix}_3002_measurements_stats.csv");

        var t3001 = ThresholdConfig.LoadThresholdsFromCsv(csv3001);
        var t3002 = ThresholdConfig.LoadThresholdsFromCsv(csv3002);
        return (t3001, t3002);
    }

    /// <summary>
    /// Shared geometric measurement + threshold evaluation.
    /// Returns the partially filled result and the geo verdict.
    /// If the result is terminal (ERROR / REWORK / REJECT), geoVerdict != "PASS".
    /// </summary>
    private InspectionResult RunGeoEvaluation(
        Bitmap rawImage, string detectorType, Stopwatch totalSw,
        Dictionary<string, MetricThreshold> thresholds,
        out string geoVerdict, int slot = 0)
    {
        var geoSw = Stopwatch.StartNew();
        var geoResult = slot == 1
            ? OringMeasurement.MeasureCam2(rawImage)
            : OringMeasurement.Measure(rawImage);
        long geoMs = geoSw.ElapsedMilliseconds;

        if (geoResult == null)
        {
            geoVerdict = "ERROR";
            return new InspectionResult
            {
                DetectorType = detectorType,
                Verdict = "ERROR",
                TotalMs = totalSw.ElapsedMilliseconds,
                GeoMs = geoMs,
                ErrorMessage = "Could not detect o-ring contours",
                OverlayImage = rawImage,
            };
        }

        double scale = ThresholdConfig.ComputeResolutionScale(rawImage.Width, rawImage.Height);
        var rawMetrics = geoResult.ToDictionary();
        var normedMetrics = ThresholdConfig.NormalizeMeasurements(rawMetrics, scale);
        var (metricResults, verdict, failReasons) = ThresholdConfig.Evaluate(normedMetrics, thresholds);
        geoVerdict = verdict;

        var result = new InspectionResult
        {
            DetectorType = detectorType,
            GeoResult = geoResult,
            MetricResults = metricResults,
            GeoMs = geoMs,
        };

        if (geoVerdict is "REWORK" or "REJECT")
        {
            result.Verdict = geoVerdict;
            result.FailReasons = failReasons;
            result.OverlayImage = slot == 1
                ? OringMeasurement.DrawContourOverlayCam2(rawImage, geoResult)
                : OringMeasurement.DrawGeometricOverlay(rawImage, geoResult);
            result.TotalMs = totalSw.ElapsedMilliseconds;
        }

        return result;
    }

    /// <summary>
    /// Run Mask R-CNN inspection on any camera.
    /// triggerGroup selects the per-camera threshold set (1 = coil 3001, 2 = coil 3002).
    /// skipGeo = true for side-view cameras that lack a visible hole for geometric measurement.
    /// slot identifies the display slot (0 = cam1, 1 = cam2) to select the geo method.
    /// </summary>
    public InspectionResult InspectMaskRCNN(Bitmap rawImage, int triggerGroup = 1, bool skipGeo = false, int slot = 0)
    {
        var totalSw = Stopwatch.StartNew();
        InspectionResult result;

        if (!skipGeo)
        {
            var thresholds = triggerGroup == 2 ? _thresholdsCam3002 : _thresholdsCam3001;
            result = RunGeoEvaluation(rawImage, "MaskRCNN", totalSw, thresholds, out string geoVerdict, slot);
            if (geoVerdict != "PASS")
                return result;
        }
        else
        {
            result = new InspectionResult { DetectorType = "MaskRCNN" };
        }

        var prepSw = Stopwatch.StartNew();
        using var binned = ResizeTo512x384(rawImage);
        long prepMs = prepSw.ElapsedMilliseconds;

        var detections = _maskrcnn.Detect(binned, _scoreThreshold,
            out long tensorMs, out long inferenceMs);

        result.TensorMs = tensorMs;
        result.InferenceMs = inferenceMs;
        result.PrepMs = prepMs;
        result.Detections = detections;
        result.HasDefect = detections.Count > 0;
        result.TopScore = result.HasDefect ? detections.Max(d => d.Score) : 0f;

        var counts = new int[detections.Count];
        for (int di = 0; di < detections.Count; di++)
        {
            var mask = detections[di].Mask;
            int h = mask.GetLength(0), w = mask.GetLength(1), c = 0;
            for (int py = 0; py < h; py++)
                for (int px = 0; px < w; px++)
                    if (mask[py, px] > 0.5f) c++;
            counts[di] = c;
        }
        result.MaskPixelCounts = [..counts];

        if (result.HasDefect)
        {
            result.Verdict = "REJECT";
            result.FailReasons = [$"Defect detected ({detections.Count}, top: {result.TopScore:P0})"];

            var overlaySw = Stopwatch.StartNew();
            result.OverlayImage = DrawDefectOverlay(binned, detections);
            result.OverlayMs = overlaySw.ElapsedMilliseconds;
        }
        else
        {
            result.Verdict = "PASS";
            result.OverlayImage = rawImage;
        }

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

    /// <summary>
    /// Resize any input image to exactly 512x384 for the combined MaskRCNN model.
    /// Uses 4x4 binning when input is 2048x1536; high-quality resize otherwise.
    /// </summary>
    private static Bitmap ResizeTo512x384(Bitmap original)
    {
        const int targetW = 512;
        const int targetH = 384;

        if (original.Width == 2048 && original.Height == 1536)
            return BinTo512x384(original);

        var result = new Bitmap(targetW, targetH, System.Drawing.Imaging.PixelFormat.Format24bppRgb);
        using (var g = Graphics.FromImage(result))
        {
            g.InterpolationMode = System.Drawing.Drawing2D.InterpolationMode.HighQualityBicubic;
            g.PixelOffsetMode = System.Drawing.Drawing2D.PixelOffsetMode.HighQuality;
            g.DrawImage(original, 0, 0, targetW, targetH);
        }
        return result;
    }

    /// <summary>
    /// 4x4 binning: reduce 2048x1536 to 512x384 by averaging each 4x4 pixel block.
    /// </summary>
    private static Bitmap BinTo512x384(Bitmap original)
    {
        const int bin = 4;
        int outW = original.Width / bin;
        int outH = original.Height / bin;

        var srcRect = new Rectangle(0, 0, original.Width, original.Height);
        var srcData = original.LockBits(srcRect, ImageLockMode.ReadOnly,
            System.Drawing.Imaging.PixelFormat.Format24bppRgb);

        var result = new Bitmap(outW, outH, System.Drawing.Imaging.PixelFormat.Format24bppRgb);
        var dstRect = new Rectangle(0, 0, outW, outH);
        var dstData = result.LockBits(dstRect, ImageLockMode.WriteOnly,
            System.Drawing.Imaging.PixelFormat.Format24bppRgb);

        try
        {
            int srcStride = srcData.Stride;
            int dstStride = dstData.Stride;
            byte[] srcPixels = new byte[srcStride * original.Height];
            byte[] dstPixels = new byte[dstStride * outH];
            Marshal.Copy(srcData.Scan0, srcPixels, 0, srcPixels.Length);

            int binArea = bin * bin;
            for (int oy = 0; oy < outH; oy++)
            {
                int dstRow = oy * dstStride;
                for (int ox = 0; ox < outW; ox++)
                {
                    int sumB = 0, sumG = 0, sumR = 0;
                    for (int dy = 0; dy < bin; dy++)
                    {
                        int srcRow = (oy * bin + dy) * srcStride;
                        for (int dx = 0; dx < bin; dx++)
                        {
                            int srcIdx = srcRow + (ox * bin + dx) * 3;
                            sumB += srcPixels[srcIdx];
                            sumG += srcPixels[srcIdx + 1];
                            sumR += srcPixels[srcIdx + 2];
                        }
                    }
                    int dstIdx = dstRow + ox * 3;
                    dstPixels[dstIdx] = (byte)(sumB / binArea);
                    dstPixels[dstIdx + 1] = (byte)(sumG / binArea);
                    dstPixels[dstIdx + 2] = (byte)(sumR / binArea);
                }
            }

            Marshal.Copy(dstPixels, 0, dstData.Scan0, dstPixels.Length);
        }
        finally
        {
            original.UnlockBits(srcData);
            result.UnlockBits(dstData);
        }

        return result;
    }

    public void Dispose()
    {
        _maskrcnn?.Dispose();
    }
}
