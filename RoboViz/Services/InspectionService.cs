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
    private readonly PatchCoreDetector _patchcore;
    private readonly YoloSegDetector _yoloSeg;
    private readonly float _scoreThreshold;
    private Dictionary<string, MetricThreshold> _thresholds;
    private string _currentModel;

    public string ActiveProvider => _maskrcnn.ActiveProvider;
    public string? GpuError => _maskrcnn.GpuError;
    public string CurrentModel => _currentModel;
    public Dictionary<string, MetricThreshold> CurrentThresholds => _thresholds;

    public InspectionService(string maskrcnnModelPath, float scoreThreshold = 0.5f,
        bool useGpu = true, IProgress<string>? progress = null,
        string modelName = "Model 2")
    {
        _maskrcnn = new MaskRCNNDetector(maskrcnnModelPath, useGpu, progress);
        _patchcore = new PatchCoreDetector(useGpu);
        _yoloSeg = new YoloSegDetector();
        _scoreThreshold = scoreThreshold;
        _currentModel = modelName;
        _thresholds = LoadThresholdsForModel(modelName);

        LoadPatchCoreModel(modelName, progress);
        LoadYoloSegModel(progress);
    }

    private void LoadYoloSegModel(IProgress<string>? progress)
    {
        string assetsDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Assets");
        string yoloPath = Path.Combine(assetsDir, "yolo11_seg_oring.onnx");

        if (!File.Exists(yoloPath))
        {
            progress?.Report("YOLO segmentation model not found — using legacy PatchCore preprocessing.");
            MaskRCNNDetector.LogDiag($"[YOLO-Seg] Model not found at: {yoloPath}");
            return;
        }

        _yoloSeg.LoadModel(yoloPath, progress);
    }

    public void SwitchModel(string modelName, IProgress<string>? progress = null)
    {
        if (_currentModel == modelName) return;
        _currentModel = modelName;
        _thresholds = LoadThresholdsForModel(modelName);

        // Unload previous PatchCore model and load new one to save GPU memory
        LoadPatchCoreModel(modelName, progress);
    }

    private const float PatchCoreHardcodedThreshold = 15.0f;

    private void LoadPatchCoreModel(string modelName, IProgress<string>? progress)
    {
        string assetsDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Assets");
        string suffix = modelName == "Model 1" ? "model1" : "model2";
        string onnxPath = Path.Combine(assetsDir, $"patchcore_{suffix}_cropped_resnet50_fp16.onnx");

        if (!File.Exists(onnxPath))
        {
            progress?.Report($"PatchCore model not found: {onnxPath}");
            return;
        }

        _patchcore.LoadModel(onnxPath, PatchCoreHardcodedThreshold, progress);
    }

    private static Dictionary<string, MetricThreshold> LoadThresholdsForModel(string modelName)
    {
        string fileName = modelName == "Model 1"
            ? "model1_tuned_thresholds.json"
            : "model2_tuned_thresholds.json";
        string path = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Assets", fileName);
        return ThresholdConfig.LoadThresholds(path);
    }

    /// <summary>
    /// Shared geometric measurement + threshold evaluation.
    /// Returns the partially filled result and the geo verdict.
    /// If the result is terminal (ERROR / REWORK / REJECT), geoVerdict != "PASS".
    /// </summary>
    private InspectionResult RunGeoEvaluation(
        Bitmap rawImage, string detectorType, Stopwatch totalSw,
        out string geoVerdict)
    {
        var geoSw = Stopwatch.StartNew();
        var geoResult = OringMeasurement.Measure(rawImage);
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
        var (metricResults, verdict, failReasons) = ThresholdConfig.Evaluate(normedMetrics, _thresholds);
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
            result.OverlayImage = OringMeasurement.DrawGeometricOverlay(rawImage, geoResult);
            result.TotalMs = totalSw.ElapsedMilliseconds;
        }

        return result;
    }

    /// <summary>
    /// Run Mask R-CNN inspection (CAM 1, 3).
    /// </summary>
    public InspectionResult InspectMaskRCNN(Bitmap rawImage)
    {
        var totalSw = Stopwatch.StartNew();
        var result = RunGeoEvaluation(rawImage, "MaskRCNN", totalSw, out string geoVerdict);
        if (geoVerdict != "PASS")
            return result;

        var prepSw = Stopwatch.StartNew();
        using var img720 = OringMeasurement.BinCrop720(rawImage);
        long prepMs = prepSw.ElapsedMilliseconds;

        var detections = _maskrcnn.Detect(img720, _scoreThreshold,
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
            result.OverlayImage = DrawDefectOverlay(img720, detections);
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

    /// <summary>
    /// Run PatchCore inspection (CAM 2, 4).
    /// No geometric measurement — side-view cameras may not have a visible hole.
    /// Pipeline: 4×4 bin ? YOLO seg ? crop ? resize 384×384 ? PatchCore inference.
    /// </summary>
    public InspectionResult InspectPatchCore(Bitmap rawImage)
    {
        var totalSw = Stopwatch.StartNew();

        var result = new InspectionResult
        {
            DetectorType = "PatchCore",
        };

        if (!_patchcore.IsLoaded)
        {
            result.Verdict = "ERROR";
            result.ErrorMessage = "PatchCore model not loaded";
            result.OverlayImage = rawImage;
            result.TotalMs = totalSw.ElapsedMilliseconds;
            return result;
        }

        // Preprocess: 4x4 bin 512x384, YOLO seg crop, resize 384x384
        var prepSw = Stopwatch.StartNew();
        using var patchInput = PreprocessPatchCoreImage(rawImage, out long yoloSegMs);
        long prepMs = prepSw.ElapsedMilliseconds;

        var pcResult = _patchcore.Detect(patchInput, out long tensorMs, out long inferenceMs);

        result.TensorMs = tensorMs;
        result.InferenceMs = inferenceMs;
        result.PrepMs = prepMs;
        result.AnomalyScore = pcResult.AnomalyScore;
        result.AnomalyThreshold = _patchcore.Threshold;
        result.HasDefect = pcResult.IsAnomaly;

        if (pcResult.IsAnomaly)
        {
            result.Verdict = "REJECT";
            result.FailReasons = [$"Anomaly detected (score: {pcResult.AnomalyScore:F2} > {_patchcore.Threshold:F2})"];

            var overlaySw = Stopwatch.StartNew();
            result.OverlayImage = PatchCoreDetector.DrawHeatmapOverlay(patchInput, pcResult.AnomalyMap);
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
    /// Preprocess an image for PatchCore inference.
    /// New pipeline: 4x4 bin 2048x1536 to 512x384, YOLO seg crop, resize 384x384.
    /// Fallback: BinCrop720, resize 400, center-crop 384x384.
    /// </summary>
    private Bitmap PreprocessPatchCoreImage(Bitmap rawImage, out long yoloSegMs)
    {
        yoloSegMs = 0;

        // Step 1: 4x4 binning 2048x1536 to 512x384
        using var binned = BinTo512x384(rawImage);

        if (_yoloSeg.IsLoaded)
        {
            // Step 2: YOLO segmentation on 512x384 directly (model accepts this size)
            var cropRect = _yoloSeg.Segment(binned, out yoloSegMs);

            Bitmap source;
            if (cropRect is { Width: > 10, Height: > 10 })
            {
                source = binned.Clone(cropRect.Value, binned.PixelFormat);
            }
            else
            {
                Debug.WriteLine("[PatchCore] YOLO seg returned no crop, using full binned image.");
                source = new Bitmap(binned);
            }

            // Step 3: Resize crop to 384x384
            var result = new Bitmap(384, 384, System.Drawing.Imaging.PixelFormat.Format24bppRgb);
            using (var g = Graphics.FromImage(result))
            {
                g.InterpolationMode = System.Drawing.Drawing2D.InterpolationMode.HighQualityBicubic;
                g.PixelOffsetMode = System.Drawing.Drawing2D.PixelOffsetMode.HighQuality;
                g.DrawImage(source, 0, 0, 384, 384);
            }
            source.Dispose();
            return result;
        }

        // Legacy fallback: BinCrop720, resize 400, center-crop 384
        Debug.WriteLine("[PatchCore] YOLO seg not loaded, using legacy BinCrop720 preprocessing.");
        using var img720 = OringMeasurement.BinCrop720(rawImage);
        return PatchCoreDetector.Preprocess(img720);
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
        _patchcore?.Dispose();
        _yoloSeg?.Dispose();
    }
}
