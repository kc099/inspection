using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text.Json;
using System.Threading;
using Microsoft.ML.OnnxRuntime;
using Microsoft.ML.OnnxRuntime.Tensors;

namespace RoboViz;

/// <summary>
/// PatchCore ONNX inference wrapper for O-Ring anomaly detection.
/// Input:  [1, 3, 640, 640] float32, RGB, [0-1]
/// Output: anomaly_score [1] + anomaly_map [1, 1, 640, 640]
/// Attempts CUDA GPU with pre-flight checks; falls back to CPU automatically.
/// </summary>
public class PatchCoreDetector : IDisposable
{
    private InferenceSession? _session;
    private string? _inputName;
    private string? _scoreOutputName;
    private string? _mapOutputName;
    private readonly ThreadLocal<float[]> _inputBuffer;
    private const int InputSize = 640;
    private const int BufferLength = 1 * 3 * InputSize * InputSize;

    private float _threshold;
    private readonly bool _useGpu;

    public string ActiveProvider { get; private set; } = "CPU";
    public string? GpuError { get; private set; }
    public float Threshold => _threshold;
    public bool IsLoaded => _session != null;

    public PatchCoreDetector(bool useGpu = true)
    {
        _useGpu = useGpu;
        _inputBuffer = new ThreadLocal<float[]>(() => new float[BufferLength]);
    }

    /// <summary>
    /// Load a PatchCore ONNX model. Disposes any previously loaded model first.
    /// </summary>
    public void LoadModel(string modelPath, float threshold, IProgress<string>? progress = null)
    {
        Unload();

        if (_useGpu)
            MaskRCNNDetector.EnsureNvidiaLibsOnPath();

        var options = new SessionOptions
        {
            GraphOptimizationLevel = GraphOptimizationLevel.ORT_ENABLE_ALL,
            InterOpNumThreads = 4,
            IntraOpNumThreads = 4,
            EnableMemoryPattern = true
        };

        if (_useGpu)
        {
            try
            {
                var cudaOptions = new OrtCUDAProviderOptions();
                cudaOptions.UpdateOptions(new Dictionary<string, string>
                {
                    ["device_id"]                   = "0",
                    ["cudnn_conv_algo_search"]       = "DEFAULT",
                    ["cudnn_conv_use_max_workspace"] = "1",
                    ["do_copy_in_default_stream"]    = "1",
                    ["arena_extend_strategy"]        = "kSameAsRequested",
                });
                options.AppendExecutionProvider_CUDA(cudaOptions);
                ActiveProvider = "CUDA (GPU)";
                MaskRCNNDetector.LogDiag("  PatchCore CUDA EP appended to session options.");
            }
            catch (Exception ex)
            {
                ActiveProvider = "CPU (CUDA unavailable)";
                GpuError = ex.Message;
                MaskRCNNDetector.LogDiag($"  PatchCore CUDA EP failed: {ex}");
            }
        }

        _threshold = threshold;
        progress?.Report("Loading PatchCore ONNX session...");
        
        try
        {
            _session = new InferenceSession(modelPath, options);
            _inputName = _session.InputMetadata.Keys.First();

            // Resolve output names (order may differ between models)
            var outputNames = _session.OutputMetadata.Keys.ToList();
            _scoreOutputName = outputNames.FirstOrDefault(n => n.Contains("score", StringComparison.OrdinalIgnoreCase))
                               ?? outputNames[0];
            _mapOutputName = outputNames.FirstOrDefault(n => n.Contains("map", StringComparison.OrdinalIgnoreCase))
                             ?? outputNames[1];
            progress?.Report($"PatchCore outputs: score='{_scoreOutputName}', map='{_mapOutputName}' | threshold={threshold:F2}");

            // Log actual tensor element types to confirm fp16 vs fp32
            var inputMeta = _session.InputMetadata[_inputName];
            var outMeta = _session.OutputMetadata[_scoreOutputName!];
            MaskRCNNDetector.LogDiag($"PatchCore session created. Provider: {ActiveProvider}");
            MaskRCNNDetector.LogDiag($"  Input  '{_inputName}': elementType={inputMeta.ElementType}  shape=[{string.Join(",", inputMeta.Dimensions)}]");
            MaskRCNNDetector.LogDiag($"  Output '{_scoreOutputName}': elementType={outMeta.ElementType}  shape=[{string.Join(",", outMeta.Dimensions)}]");

            if (ActiveProvider.StartsWith("CUDA"))
                RunWarmUp(progress);
        }
        catch (Exception ex)
        {
            MaskRCNNDetector.LogDiag($"PatchCore session creation failed: {ex}");
            ActiveProvider = "CPU (Session creation failed)";
            GpuError = ex.Message;
            throw; // Re-throw to let caller handle
        }
    }

    private void ResolveOutputNames()
    {
        var outputNames = _session!.OutputMetadata.Keys.ToList();
        _scoreOutputName = outputNames.FirstOrDefault(n => n.Contains("score", StringComparison.OrdinalIgnoreCase))
                           ?? outputNames[0];
        _mapOutputName = outputNames.FirstOrDefault(n => n.Contains("map", StringComparison.OrdinalIgnoreCase))
                         ?? outputNames[1];
        MaskRCNNDetector.LogDiag(
            $"PatchCore outputs: score='{_scoreOutputName}', map='{_mapOutputName}' | threshold={_threshold:F2}");
    }

    private bool RunWarmUp(IProgress<string>? progress)
    {
        var buffer = _inputBuffer.Value!;
        var tensor = new DenseTensor<float>(buffer, [1, 3, InputSize, InputSize]);
        var inputs = new List<NamedOnnxValue>
        {
            NamedOnnxValue.CreateFromTensor(_inputName!, tensor)
        };

        for (int i = 0; i < 3; i++)
        {
            progress?.Report($"PatchCore GPU warmup {i + 1}/3 (may take 30-60s on PCIe Gen2)...");
            MaskRCNNDetector.LogDiag($"  PatchCore warmup {i + 1}/3...");
            var sw = Stopwatch.StartNew();
            try
            {
                using var _ = _session!.Run(inputs);
                sw.Stop();
                MaskRCNNDetector.LogDiag($"  PatchCore warmup {i + 1}/3 completed in {sw.ElapsedMilliseconds}ms.");
            }
            catch (Exception ex)
            {
                sw.Stop();
                MaskRCNNDetector.LogDiag($"  PatchCore warmup {i + 1}/3 FAILED after {sw.ElapsedMilliseconds}ms: {ex}");
                MaskRCNNDetector.LogDiag($"  Exception type: {ex.GetType().Name}");
                MaskRCNNDetector.LogDiag($"  Inner exception: {ex.InnerException?.Message}");
                ActiveProvider = "CPU (CUDA warmup failed)";
                GpuError = $"Warmup failed: {ex.Message}";
                progress?.Report($"PatchCore GPU warmup failed – falling back to CPU. {ex.Message}");
                return false;
            }
        }

        MaskRCNNDetector.LogDiag("  PatchCore warmup complete.");
        return true;
    }

    /// <summary>
    /// Unload the current model to free GPU memory.
    /// </summary>
    public void Unload()
    {
        _session?.Dispose();
        _session = null;
        _inputName = null;
        _scoreOutputName = null;
        _mapOutputName = null;
    }

    /// <summary>
    /// Run anomaly detection on a preprocessed 640x640 image.
    /// </summary>
    public PatchCoreResult Detect(Bitmap image, out long tensorMs, out long inferenceMs)
    {
        if (_session == null || _inputName == null)
            throw new InvalidOperationException("No PatchCore model loaded.");

        var sw = Stopwatch.StartNew();
        var buffer = _inputBuffer.Value!;
        FillInputBuffer(image, buffer);
        
        // Create input tensor - using float for both FP16 and FP32 models
        // ONNX Runtime handles conversion internally if needed
        var tensor = new DenseTensor<float>(buffer, [1, 3, image.Height, image.Width]);
        var inputs = new List<NamedOnnxValue>
        {
            NamedOnnxValue.CreateFromTensor(_inputName, tensor)
        };
        tensorMs = sw.ElapsedMilliseconds;

        sw.Restart();
        using var results = _session.Run(inputs);
        inferenceMs = sw.ElapsedMilliseconds;

        // Look up outputs by name (not index — order is not guaranteed)
        var resultList = results.ToList();
        var scoreResult = resultList.First(r => r.Name == _scoreOutputName);
        var mapResult = resultList.First(r => r.Name == _mapOutputName);

        float score = scoreResult.AsTensor<float>()[0];
        var mapTensorF32 = mapResult.AsTensor<float>();
        var mapTensorF16 = mapTensorF32 == null ? mapResult.AsTensor<Float16>() : null;

        int mapH = mapTensorF32?.Dimensions[2] ?? mapTensorF16!.Dimensions[2];
        int mapW = mapTensorF32?.Dimensions[3] ?? mapTensorF16!.Dimensions[3];

        var map = new float[mapH, mapW];
        if (mapTensorF32 is DenseTensor<float> denseF32)
        {
            // Fast path: read contiguous buffer directly (avoids per-element indexer overhead)
            var span = denseF32.Buffer.Span;
            for (int y = 0; y < mapH; y++)
            {
                int srcOffset = y * mapW; // shape is [1, 1, H, W] so data starts at offset 0
                for (int x = 0; x < mapW; x++)
                    map[y, x] = span[srcOffset + x];
            }
        }
        else if (mapTensorF16 is DenseTensor<Float16> denseF16)
        {
            var span = denseF16.Buffer.Span;
            for (int y = 0; y < mapH; y++)
            {
                int srcOffset = y * mapW;
                for (int x = 0; x < mapW; x++)
                    map[y, x] = (float)span[srcOffset + x];
            }
        }
        else if (mapTensorF32 != null)
        {
            for (int y = 0; y < mapH; y++)
                for (int x = 0; x < mapW; x++)
                    map[y, x] = mapTensorF32[0, 0, y, x];
        }
        else
        {
            for (int y = 0; y < mapH; y++)
                for (int x = 0; x < mapW; x++)
                    map[y, x] = (float)mapTensorF16![0, 0, y, x];
        }

        return new PatchCoreResult
        {
            AnomalyScore = score,
            AnomalyMap = map,
            IsAnomaly = score > _threshold
        };
    }

    /// <summary>
    /// Preprocess: resize to 660x660, center-crop to 640x640.
    /// </summary>
    public static Bitmap Preprocess(Bitmap input, int resizeSize = 660, int cropSize = 640)
    {
        var resized = new Bitmap(resizeSize, resizeSize, PixelFormat.Format24bppRgb);
        using (var g = Graphics.FromImage(resized))
        {
            g.InterpolationMode = InterpolationMode.HighQualityBicubic;
            g.PixelOffsetMode = PixelOffsetMode.HighQuality;
            g.DrawImage(input, 0, 0, resizeSize, resizeSize);
        }

        int margin = (resizeSize - cropSize) / 2;
        var cropped = new Bitmap(cropSize, cropSize, PixelFormat.Format24bppRgb);
        using (var g = Graphics.FromImage(cropped))
        {
            g.DrawImage(resized,
                new Rectangle(0, 0, cropSize, cropSize),
                new Rectangle(margin, margin, cropSize, cropSize),
                GraphicsUnit.Pixel);
        }

        resized.Dispose();
        return cropped;
    }

    /// <summary>
    /// Draw a heatmap overlay on the image (jet colormap).
    /// </summary>
    public static Bitmap DrawHeatmapOverlay(Bitmap image, float[,] anomalyMap, float alpha = 0.5f)
    {
        int h = image.Height, w = image.Width;
        int mapH = anomalyMap.GetLength(0), mapW = anomalyMap.GetLength(1);

        float min = float.MaxValue, max = float.MinValue;
        for (int y = 0; y < mapH; y++)
            for (int x = 0; x < mapW; x++)
            {
                float v = anomalyMap[y, x];
                if (v < min) min = v;
                if (v > max) max = v;
            }
        float range = max - min;
        if (range < 1e-6f) range = 1f;

        var overlay = new Bitmap(image);
        var rect = new Rectangle(0, 0, w, h);
        var bmpData = overlay.LockBits(rect, ImageLockMode.ReadWrite, PixelFormat.Format24bppRgb);
        try
        {
            int stride = bmpData.Stride;
            byte[] pixels = new byte[stride * h];
            Marshal.Copy(bmpData.Scan0, pixels, 0, pixels.Length);

            for (int y = 0; y < h; y++)
            {
                int my = Math.Min((int)((float)y / h * mapH), mapH - 1);
                int rowOff = y * stride;
                for (int x = 0; x < w; x++)
                {
                    int mx = Math.Min((int)((float)x / w * mapW), mapW - 1);
                    float norm = Math.Clamp((anomalyMap[my, mx] - min) / range, 0f, 1f);
                    JetColor(norm, out byte hr, out byte hg, out byte hb);

                    int idx = rowOff + x * 3;
                    pixels[idx] = (byte)(pixels[idx] * (1 - alpha) + hb * alpha);
                    pixels[idx + 1] = (byte)(pixels[idx + 1] * (1 - alpha) + hg * alpha);
                    pixels[idx + 2] = (byte)(pixels[idx + 2] * (1 - alpha) + hr * alpha);
                }
            }

            Marshal.Copy(pixels, 0, bmpData.Scan0, pixels.Length);
        }
        finally
        {
            overlay.UnlockBits(bmpData);
        }
        return overlay;
    }

    private static void JetColor(float t, out byte r, out byte g, out byte b)
    {
        if (t < 0.25f)      { float s = t / 0.25f;           r = 0;   g = (byte)(255 * s); b = 255; }
        else if (t < 0.5f)  { float s = (t - 0.25f) / 0.25f; r = 0;   g = 255;             b = (byte)(255 * (1 - s)); }
        else if (t < 0.75f) { float s = (t - 0.5f) / 0.25f;  r = (byte)(255 * s); g = 255; b = 0; }
        else                { float s = (t - 0.75f) / 0.25f;  r = 255; g = (byte)(255 * (1 - s)); b = 0; }
    }

    private static void FillInputBuffer(Bitmap bmp, float[] buffer)
    {
        int w = bmp.Width, h = bmp.Height;
        int planeSize = h * w;

        var rect = new Rectangle(0, 0, w, h);
        var bmpData = bmp.LockBits(rect, ImageLockMode.ReadOnly, PixelFormat.Format24bppRgb);
        try
        {
            int stride = bmpData.Stride;
            byte[] pixels = new byte[stride * h];
            Marshal.Copy(bmpData.Scan0, pixels, 0, pixels.Length);

            for (int y = 0; y < h; y++)
            {
                int rowOffset = y * stride;
                int yOffset = y * w;
                for (int x = 0; x < w; x++)
                {
                    int idx = rowOffset + x * 3;
                    buffer[yOffset + x] = pixels[idx + 2] / 255f;
                    buffer[planeSize + yOffset + x] = pixels[idx + 1] / 255f;
                    buffer[planeSize * 2 + yOffset + x] = pixels[idx] / 255f;
                }
            }
        }
        finally
        {
            bmp.UnlockBits(bmpData);
        }
    }

    /// <summary>
    /// Load metadata from the JSON produced by the export script.
    /// </summary>
    public static PatchCoreMetadata? LoadMetadata(string jsonPath)
    {
        if (!File.Exists(jsonPath)) return null;
        string json = File.ReadAllText(jsonPath);
        return JsonSerializer.Deserialize<PatchCoreMetadata>(json);
    }

    public void Dispose()
    {
        _session?.Dispose();
        _inputBuffer?.Dispose();
    }
}
