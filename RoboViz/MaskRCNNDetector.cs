using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Threading;
using Microsoft.ML.OnnxRuntime;
using Microsoft.ML.OnnxRuntime.Tensors;

namespace RoboViz;

/// <summary>
/// Result from a single Mask R-CNN inference pass.
/// </summary>
public class Detection
{
    public float X1 { get; set; }
    public float Y1 { get; set; }
    public float X2 { get; set; }
    public float Y2 { get; set; }
    public float Score { get; set; }
    public int Label { get; set; }
    public float[,] Mask { get; set; } = null!;
}

/// <summary>
/// Mask R-CNN ONNX inference wrapper for O-Ring defect detection.
/// Two classes: 0 = background, 1 = defect.
/// Uses CUDA GPU with cuDNN exhaustive algo search + warmup.
/// </summary>
public class MaskRCNNDetector : IDisposable
{
    private readonly InferenceSession _session;
    private readonly string _inputName;
    private readonly ThreadLocal<float[]> _inputBuffer;
    private const int InputSize = 720;
    private const int BufferLength = 1 * 3 * InputSize * InputSize;

    public string ActiveProvider { get; private set; } = "CPU";
    public string? GpuError { get; private set; }

    public MaskRCNNDetector(string modelPath, bool useGpu = true, IProgress<string>? progress = null)
    {
        if (useGpu)
            EnsureNvidiaLibsOnPath();

        var options = new SessionOptions
        {
            GraphOptimizationLevel = GraphOptimizationLevel.ORT_ENABLE_ALL,
            InterOpNumThreads = 4,
            IntraOpNumThreads = 4,
            EnableMemoryPattern = true
        };

        if (useGpu)
        {
            try
            {
                var cudaOptions = new OrtCUDAProviderOptions();
                cudaOptions.UpdateOptions(new Dictionary<string, string>
                {
                    ["device_id"] = "0",
                    ["cudnn_conv_algo_search"] = "EXHAUSTIVE",
                    ["cudnn_conv_use_max_workspace"] = "1",
                    ["do_copy_in_default_stream"] = "1",
                    ["arena_extend_strategy"] = "kNextPowerOfTwo",
                });
                options.AppendExecutionProvider_CUDA(cudaOptions);
                ActiveProvider = "CUDA (GPU)";
            }
            catch (Exception ex)
            {
                ActiveProvider = "CPU (CUDA unavailable)";
                GpuError = ex.Message;
            }
        }

        progress?.Report("Loading ONNX session...");
        _session = new InferenceSession(modelPath, options);
        _inputName = _session.InputMetadata.Keys.First();

        _inputBuffer = new ThreadLocal<float[]>(() => new float[BufferLength]);

        if (ActiveProvider.StartsWith("CUDA"))
        {
            WarmUp(progress);
        }
    }

    private void WarmUp(IProgress<string>? progress)
    {
        var buffer = _inputBuffer.Value!;
        var tensor = new DenseTensor<float>(buffer, [1, 3, InputSize, InputSize]);
        var inputs = new List<NamedOnnxValue>
        {
            NamedOnnxValue.CreateFromTensor(_inputName, tensor)
        };

        for (int i = 0; i < 3; i++)
        {
            progress?.Report($"GPU warmup {i + 1}/3...");
            using var _ = _session.Run(inputs);
        }
    }

    /// <summary>
    /// Prepend pip-installed NVIDIA library paths (cuDNN 9, cuBLAS 12) to PATH
    /// so the native ORT CUDA provider can find them at runtime.
    /// </summary>
    private static void EnsureNvidiaLibsOnPath()
    {
        string sitePackages = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.UserProfile),
            "anaconda3", "Lib", "site-packages", "nvidia");

        if (!Directory.Exists(sitePackages))
            return;

        var path = Environment.GetEnvironmentVariable("PATH") ?? "";
        var newPaths = new List<string>();

        string[] libs = ["cudnn", "cublas", "cufft", "curand", "cusolver", "cusparse"];
        foreach (var lib in libs)
        {
            string binDir = Path.Combine(sitePackages, lib, "bin");
            if (Directory.Exists(binDir) && !path.Contains(binDir, StringComparison.OrdinalIgnoreCase))
                newPaths.Add(binDir);
        }

        if (newPaths.Count > 0)
        {
            string combined = string.Join(';', newPaths) + ";" + path;
            Environment.SetEnvironmentVariable("PATH", combined);
        }
    }

    private const int MaxDetections = 5;

    /// <summary>
    /// Run inference on a 720×720 Bitmap image. Returns at most 5 detections.
    /// </summary>
    public List<Detection> Detect(Bitmap image, float scoreThreshold, out long tensorMs, out long inferenceMs)
    {
        var sw = Stopwatch.StartNew();
        var buffer = _inputBuffer.Value!;
        FillInputBuffer(image, buffer);
        var tensor = new DenseTensor<float>(buffer, [1, 3, image.Height, image.Width]);
        tensorMs = sw.ElapsedMilliseconds;

        var inputs = new List<NamedOnnxValue>
        {
            NamedOnnxValue.CreateFromTensor(_inputName, tensor)
        };

        sw.Restart();
        using var results = _session.Run(inputs);
        inferenceMs = sw.ElapsedMilliseconds;

        var scores = results.ElementAt(2).AsTensor<float>();
        int numDetections = (int)scores.Length;

        // Early exit if no detections above threshold
        var detections = new List<Detection>(MaxDetections);
        bool anyAbove = false;
        for (int i = 0; i < numDetections && detections.Count < MaxDetections; i++)
        {
            if (scores[i] >= scoreThreshold) { anyAbove = true; break; }
        }
        if (!anyAbove) return detections;

        var boxes = results.ElementAt(0).AsTensor<float>();
        var labels = results.ElementAt(1).AsTensor<long>();
        var masks = results.ElementAt(3).AsTensor<float>();

        int maskH = masks.Dimensions.Length >= 3 ? masks.Dimensions[2] : image.Height;
        int maskW = masks.Dimensions.Length >= 4 ? masks.Dimensions[3] : image.Width;
        int maskPlane = maskH * maskW;

        for (int i = 0; i < numDetections && detections.Count < MaxDetections; i++)
        {
            float score = scores[i];
            if (score < scoreThreshold)
                continue;

            var det = new Detection
            {
                X1 = boxes[i, 0],
                Y1 = boxes[i, 1],
                X2 = boxes[i, 2],
                Y2 = boxes[i, 3],
                Score = score,
                Label = (int)labels[i],
                Mask = new float[maskH, maskW]
            };

            // Copy mask using span for contiguous access
            for (int y = 0; y < maskH; y++)
            {
                int srcOffset = y * maskW;
                for (int x = 0; x < maskW; x++)
                    det.Mask[y, x] = masks[i, 0, y, x];
            }

            detections.Add(det);
        }

        return detections;
    }

    /// <summary>
    /// Fill buffer from Bitmap. BGR ? RGB planar, [0..1]. Thread-safe (uses caller's buffer).
    /// </summary>
    private static void FillInputBuffer(Bitmap bmp, float[] buffer)
    {
        int w = bmp.Width;
        int h = bmp.Height;
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
                    buffer[yOffset + x] = pixels[idx + 2] / 255f;             // R
                    buffer[planeSize + yOffset + x] = pixels[idx + 1] / 255f;  // G
                    buffer[planeSize * 2 + yOffset + x] = pixels[idx] / 255f;  // B
                }
            }
        }
        finally
        {
            bmp.UnlockBits(bmpData);
        }
    }

    public void Dispose()
    {
        _session?.Dispose();
        _inputBuffer?.Dispose();
    }
}
