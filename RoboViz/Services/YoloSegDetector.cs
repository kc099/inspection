using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using Microsoft.ML.OnnxRuntime;
using Microsoft.ML.OnnxRuntime.Tensors;

namespace RoboViz;

/// <summary>
/// YOLO11-seg ONNX inference for O-ring segmentation (CPU only).
/// Segments the O-ring from the background and returns the crop bounding box.
///
/// Pipeline:
///   Camera 2048×1536 → 4×4 bin 512×384 → YOLO inference
///   → Segmentation mask → Bounding box in 512×384 coords
///
/// Input:   "images"   [1, 3, 384, 512]  float32, RGB, [0-1]
/// Output0: "output0"  [1, 37, 4032]     float32  (cx, cy, w, h, conf, 32 mask coeffs)
/// Output1: "output1"  [1, 32, 96, 128]  float32  (mask prototypes)
/// </summary>
public class YoloSegDetector : IDisposable
{
    private InferenceSession? _session;
    private string? _inputName;

    private const int InputW = 512;
    private const int InputH = 384;
    private const int NumDetections = 4032;
    private const int NumMaskCoeffs = 32;
    private const int ProtoW = 128;
    private const int ProtoH = 96;
    private const float ConfThreshold = 0.25f;
    private const float MaskThreshold = 0.5f;

    public bool IsLoaded => _session != null;

    public void LoadModel(string modelPath, IProgress<string>? progress = null)
    {
        _session?.Dispose();

        var options = new SessionOptions
        {
            GraphOptimizationLevel = GraphOptimizationLevel.ORT_ENABLE_ALL,
            IntraOpNumThreads = 4,
            InterOpNumThreads = 2,
        };

        progress?.Report("Loading YOLO segmentation model (CPU)...");
        _session = new InferenceSession(modelPath, options);
        _inputName = _session.InputMetadata.Keys.First();

        var outputNames = _session.OutputMetadata.Keys.ToList();
        MaskRCNNDetector.LogDiag($"[YOLO-Seg] Loaded: {Path.GetFileName(modelPath)}, " +
            $"input='{_inputName}', outputs=[{string.Join(", ", outputNames)}]");
        progress?.Report("YOLO segmentation model loaded (CPU).");
    }

    /// <summary>
    /// Segment the O-ring from a 512×384 binned image.
    /// Returns the crop rectangle in 512×384 coordinates, or null if no O-ring detected.
    /// </summary>
    public Rectangle? Segment(Bitmap image512x384, out long inferenceMs)
    {
        inferenceMs = 0;
        if (_session == null || _inputName == null)
            return null;

        // Step 1: Fill [1, 3, 384, 512] input tensor directly (no letterboxing)
        var inputBuffer = new float[3 * InputH * InputW];
        FillInputBuffer(image512x384, inputBuffer);

        var tensor = new DenseTensor<float>(inputBuffer, [1, 3, InputH, InputW]);
        var inputs = new List<NamedOnnxValue>
        {
            NamedOnnxValue.CreateFromTensor(_inputName, tensor)
        };

        // Step 2: Inference
        var sw = Stopwatch.StartNew();
        using var results = _session.Run(inputs);
        inferenceMs = sw.ElapsedMilliseconds;

        var resultList = results.ToList();
        // output0: [1, 37, 4032] — detections (attributes × candidates)
        var det0 = resultList[0].AsTensor<float>() as DenseTensor<float>;
        // output1: [1, 32, 96, 128] — mask prototypes
        var protos = resultList[1].AsTensor<float>() as DenseTensor<float>;

        if (det0 == null || protos == null)
        {
            Debug.WriteLine("[YOLO-Seg] Failed to read output tensors.");
            return null;
        }

        var detSpan = det0.Buffer.Span;
        var protoSpan = protos.Buffer.Span;

        // Step 3: Find best detection (highest confidence > threshold)
        // output0 layout: [1, 37, 4032] — detSpan index = attr * 4032 + det_idx
        int bestIdx = -1;
        float bestConf = ConfThreshold;
        for (int i = 0; i < NumDetections; i++)
        {
            float conf = detSpan[4 * NumDetections + i]; // attribute 4 = class confidence
            if (conf > bestConf)
            {
                bestConf = conf;
                bestIdx = i;
            }
        }

        if (bestIdx < 0)
        {
            Debug.WriteLine("[YOLO-Seg] No detection above confidence threshold.");
            return null;
        }

        Debug.WriteLine($"[YOLO-Seg] Best detection: idx={bestIdx}, conf={bestConf:F3}");

        // Step 4: Extract 32 mask coefficients for best detection
        Span<float> coeffs = stackalloc float[NumMaskCoeffs];
        for (int k = 0; k < NumMaskCoeffs; k++)
            coeffs[k] = detSpan[(5 + k) * NumDetections + bestIdx];

        // Step 5: Compute mask at 128×96 = sigmoid(coeffs · protos)
        // proto layout: [1, 32, 96, 128] — protoSpan index = k * ProtoH*ProtoW + y * ProtoW + x
        int minX = ProtoW, minY = ProtoH, maxX = -1, maxY = -1;
        int protoPlane = ProtoH * ProtoW;

        for (int y = 0; y < ProtoH; y++)
        {
            int yOff = y * ProtoW;
            for (int x = 0; x < ProtoW; x++)
            {
                float val = 0;
                int px = yOff + x;
                for (int k = 0; k < NumMaskCoeffs; k++)
                    val += coeffs[k] * protoSpan[k * protoPlane + px];

                // sigmoid + threshold
                float sigmoid = 1f / (1f + MathF.Exp(-val));
                if (sigmoid > MaskThreshold)
                {
                    if (x < minX) minX = x;
                    if (x > maxX) maxX = x;
                    if (y < minY) minY = y;
                    if (y > maxY) maxY = y;
                }
            }
        }

        if (maxX < 0)
        {
            Debug.WriteLine("[YOLO-Seg] Mask is empty after thresholding.");
            return null;
        }

        // Step 6: Scale from proto (128×96) to input (512×384)
        float scaleX = InputW / (float)ProtoW; // 4.0
        float scaleY = InputH / (float)ProtoH; // 4.0
        int bx1 = (int)(minX * scaleX);
        int by1 = (int)(minY * scaleY);
        int bx2 = (int)((maxX + 1) * scaleX);
        int by2 = (int)((maxY + 1) * scaleY);

        // Clamp to 512×384 image bounds
        bx1 = Math.Max(0, bx1);
        by1 = Math.Max(0, by1);
        bx2 = Math.Min(InputW, bx2);
        by2 = Math.Min(InputH, by2);

        int cropW = bx2 - bx1;
        int cropH = by2 - by1;

        if (cropW < 10 || cropH < 10)
        {
            Debug.WriteLine($"[YOLO-Seg] Crop too small: {cropW}x{cropH}.");
            return null;
        }

        Debug.WriteLine($"[YOLO-Seg] Crop: ({bx1},{by1})-({bx2},{by2}) = {cropW}x{cropH} on 512x384");
        return new Rectangle(bx1, by1, cropW, cropH);
    }

    /// <summary>
    /// Fill the input buffer: [3, 384, 512] float32, RGB, [0-1].
    /// No letterboxing — the model accepts 512×384 directly.
    /// </summary>
    private static void FillInputBuffer(Bitmap image, float[] buffer)
    {
        int w = image.Width;
        int h = image.Height;

        var rect = new Rectangle(0, 0, w, h);
        var bmpData = image.LockBits(rect, ImageLockMode.ReadOnly, PixelFormat.Format24bppRgb);
        try
        {
            int stride = bmpData.Stride;
            byte[] pixels = new byte[stride * h];
            Marshal.Copy(bmpData.Scan0, pixels, 0, pixels.Length);

            int planeR = 0;
            int planeG = h * w;
            int planeB = 2 * h * w;

            for (int y = 0; y < h; y++)
            {
                int srcRow = y * stride;
                int dstRow = y * w;

                for (int x = 0; x < w; x++)
                {
                    int px = srcRow + x * 3;
                    // BMP is BGR, model expects RGB
                    buffer[planeR + dstRow + x] = pixels[px + 2] / 255f; // R
                    buffer[planeG + dstRow + x] = pixels[px + 1] / 255f; // G
                    buffer[planeB + dstRow + x] = pixels[px]     / 255f; // B
                }
            }
        }
        catch { }
        finally
        {
            image.UnlockBits(bmpData);
        }
    }

    public void Dispose()
    {
        _session?.Dispose();
        _session = null;
    }
}
