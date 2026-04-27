using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using Microsoft.ML.OnnxRuntime;
using Microsoft.ML.OnnxRuntime.Tensors;

namespace RoboViz;

/// <summary>
/// Single YOLO11-seg detection (bbox in RAW image coordinates).
/// Mask coefficients are intentionally ignored — only bbox W/H are used.
/// </summary>
public readonly record struct YoloDetection(
    float X1, float Y1, float X2, float Y2, float Confidence, int ClassId)
{
    public float Width  => X2 - X1;
    public float Height => Y2 - Y1;
    public float Area   => Width * Height;
    public float CenterX => (X1 + X2) * 0.5f;
    public float CenterY => (Y1 + Y2) * 0.5f;
}

/// <summary>
/// CPU-only YOLO11n-seg ONNX inference for the cam2 contour replacement pipeline.
///
/// Pipeline:
///   Raw camera image (e.g. 2448×2048) ? straight resize to 640×512 ? ONNX ? bboxes
/// The Python verify script (verify_onnx_contour.py) uses the same straight-resize
/// preprocessing (no letterbox), so we mirror that here.
///
/// Input:   "images"  [1, 3, 512, 640] float32, RGB, [0-1]
/// Output0: "output0" [1, 40, 6720]    float32   (4 bbox + 4 class + 32 mask coeffs)
/// Output1: "output1" [1, 32, 128, 160] float32  (mask prototypes — ignored)
///
/// We only consume bbox geometry for outer/inner radius estimation, so the mask
/// prototypes are not parsed.
/// </summary>
public class YoloContourDetector : IDisposable
{
    private InferenceSession? _session;
    private string? _inputName;

    public const int InputW = 640;
    public const int InputH = 512;
    private const int NumDet = 6720;
    private const int NumClasses = 4; // cut, deformation, hole, tear
    private const int Stride = 4 + NumClasses + 32; // 40

    public float ConfThreshold { get; set; } = 0.25f;
    public float NmsIoU { get; set; } = 0.45f;

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

        progress?.Report("Loading YOLO contour model (CPU)...");
        _session = new InferenceSession(modelPath, options);
        _inputName = _session.InputMetadata.Keys.First();

        var inMeta = _session.InputMetadata[_inputName];
        var outNames = string.Join(", ", _session.OutputMetadata.Keys);
        MaskRCNNDetector.LogDiag(
            $"[YOLO-Contour] Loaded: {Path.GetFileName(modelPath)}, " +
            $"input='{_inputName}' shape=[{string.Join(",", inMeta.Dimensions)}], outputs=[{outNames}]");
        progress?.Report("YOLO contour model loaded (CPU).");
    }

    /// <summary>
    /// Detect bboxes on a raw camera image. Results are returned in RAW image
    /// coordinates (already scaled back from the 640×512 model space).
    /// </summary>
    public List<YoloDetection> Detect(Bitmap rawImage, out long inferenceMs)
    {
        inferenceMs = 0;
        if (_session == null || _inputName == null)
            return [];

        // 1. Resize raw ? 640×512 (straight stretch, matches verify_onnx_contour.py)
        using var resized = new Bitmap(InputW, InputH, PixelFormat.Format24bppRgb);
        using (var g = Graphics.FromImage(resized))
        {
            g.InterpolationMode = InterpolationMode.HighQualityBilinear;
            g.PixelOffsetMode = PixelOffsetMode.HighQuality;
            g.DrawImage(rawImage, 0, 0, InputW, InputH);
        }

        // 2. Build CHW float tensor, RGB, [0,1]
        var input = new float[3 * InputH * InputW];
        FillInputBuffer(resized, input);

        var tensor = new DenseTensor<float>(input, [1, 3, InputH, InputW]);
        var inputs = new List<NamedOnnxValue>
        {
            NamedOnnxValue.CreateFromTensor(_inputName, tensor)
        };

        // 3. Inference
        var sw = Stopwatch.StartNew();
        using var results = _session.Run(inputs);
        inferenceMs = sw.ElapsedMilliseconds;

        // 4. Parse output0 [1, 40, 6720] in YOLOv8/11 layout:
        //    rows 0..3   = cx, cy, w, h     (in 640×512 space)
        //    rows 4..7   = per-class scores (sigmoid already applied)
        //    rows 8..39  = mask coeffs      (ignored)
        var det0 = results.First().AsTensor<float>();
        if (det0 is not DenseTensor<float> dense)
        {
            Debug.WriteLine("[YOLO-Contour] Unexpected output tensor type.");
            return [];
        }
        var span = dense.Buffer.Span;

        // Scale factors: 640×512 model space ? raw image space
        float sx = (float)rawImage.Width  / InputW;
        float sy = (float)rawImage.Height / InputH;

        var raw = new List<YoloDetection>(16);
        for (int i = 0; i < NumDet; i++)
        {
            // Find best class score for this anchor
            float bestScore = 0f;
            int bestId = 0;
            for (int k = 0; k < NumClasses; k++)
            {
                float s = span[(4 + k) * NumDet + i];
                if (s > bestScore) { bestScore = s; bestId = k; }
            }
            if (bestScore < ConfThreshold) continue;

            float cx = span[0 * NumDet + i];
            float cy = span[1 * NumDet + i];
            float bw = span[2 * NumDet + i];
            float bh = span[3 * NumDet + i];

            float x1 = (cx - bw * 0.5f) * sx;
            float y1 = (cy - bh * 0.5f) * sy;
            float x2 = (cx + bw * 0.5f) * sx;
            float y2 = (cy + bh * 0.5f) * sy;

            raw.Add(new YoloDetection(x1, y1, x2, y2, bestScore, bestId));
        }

        // 5. Greedy NMS (class-agnostic — we just want geometry)
        return ApplyNms(raw, NmsIoU);
    }

    // ??? Helpers ???????????????????????????????????????????????????????

    private static void FillInputBuffer(Bitmap image, float[] buffer)
    {
        int w = image.Width, h = image.Height;
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
                    // BMP/GDI stores BGR — model expects RGB
                    buffer[planeR + dstRow + x] = pixels[px + 2] / 255f;
                    buffer[planeG + dstRow + x] = pixels[px + 1] / 255f;
                    buffer[planeB + dstRow + x] = pixels[px]     / 255f;
                }
            }
        }
        finally
        {
            image.UnlockBits(bmpData);
        }
    }

    private static List<YoloDetection> ApplyNms(List<YoloDetection> dets, float iouThr)
    {
        if (dets.Count <= 1) return dets;
        dets.Sort((a, b) => b.Confidence.CompareTo(a.Confidence));

        var keep = new List<YoloDetection>(dets.Count);
        var suppressed = new bool[dets.Count];

        for (int i = 0; i < dets.Count; i++)
        {
            if (suppressed[i]) continue;
            var a = dets[i];
            keep.Add(a);
            for (int j = i + 1; j < dets.Count; j++)
            {
                if (suppressed[j]) continue;
                if (Iou(a, dets[j]) > iouThr) suppressed[j] = true;
            }
        }
        return keep;
    }

    private static float Iou(in YoloDetection a, in YoloDetection b)
    {
        float ix1 = MathF.Max(a.X1, b.X1);
        float iy1 = MathF.Max(a.Y1, b.Y1);
        float ix2 = MathF.Min(a.X2, b.X2);
        float iy2 = MathF.Min(a.Y2, b.Y2);
        float iw = MathF.Max(0f, ix2 - ix1);
        float ih = MathF.Max(0f, iy2 - iy1);
        float inter = iw * ih;
        float union = a.Area + b.Area - inter;
        return union > 0 ? inter / union : 0f;
    }

    public void Dispose()
    {
        _session?.Dispose();
        _session = null;
    }
}
