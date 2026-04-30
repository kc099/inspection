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
using OpenCvSharp;

namespace RoboViz;

/// <summary>
/// Single YOLO11-seg detection (bbox in RAW image coordinates).
/// Kept for compatibility with earlier bbox-only callers.
/// </summary>
public readonly record struct YoloDetection(
    float X1, float Y1, float X2, float Y2, float Confidence, int ClassId)
{
    public float Width => X2 - X1;
    public float Height => Y2 - Y1;
    public float Area => Width * Height;
    public float CenterX => (X1 + X2) * 0.5f;
    public float CenterY => (Y1 + Y2) * 0.5f;
}

/// <summary>
/// Single YOLO11-seg detection with a reconstructed contour derived from the
/// segmentation mask. Used by CAM2 geometry.
/// </summary>
public readonly record struct YoloContourDetection(
    float X1, float Y1, float X2, float Y2, float Confidence, int ClassId, OpenCvSharp.Point[] Contour)
{
    public float Width => X2 - X1;
    public float Height => Y2 - Y1;
    public float Area => Width * Height;
    public float CenterX => (X1 + X2) * 0.5f;
    public float CenterY => (Y1 + Y2) * 0.5f;
    public double ContourArea => Contour.Length > 0 ? Cv2.ContourArea(Contour) : 0;
}

/// <summary>
/// CPU-only YOLO11n-seg ONNX inference for the CAM2 contour pipeline.
/// Uses the segmentation mask branch to reconstruct usable contours.
/// </summary>
public class YoloContourDetector : IDisposable
{
    private InferenceSession? _session;
    private string? _inputName;

    public const int InputW = 640;
    public const int InputH = 512;
    private const int ProtoW = 160;
    private const int ProtoH = 128;
    private const int NumDet = 6720;
    private const int NumClasses = 4;
    private const int NumMaskCoeffs = 32;

    public float ConfThreshold { get; set; } = 0.25f;
    public float NmsIoU { get; set; } = 0.45f;
    public float MaskThreshold { get; set; } = 0.50f;

    public bool IsLoaded => _session != null;

    private sealed class Candidate
    {
        public required YoloDetection Detection { get; init; }
        public required float[] Coeffs { get; init; }
        public required float ModelCx { get; init; }
        public required float ModelCy { get; init; }
        public required float ModelW { get; init; }
        public required float ModelH { get; init; }
    }

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
    /// Legacy bbox-only API kept for compatibility.
    /// </summary>
    public List<YoloDetection> Detect(Bitmap rawImage, out long inferenceMs) =>
        [.. DetectContours(rawImage, out inferenceMs).Select(d => new YoloDetection(d.X1, d.Y1, d.X2, d.Y2, d.Confidence, d.ClassId))];

    /// <summary>
    /// Detect contours on a raw camera image using the mask branch of the
    /// segmentation model. Contours are returned in RAW image coordinates.
    /// </summary>
    public List<YoloContourDetection> DetectContours(Bitmap rawImage, out long inferenceMs)
    {
        inferenceMs = 0;
        if (_session == null || _inputName == null)
            return [];

        using var resized = new Bitmap(InputW, InputH, PixelFormat.Format24bppRgb);
        using (var g = Graphics.FromImage(resized))
        {
            g.InterpolationMode = InterpolationMode.HighQualityBilinear;
            g.PixelOffsetMode = PixelOffsetMode.HighQuality;
            g.DrawImage(rawImage, 0, 0, InputW, InputH);
        }

        var input = new float[3 * InputH * InputW];
        FillInputBuffer(resized, input);

        var tensor = new DenseTensor<float>(input, [1, 3, InputH, InputW]);
        var inputs = new List<NamedOnnxValue>
        {
            NamedOnnxValue.CreateFromTensor(_inputName, tensor)
        };

        var sw = Stopwatch.StartNew();
        using var results = _session.Run(inputs);
        inferenceMs = sw.ElapsedMilliseconds;

        var resultList = results.ToList();
        if (resultList.Count < 2)
        {
            Debug.WriteLine("[YOLO-Contour] Expected detection + prototype outputs.");
            return [];
        }

        var det0 = resultList[0].AsTensor<float>();
        var protos = resultList[1].AsTensor<float>();
        if (det0 is not DenseTensor<float> detDense || protos is not DenseTensor<float> protoDense)
        {
            Debug.WriteLine("[YOLO-Contour] Unexpected tensor type.");
            return [];
        }

        var detSpan = detDense.Buffer.Span;
        var protoSpan = protoDense.Buffer.Span;

        float sx = (float)rawImage.Width / InputW;
        float sy = (float)rawImage.Height / InputH;

        var candidates = new List<Candidate>(16);
        for (int i = 0; i < NumDet; i++)
        {
            float bestScore = 0f;
            int bestId = 0;
            for (int k = 0; k < NumClasses; k++)
            {
                float s = detSpan[(4 + k) * NumDet + i];
                if (s > bestScore) { bestScore = s; bestId = k; }
            }
            if (bestScore < ConfThreshold) continue;

            float cx = detSpan[0 * NumDet + i];
            float cy = detSpan[1 * NumDet + i];
            float bw = detSpan[2 * NumDet + i];
            float bh = detSpan[3 * NumDet + i];

            float x1 = (cx - bw * 0.5f) * sx;
            float y1 = (cy - bh * 0.5f) * sy;
            float x2 = (cx + bw * 0.5f) * sx;
            float y2 = (cy + bh * 0.5f) * sy;

            var coeffs = new float[NumMaskCoeffs];
            for (int m = 0; m < NumMaskCoeffs; m++)
                coeffs[m] = detSpan[(4 + NumClasses + m) * NumDet + i];

            candidates.Add(new Candidate
            {
                Detection = new YoloDetection(x1, y1, x2, y2, bestScore, bestId),
                Coeffs = coeffs,
                ModelCx = cx,
                ModelCy = cy,
                ModelW = bw,
                ModelH = bh,
            });
        }

        var kept = ApplyNms(candidates, NmsIoU);
        var contourDetections = new List<YoloContourDetection>(kept.Count);
        foreach (var cand in kept)
        {
            var contour = BuildContourFromMask(cand, protoSpan, rawImage.Width, rawImage.Height);
            if (contour.Length == 0) continue;

            contourDetections.Add(new YoloContourDetection(
                cand.Detection.X1, cand.Detection.Y1, cand.Detection.X2, cand.Detection.Y2,
                cand.Detection.Confidence, cand.Detection.ClassId, contour));
        }

        return contourDetections;
    }

    private OpenCvSharp.Point[] BuildContourFromMask(Candidate cand, Span<float> protoSpan, int rawW, int rawH)
    {
        int protoPlane = ProtoW * ProtoH;
        using var mask = new Mat(ProtoH, ProtoW, MatType.CV_8UC1, Scalar.All(0));

        int px1 = Math.Clamp((int)MathF.Floor((cand.ModelCx - cand.ModelW * 0.5f) * ProtoW / InputW), 0, ProtoW - 1);
        int py1 = Math.Clamp((int)MathF.Floor((cand.ModelCy - cand.ModelH * 0.5f) * ProtoH / InputH), 0, ProtoH - 1);
        int px2 = Math.Clamp((int)MathF.Ceiling((cand.ModelCx + cand.ModelW * 0.5f) * ProtoW / InputW), 0, ProtoW);
        int py2 = Math.Clamp((int)MathF.Ceiling((cand.ModelCy + cand.ModelH * 0.5f) * ProtoH / InputH), 0, ProtoH);

        for (int y = py1; y < py2; y++)
        {
            int row = y * ProtoW;
            for (int x = px1; x < px2; x++)
            {
                int idx = row + x;
                float v = 0f;
                for (int k = 0; k < NumMaskCoeffs; k++)
                    v += cand.Coeffs[k] * protoSpan[k * protoPlane + idx];

                float sigmoid = 1f / (1f + MathF.Exp(-v));
                if (sigmoid >= MaskThreshold)
                    mask.Set(y, x, (byte)255);
            }
        }

        using var kernel = Cv2.GetStructuringElement(MorphShapes.Ellipse, new OpenCvSharp.Size(3, 3));
        Cv2.MorphologyEx(mask, mask, MorphTypes.Close, kernel, iterations: 1);

        Cv2.FindContours(mask, out OpenCvSharp.Point[][] contours, out _, RetrievalModes.External, ContourApproximationModes.ApproxSimple);
        if (contours.Length == 0) return [];

        var best = contours.OrderByDescending(c => Cv2.ContourArea(c)).FirstOrDefault();
        if (best == null || best.Length == 0) return [];

        float scaleX = (float)rawW / ProtoW;
        float scaleY = (float)rawH / ProtoH;
        var scaled = new OpenCvSharp.Point[best.Length];
        for (int i = 0; i < best.Length; i++)
        {
            int x = Math.Clamp((int)MathF.Round(best[i].X * scaleX), 0, rawW - 1);
            int y = Math.Clamp((int)MathF.Round(best[i].Y * scaleY), 0, rawH - 1);
            scaled[i] = new OpenCvSharp.Point(x, y);
        }
        return scaled;
    }

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
                    buffer[planeR + dstRow + x] = pixels[px + 2] / 255f;
                    buffer[planeG + dstRow + x] = pixels[px + 1] / 255f;
                    buffer[planeB + dstRow + x] = pixels[px] / 255f;
                }
            }
        }
        finally
        {
            image.UnlockBits(bmpData);
        }
    }

    private static List<Candidate> ApplyNms(List<Candidate> dets, float iouThr)
    {
        if (dets.Count <= 1) return dets;
        dets.Sort((a, b) => b.Detection.Confidence.CompareTo(a.Detection.Confidence));

        var keep = new List<Candidate>(dets.Count);
        var suppressed = new bool[dets.Count];
        for (int i = 0; i < dets.Count; i++)
        {
            if (suppressed[i]) continue;
            var a = dets[i];
            keep.Add(a);
            for (int j = i + 1; j < dets.Count; j++)
            {
                if (suppressed[j]) continue;
                if (Iou(a.Detection, dets[j].Detection) > iouThr) suppressed[j] = true;
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
