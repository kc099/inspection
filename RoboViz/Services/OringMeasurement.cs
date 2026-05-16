using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Runtime.InteropServices;
using OpenCvSharp;

namespace RoboViz;

/// <summary>
/// O-ring geometric measurement using OpenCvSharp.
/// Direct port of measure_oring() / bin_crop_720() from inspection_gui.py.
/// </summary>
public static class OringMeasurement
{
    public const int DefaultBgValue = 24;
    private const int DefaultThreshold =40;//lohith changed bg subtraction

    // Cam2 background image cache (loaded once, reused every call)
    private static Mat? _cam2BgGray;
    private static readonly object _cam2BgLock = new();

    // Cam2 YOLO contour detector (replaces the OpenCV adaptive-threshold pipeline).
    private static YoloContourDetector? _cam2Yolo;

    /// <summary>
    /// Wire up the YOLO contour detector used by <see cref="MeasureCam2"/>.
    /// Pass null to fall back to the legacy OpenCV pipeline.
    /// </summary>
    public static void SetCam2YoloDetector(YoloContourDetector? detector)
    {
        _cam2Yolo = detector;
    }

    static OringMeasurement()
    {
        // Limit OpenCV internal parallelism to avoid thread oversubscription
        // when multiple Inspect() calls run in parallel via Parallel.For
        Cv2.SetNumThreads(4);
    }

    /// <summary>
    /// Measure an o-ring from a raw image (any resolution).
    /// Returns null if contours cannot be found.
    /// </summary>
    public static GeometricResult? Measure(Bitmap image, int bgValue = DefaultBgValue, int threshold = DefaultThreshold)
    {
        using var src = BitmapToMat(image);
        using var gray = new Mat();
        Cv2.CvtColor(src, gray, ColorConversionCodes.BGR2GRAY);

        using var mask = BuildMask(gray, bgValue, threshold);

        // find_contours with RETR_CCOMP — gives 2-level hierarchy
        Cv2.FindContours(mask, out OpenCvSharp.Point[][] contours, out HierarchyIndex[] hierarchy,
            RetrievalModes.CComp, ContourApproximationModes.ApproxNone);

        if (contours.Length == 0 || hierarchy.Length == 0)
        {
            Debug.WriteLine($"[Measure] FAIL: no contours found. Image={image.Width}x{image.Height}, bgValue={bgValue}, threshold={threshold}, fgPixels={Cv2.CountNonZero(mask)}");
            return null;
        }

        // Outer = largest contour by area
        int outerIdx = -1;
        double maxArea = 0;
        for (int i = 0; i < contours.Length; i++)
        {
            double a = Cv2.ContourArea(contours[i]);
            if (a > maxArea) { maxArea = a; outerIdx = i; }
        }
        if (outerIdx < 0) return null;
        var outer = contours[outerIdx];

        // Inner = largest child of outer
        OpenCvSharp.Point[]? inner = null;
        double bestInnerArea = 0;
        for (int i = 0; i < contours.Length; i++)
        {
            if (hierarchy[i].Parent == outerIdx)
            {
                double a = Cv2.ContourArea(contours[i]);
                if (a > bestInnerArea) { bestInnerArea = a; inner = contours[i]; }
            }
        }
        if (inner == null)
        {
            Debug.WriteLine($"[Measure] FAIL: no inner contour (hole) found. contours={contours.Length}, outerArea={maxArea:F0}");
            return null;
        }

        // Fit circles via minimum enclosing circle
        Cv2.MinEnclosingCircle(outer, out var outerCenter, out float outerRadius);
        Cv2.MinEnclosingCircle(inner, out var innerCenter, out float innerRadius);
        double ox = outerCenter.X, oy = outerCenter.Y, orad = outerRadius;
        double ix = innerCenter.X, iy = innerCenter.Y, irad = innerRadius;
        double cdist = Math.Sqrt((ox - ix) * (ox - ix) + (oy - iy) * (oy - iy));
        double mrad = (orad + irad) / 2.0;

        // Circularity
        double oArea = Cv2.ContourArea(outer);
        double oPeri = Cv2.ArcLength(outer, true);
        double circOuter = oPeri > 0 ? (4.0 * Math.PI * oArea / (oPeri * oPeri)) : 0;

        double iArea = Cv2.ContourArea(inner);
        double iPeri = Cv2.ArcLength(inner, true);
        double circInner = iPeri > 0 ? (4.0 * Math.PI * iArea / (iPeri * iPeri)) : 0;

        return new GeometricResult
        {
            OuterRadius = orad,
            InnerRadius = irad,
            CenterDist = cdist,
            EccentricityPct = mrad > 0 ? (cdist / mrad * 100.0) : 0.0,
            CircularityOuter = circOuter,
            CircularityInner = circInner,
            OuterCenter = new PointF((float)ox, (float)oy),
            InnerCenter = new PointF((float)ix, (float)iy),
            OuterContour = outer,
            InnerContour = inner,
        };
    }

    public static void LoadCam2Background(string bmpPath)
    {
        lock (_cam2BgLock)
        {
            _cam2BgGray?.Dispose();
            var bgBgr = Cv2.ImRead(bmpPath, ImreadModes.Grayscale);
            if (bgBgr.Empty())
                throw new FileNotFoundException($"Cam2 background image not found: {bmpPath}");
            _cam2BgGray = bgBgr;
            Debug.WriteLine($"[Cam2] Background loaded: {bmpPath} ({bgBgr.Width}x{bgBgr.Height})");
        }
    }

    /// <summary>
    /// Measure an o-ring from a cam2 raw image using the trained YOLO11n-seg
    /// contour model. The segmentation masks are reconstructed, contours are
    /// extracted, and geometry is computed from min-enclosing circles and
    /// contour circularity.
    /// Returns null if fewer than two usable contours are detected.
    /// </summary>
    public static GeometricResult? MeasureCam2(Bitmap image)
    {
        var detector = _cam2Yolo;
        if (detector == null || !detector.IsLoaded)
        {
            Debug.WriteLine("[MeasureCam2] FAIL: YOLO contour detector not loaded. " +
                "Call OringMeasurement.SetCam2YoloDetector(...) at startup.");
            return null;
        }

        var dets = detector.DetectContours(image, out long inferMs);
        Debug.WriteLine($"[MeasureCam2] YOLO inference: {inferMs}ms, contourDetections={dets.Count}");

        if (dets.Count < 2)
        {
            Debug.WriteLine($"[MeasureCam2] FAIL: need >=2 contour detections, got {dets.Count}.");
            return null;
        }

        dets.Sort((a, b) => b.ContourArea.CompareTo(a.ContourArea));
        var outerD = dets[0];
        var innerD = dets[1];

        Cv2.MinEnclosingCircle(outerD.Contour, out var outerCenter, out float outerRadius);
        Cv2.MinEnclosingCircle(innerD.Contour, out var innerCenter, out float innerRadius);

        double cdist = Math.Sqrt(
            (outerCenter.X - innerCenter.X) * (outerCenter.X - innerCenter.X) +
            (outerCenter.Y - innerCenter.Y) * (outerCenter.Y - innerCenter.Y));
        double mrad = (outerRadius + innerRadius) / 2.0;

        double circOuter = ComputeCircularity(outerD.Contour);
        double circInner = ComputeCircularity(innerD.Contour);

        return new GeometricResult
        {
            OuterRadius = outerRadius,
            InnerRadius = innerRadius,
            CenterDist = cdist,
            EccentricityPct = mrad > 0 ? (cdist / mrad * 100.0) : 0.0,
            CircularityOuter = circOuter,
            CircularityInner = circInner,
            OuterCenter = new PointF(outerCenter.X, outerCenter.Y),
            InnerCenter = new PointF(innerCenter.X, innerCenter.Y),
            OuterContour = outerD.Contour,
            InnerContour = innerD.Contour,
        };
    }

    /// <summary>
    /// Draw contour-based overlay for cam2 (actual contour outlines + centers + connection line).
    /// </summary>
    public static Bitmap DrawContourOverlayCam2(Bitmap image, GeometricResult result)
    {
        using var vis = BitmapToMat(image);

        // Draw actual contour outlines
        if (result.OuterContour != null)
            Cv2.DrawContours(vis, [result.OuterContour], -1, new Scalar(0, 255, 0), 2, LineTypes.AntiAlias);
        if (result.InnerContour != null)
            Cv2.DrawContours(vis, [result.InnerContour], -1, new Scalar(0, 0, 255), 2, LineTypes.AntiAlias);

        // Draw enclosing circles as the actual measured radii for CAM2.
        var oc = new OpenCvSharp.Point((int)result.OuterCenter.X, (int)result.OuterCenter.Y);
        var ic = new OpenCvSharp.Point((int)result.InnerCenter.X, (int)result.InnerCenter.Y);
        Cv2.Circle(vis, oc, (int)Math.Round(result.OuterRadius), new Scalar(0, 255, 0), 2, LineTypes.AntiAlias);
        Cv2.Circle(vis, ic, (int)Math.Round(result.InnerRadius), new Scalar(0, 0, 255), 2, LineTypes.AntiAlias);

        // Center dots + connection line (same as cam1 style)
        Cv2.Circle(vis, oc, 8, new Scalar(0, 255, 0), -1);
        Cv2.Circle(vis, ic, 8, new Scalar(0, 0, 255), -1);
        Cv2.Line(vis, oc, ic, new Scalar(0, 255, 255), 2);

        return MatToBitmap(vis);
    }

    #region Helpers

    private static double ComputeCircularity(OpenCvSharp.Point[] contour)
    {
        double area = Cv2.ContourArea(contour);
        double peri = Cv2.ArcLength(contour, true);
        return peri > 0 ? (4.0 * Math.PI * area / (peri * peri)) : 0.0;
    }

    /// <summary>
    /// Build binary mask — exact port of build_mask() from inspection_gui.py.
    /// </summary>
    private static Mat BuildMask(Mat gray, int bgValue, int threshold)
    {
        using var bgMat = new Mat(gray.Size(), MatType.CV_8UC1, new Scalar(bgValue));
        using var diff = new Mat();
        Cv2.Absdiff(gray, bgMat, diff);

        var binary = new Mat();
        Cv2.Threshold(diff, binary, threshold, 255, ThresholdTypes.Binary);

        // If more than 75% is foreground, invert
        if (Cv2.Mean(binary).Val0 / 255.0 > 0.75)
            Cv2.BitwiseNot(binary, binary);

        using var kernel = Cv2.GetStructuringElement(MorphShapes.Ellipse, new OpenCvSharp.Size(7, 7));
        Cv2.MorphologyEx(binary, binary, MorphTypes.Close, kernel, iterations: 2);
        Cv2.MorphologyEx(binary, binary, MorphTypes.Open, kernel, iterations: 1);

        // Keep largest connected component
        using var labels = new Mat();
        using var stats = new Mat();
        using var centroids = new Mat();
        int n = Cv2.ConnectedComponentsWithStats(binary, labels, stats, centroids);

        if (n > 1)
        {
            int bestLabel = 1;
            int bestArea = 0;
            for (int i = 1; i < n; i++)
            {
                int area = stats.At<int>(i, 4); // CC_STAT_AREA = 4
                if (area > bestArea) { bestArea = area; bestLabel = i; }
            }
            // binary = (labels == bestLabel) * 255
            using var eqMask = new Mat();
            Cv2.Compare(labels, bestLabel, eqMask, CmpType.EQ);
            eqMask.ConvertTo(binary, MatType.CV_8UC1, 255);
        }

        return binary;
    }

    /// <summary>
    /// Build CAM 2 mask using background-subtracted ratio normalization.
    /// </summary>
    private static Mat? BuildMaskCam2(Mat gray)
    {
        Mat? bg;
        lock (_cam2BgLock)
        {
            bg = _cam2BgGray?.Clone();
        }
        if (bg == null)
        {
            Debug.WriteLine("[BuildMaskCam2] No background image loaded. Call LoadCam2Background() first.");
            return null;
        }

        try
        {
            if (bg.Size() != gray.Size())
                Cv2.Resize(bg, bg, gray.Size(), 0, 0, InterpolationFlags.Area);

            using var gray32 = new Mat();
            using var bg32 = new Mat();
            gray.ConvertTo(gray32, MatType.CV_32F);
            bg.ConvertTo(bg32, MatType.CV_32F);

            using var denom = new Mat();
            Cv2.Max(bg32, 1.0, denom); // prevent divide-by-zero

            using var ratio = new Mat();
            Cv2.Divide(gray32, denom, ratio);

            using var ratio8 = new Mat();
            ratio.ConvertTo(ratio8, MatType.CV_8U, 255.0);

            using var blurred = new Mat();
            Cv2.GaussianBlur(ratio8, blurred, new OpenCvSharp.Size(21, 21), 0);

            var binary = new Mat();
            Cv2.AdaptiveThreshold(blurred, binary, 255,
                AdaptiveThresholdTypes.GaussianC, ThresholdTypes.BinaryInv, 151, 5);

            using var kClose = Cv2.GetStructuringElement(MorphShapes.Ellipse, new OpenCvSharp.Size(35, 35));
            using var kOpen = Cv2.GetStructuringElement(MorphShapes.Ellipse, new OpenCvSharp.Size(15, 15));
            Cv2.MorphologyEx(binary, binary, MorphTypes.Close, kClose, iterations: 3);
            Cv2.MorphologyEx(binary, binary, MorphTypes.Open, kOpen, iterations: 1);

            return binary;
        }
        finally
        {
            bg.Dispose();
        }
    }

    private static (double X, double Y, double R) FitCircleLsq(OpenCvSharp.Point[] contour)
    {
        if (contour.Length < 3)
        {
            var c = contour.Length > 0 ? contour[0] : default;
            return (c.X, c.Y, 0);
        }

        // Kasa algebraic least-squares circle fit.
        double sumX = 0, sumY = 0, sumXX = 0, sumYY = 0, sumXY = 0;
        double sumXXX = 0, sumYYY = 0, sumXYY = 0, sumXXY = 0;
        int n = contour.Length;

        foreach (var p in contour)
        {
            double x = p.X;
            double y = p.Y;
            double xx = x * x;
            double yy = y * y;

            sumX += x;
            sumY += y;
            sumXX += xx;
            sumYY += yy;
            sumXY += x * y;
            sumXXX += xx * x;
            sumYYY += yy * y;
            sumXYY += x * yy;
            sumXXY += xx * y;
        }

        double c1 = n * sumXX - sumX * sumX;
        double c2 = n * sumXY - sumX * sumY;
        double c3 = n * sumYY - sumY * sumY;
        double d1 = 0.5 * (n * (sumXXX + sumXYY) - sumX * (sumXX + sumYY));
        double d2 = 0.5 * (n * (sumYYY + sumXXY) - sumY * (sumXX + sumYY));

        double det = c1 * c3 - c2 * c2;
        if (Math.Abs(det) < 1e-12)
        {
            // Degenerate — fallback to min enclosing circle
            Cv2.MinEnclosingCircle(contour, out var cc, out float rr);
            return (cc.X, cc.Y, rr);
        }

        double a = (d1 * c3 - d2 * c2) / det;
        double b = (c1 * d2 - c2 * d1) / det;

        double r = 0;
        foreach (var p in contour)
        {
            double dx = p.X - a;
            double dy = p.Y - b;
            r += Math.Sqrt(dx * dx + dy * dy);
        }
        r /= n;

        return (a, b, r);
    }

    public static Bitmap DrawGeometricOverlay(Bitmap image, GeometricResult result)
    {
        using var vis = BitmapToMat(image);

        var outerCenter = new OpenCvSharp.Point((int)result.OuterCenter.X, (int)result.OuterCenter.Y);
        var innerCenter = new OpenCvSharp.Point((int)result.InnerCenter.X, (int)result.InnerCenter.Y);

        Cv2.Circle(vis, outerCenter, (int)Math.Round(result.OuterRadius), new Scalar(0, 255, 0), 2);
        Cv2.Circle(vis, innerCenter, (int)Math.Round(result.InnerRadius), new Scalar(0, 0, 255), 2);
        Cv2.Circle(vis, outerCenter, 8, new Scalar(0, 255, 0), -1);
        Cv2.Circle(vis, innerCenter, 8, new Scalar(0, 0, 255), -1);
        Cv2.Line(vis, outerCenter, innerCenter, new Scalar(0, 255, 255), 2);

        return MatToBitmap(vis);
    }

    public static Bitmap DrawContourOverlayCam2Legacy(Bitmap image, GeometricResult result)
    {
        using var vis = BitmapToMat(image);

        if (result.OuterContour != null)
            Cv2.DrawContours(vis, [result.OuterContour], -1, new Scalar(0, 255, 0), 2, LineTypes.AntiAlias);
        if (result.InnerContour != null)
            Cv2.DrawContours(vis, [result.InnerContour], -1, new Scalar(0, 0, 255), 2, LineTypes.AntiAlias);

        var oc = new OpenCvSharp.Point((int)result.OuterCenter.X, (int)result.OuterCenter.Y);
        var ic = new OpenCvSharp.Point((int)result.InnerCenter.X, (int)result.InnerCenter.Y);
        Cv2.Circle(vis, oc, 8, new Scalar(0, 255, 0), -1);
        Cv2.Circle(vis, ic, 8, new Scalar(0, 0, 255), -1);
        Cv2.Line(vis, oc, ic, new Scalar(0, 255, 255), 2);

        return MatToBitmap(vis);
    }

    public static Bitmap MatToBitmap(Mat mat)
    {
        using var bgr = mat.Channels() switch
        {
            1 => mat.CvtColor(ColorConversionCodes.GRAY2BGR),
            3 => mat.Clone(),
            4 => mat.CvtColor(ColorConversionCodes.BGRA2BGR),
            _ => throw new NotSupportedException($"Unsupported channel count: {mat.Channels()}")
        };

        var bmp = new Bitmap(bgr.Width, bgr.Height, PixelFormat.Format24bppRgb);
        var rect = new Rectangle(0, 0, bmp.Width, bmp.Height);
        var bmpData = bmp.LockBits(rect, ImageLockMode.WriteOnly, PixelFormat.Format24bppRgb);
        try
        {
            int srcStride = (int)bgr.Step();
            int dstStride = bmpData.Stride;
            int rowBytes = bgr.Width * 3;
            byte[] src = new byte[srcStride * bgr.Height];
            Marshal.Copy(bgr.Data, src, 0, src.Length);

            byte[] dst = new byte[dstStride * bgr.Height];
            for (int y = 0; y < bgr.Height; y++)
                Buffer.BlockCopy(src, y * srcStride, dst, y * dstStride, rowBytes);

            Marshal.Copy(dst, 0, bmpData.Scan0, dst.Length);
        }
        finally
        {
            bmp.UnlockBits(bmpData);
        }
        return bmp;
    }

    public static Mat BitmapToMat(Bitmap bmp)
    {
        Bitmap source = bmp;
        Bitmap? converted = null;
        if (bmp.PixelFormat != PixelFormat.Format24bppRgb)
        {
            converted = new Bitmap(bmp.Width, bmp.Height, PixelFormat.Format24bppRgb);
            using var g = Graphics.FromImage(converted);
            g.DrawImage(bmp, 0, 0, bmp.Width, bmp.Height);
            source = converted;
        }

        var mat = new Mat(source.Height, source.Width, MatType.CV_8UC3);
        var rect = new Rectangle(0, 0, source.Width, source.Height);
        var bmpData = source.LockBits(rect, ImageLockMode.ReadOnly, PixelFormat.Format24bppRgb);
        try
        {
            int srcStride = bmpData.Stride;
            int dstStride = (int)mat.Step();
            int rowBytes = source.Width * 3;
            byte[] src = new byte[srcStride * source.Height];
            Marshal.Copy(bmpData.Scan0, src, 0, src.Length);

            byte[] dst = new byte[dstStride * source.Height];
            for (int y = 0; y < source.Height; y++)
                Buffer.BlockCopy(src, y * srcStride, dst, y * dstStride, rowBytes);

            Marshal.Copy(dst, 0, mat.Data, dst.Length);
        }
        finally
        {
            source.UnlockBits(bmpData);
            converted?.Dispose();
        }

        return mat;
    }

    #endregion
}
