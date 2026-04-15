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
    private const int DefaultThreshold = 30;

    // Cam2 background image cache (loaded once, reused every call)
    private static Mat? _cam2BgGray;
    private static readonly object _cam2BgLock = new();

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

        // Inner = largest child of outer (hierarchy[i].Parent == outerIdx)
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
            Debug.WriteLine($"[Measure] FAIL: no inner contour (hole) found. Image={image.Width}x{image.Height}, contours={contours.Length}, outerArea={maxArea:F0}");
            return null;
        }

        // Fit circles via least squares
        var (ox, oy, orad) = FitCircleLsq(outer);
        var (ix, iy, irad) = FitCircleLsq(inner);
        double cdist = Math.Sqrt((ox - ix) * (ox - ix) + (oy - iy) * (oy - iy));
        double mrad = (orad + irad) / 2.0;

        // Circularity = 4?·area / perimeter²
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
        };
    }

    /// <summary>
    /// 2×2 binning + foreground crop + resize/pad to 720×720.
    /// </summary>
    public static Bitmap BinCrop720(Bitmap image, int bgValue = DefaultBgValue, int threshold = DefaultThreshold)
    {
        using var src = BitmapToMat(image);

        // 2×2 binning
        using var binned = new Mat();
        Cv2.Resize(src, binned, new OpenCvSharp.Size(src.Width / 2, src.Height / 2),
            interpolation: InterpolationFlags.Linear);

        // Foreground bounding box
        using var binnedGray = new Mat();
        Cv2.CvtColor(binned, binnedGray, ColorConversionCodes.BGR2GRAY);
        using var diff = new Mat();
        Cv2.Absdiff(binnedGray, new Scalar(bgValue), diff);
        using var fgMask = new Mat();
        Cv2.Threshold(diff, fgMask, threshold, 255, ThresholdTypes.Binary);

        var bbox = Cv2.BoundingRect(fgMask);
        int pad = 10;
        int x1 = Math.Max(0, bbox.X - pad);
        int y1 = Math.Max(0, bbox.Y - pad);
        int x2 = Math.Min(binned.Width, bbox.X + bbox.Width + pad);
        int y2 = Math.Min(binned.Height, bbox.Y + bbox.Height + pad);
        using var cropped = new Mat(binned, new Rect(x1, y1, x2 - x1, y2 - y1));

        // Resize to fit 720×720 maintaining aspect ratio, pad with bg color
        double scale = Math.Min(720.0 / cropped.Width, 720.0 / cropped.Height);
        int newW = (int)(cropped.Width * scale);
        int newH = (int)(cropped.Height * scale);

        using var resized = new Mat();
        Cv2.Resize(cropped, resized, new OpenCvSharp.Size(newW, newH),
            interpolation: InterpolationFlags.Linear);

        using var canvas = new Mat(720, 720, MatType.CV_8UC3, new Scalar(bgValue, bgValue, bgValue));
        int offsetX = (720 - newW) / 2;
        int offsetY = (720 - newH) / 2;
        var roi = new Rect(offsetX, offsetY, newW, newH);
        resized.CopyTo(new Mat(canvas, roi));

        return MatToBitmap(canvas);
    }

    /// <summary>
    /// Draw geometric overlay (fitted circles, centers, connection line).
    /// </summary>
    public static Bitmap DrawGeometricOverlay(Bitmap image, GeometricResult result)
    {
        using var vis = BitmapToMat(image);

        var oc = new OpenCvSharp.Point((int)result.OuterCenter.X, (int)result.OuterCenter.Y);
        var ic = new OpenCvSharp.Point((int)result.InnerCenter.X, (int)result.InnerCenter.Y);

        Cv2.Circle(vis, oc, (int)result.OuterRadius, new Scalar(0, 255, 0), 2, LineTypes.AntiAlias);
        Cv2.Circle(vis, ic, (int)result.InnerRadius, new Scalar(0, 0, 255), 2, LineTypes.AntiAlias);

        Cv2.Circle(vis, oc, 8, new Scalar(0, 255, 0), -1);
        Cv2.Circle(vis, ic, 8, new Scalar(0, 0, 255), -1);

        Cv2.Line(vis, oc, ic, new Scalar(0, 255, 255), 2);

        return MatToBitmap(vis);
    }

    // ?????????????????????????????????????????????????????????????
    //  Cam2 — washer_inspector.py methodology
    // ?????????????????????????????????????????????????????????????

    /// <summary>
    /// Load and cache the cam2 background image used for ratio normalization.
    /// Call once at startup (e.g. from InspectionService constructor).
    /// </summary>
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
    /// Measure an o-ring from a cam2 raw image using the washer_inspector.py
    /// ratio-normalization + adaptive-threshold pipeline.
    /// Returns null if contours cannot be found.
    /// The returned GeometricResult includes OuterContour / InnerContour for overlay.
    /// </summary>
    public static GeometricResult? MeasureCam2(Bitmap image)
    {
        using var src = BitmapToMat(image);
        using var gray = new Mat();
        Cv2.CvtColor(src, gray, ColorConversionCodes.BGR2GRAY);

        using var mask = BuildMaskCam2(gray);
        if (mask == null)
        {
            Debug.WriteLine($"[MeasureCam2] FAIL: BuildMaskCam2 returned null (no background loaded?).");
            return null;
        }

        // FindContours with RETR_CCOMP — same as cam1, gives outer + inner hierarchy
        Cv2.FindContours(mask, out OpenCvSharp.Point[][] contours, out HierarchyIndex[] hierarchy,
            RetrievalModes.CComp, ContourApproximationModes.ApproxNone);

        if (contours.Length == 0 || hierarchy.Length == 0)
        {
            Debug.WriteLine($"[MeasureCam2] FAIL: no contours found. Image={image.Width}x{image.Height}, fgPixels={Cv2.CountNonZero(mask)}");
            return null;
        }

        // Outer = largest contour by area (identical to cam1)
        int outerIdx = -1;
        double maxArea = 0;
        for (int i = 0; i < contours.Length; i++)
        {
            double a = Cv2.ContourArea(contours[i]);
            if (a > maxArea) { maxArea = a; outerIdx = i; }
        }
        if (outerIdx < 0) return null;
        var outer = contours[outerIdx];

        // Inner = largest child of outer (identical to cam1)
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
            Debug.WriteLine($"[MeasureCam2] FAIL: no inner contour (hole) found. contours={contours.Length}, outerArea={maxArea:F0}");
            return null;
        }

        // Fit circles via least squares (identical to cam1)
        var (ox, oy, orad) = FitCircleLsq(outer);
        var (ix, iy, irad) = FitCircleLsq(inner);
        double cdist = Math.Sqrt((ox - ix) * (ox - ix) + (oy - iy) * (oy - iy));
        double mrad = (orad + irad) / 2.0;

        // Circularity (identical to cam1)
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

        // Center dots + connection line (same as cam1 style)
        var oc = new OpenCvSharp.Point((int)result.OuterCenter.X, (int)result.OuterCenter.Y);
        var ic = new OpenCvSharp.Point((int)result.InnerCenter.X, (int)result.InnerCenter.Y);

        Cv2.Circle(vis, oc, 8, new Scalar(0, 255, 0), -1);
        Cv2.Circle(vis, ic, 8, new Scalar(0, 0, 255), -1);
        Cv2.Line(vis, oc, ic, new Scalar(0, 255, 255), 2);

        return MatToBitmap(vis);
    }

    #region Helpers

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
            using var labelScalar = new Mat(labels.Size(), labels.Type(), new Scalar(bestLabel));
            Cv2.Compare(labels, labelScalar, binary, CmpType.EQ);
        }

        return binary;
    }

    /// <summary>
    /// Build binary mask for cam2 — port of washer_inspector.py process_image() steps 1-6a.
    /// Uses ratio normalization against a background image + adaptive threshold.
    /// Returns a ring-shaped mask (central hole preserved) or null if no background loaded.
    /// </summary>
    private static Mat? BuildMaskCam2(Mat gray)
    {
        Mat? bg;
        lock (_cam2BgLock) { bg = _cam2BgGray; }
        if (bg == null)
        {
            Debug.WriteLine("[BuildMaskCam2] No background image loaded. Call LoadCam2Background() first.");
            return null;
        }

        // Resize background to match image if dimensions differ
        Mat bgResized;
        if (bg.Width != gray.Width || bg.Height != gray.Height)
        {
            bgResized = new Mat();
            Cv2.Resize(bg, bgResized, new OpenCvSharp.Size(gray.Width, gray.Height), interpolation: InterpolationFlags.Linear);
        }
        else
        {
            bgResized = bg;
        }

        try
        {
            // Step 1: Ratio normalization — bg pixels ? ~128, component deviates
            using var bgF = new Mat();
            bgResized.ConvertTo(bgF, MatType.CV_32F);
            Cv2.Max(bgF, new Scalar(5.0), bgF);

            using var imgF = new Mat();
            gray.ConvertTo(imgF, MatType.CV_32F);

            using var normalized32 = new Mat();
            Cv2.Divide(imgF, bgF, normalized32);
            Cv2.Multiply(normalized32, new Scalar(128.0), normalized32);
            Cv2.Min(normalized32, new Scalar(255), normalized32);
            Cv2.Max(normalized32, new Scalar(0), normalized32);

            using var normalized = new Mat();
            normalized32.ConvertTo(normalized, MatType.CV_8U);

            // Step 2: Deviation map |normalized - 128| + GaussianBlur
            using var mid = new Mat(normalized.Size(), MatType.CV_8U, new Scalar(128));
            using var dev = new Mat();
            Cv2.Absdiff(normalized, mid, dev);

            using var devBlur = new Mat();
            Cv2.GaussianBlur(dev, devBlur, new OpenCvSharp.Size(21, 21), 4);

            // Step 3: Adaptive threshold
            using var adaptive = new Mat();
            Cv2.AdaptiveThreshold(devBlur, adaptive, 255,
                AdaptiveThresholdTypes.GaussianC,
                ThresholdTypes.Binary,
                blockSize: 151, c: -5);

            // Step 4: Morphological cleanup
            using var kClose = Cv2.GetStructuringElement(MorphShapes.Ellipse, new OpenCvSharp.Size(35, 35));
            using var kOpen = Cv2.GetStructuringElement(MorphShapes.Ellipse, new OpenCvSharp.Size(15, 15));
            var mask = new Mat();
            Cv2.MorphologyEx(adaptive, mask, MorphTypes.Close, kClose, iterations: 3);
            Cv2.MorphologyEx(mask, mask, MorphTypes.Open, kOpen, iterations: 1);

            // Step 5: Pick the most circular connected component (score = circularity × area)
            using var labels = new Mat();
            using var stats = new Mat();
            using var centroids = new Mat();
            int nLabels = Cv2.ConnectedComponentsWithStats(mask, labels, stats, centroids, PixelConnectivity.Connectivity8);

            int bestId = -1;
            double bestScore = -1.0;
            for (int lbl = 1; lbl < nLabels; lbl++)
            {
                int area = stats.At<int>(lbl, 4);
                if (area < 500) continue;

                using var blob = new Mat();
                using var lblScalar = new Mat(labels.Size(), labels.Type(), new Scalar(lbl));
                Cv2.Compare(labels, lblScalar, blob, CmpType.EQ);

                Cv2.FindContours(blob, out OpenCvSharp.Point[][] cnts, out _,
                    RetrievalModes.External, ContourApproximationModes.ApproxNone);
                if (cnts.Length == 0) continue;

                double perim = Cv2.ArcLength(cnts[0], true);
                if (perim == 0) continue;

                double circ = 4.0 * Math.PI * area / (perim * perim);
                double score = circ * area;
                if (score > bestScore) { bestScore = score; bestId = lbl; }
            }

            Mat compMask;
            if (bestId >= 1)
            {
                compMask = new Mat();
                using var bestScalar = new Mat(labels.Size(), labels.Type(), new Scalar(bestId));
                Cv2.Compare(labels, bestScalar, compMask, CmpType.EQ);
            }
            else if (nLabels > 1)
            {
                // Fallback: largest component
                int fallbackLabel = 1;
                int fallbackArea = 0;
                for (int i = 1; i < nLabels; i++)
                {
                    int a = stats.At<int>(i, 4);
                    if (a > fallbackArea) { fallbackArea = a; fallbackLabel = i; }
                }
                compMask = new Mat();
                using var fbScalar = new Mat(labels.Size(), labels.Type(), new Scalar(fallbackLabel));
                Cv2.Compare(labels, fbScalar, compMask, CmpType.EQ);
            }
            else
            {
                compMask = mask.Clone();
            }

            // Step 6a: Fill small internal holes (<20% of component area), keep central hole
            int compArea = Cv2.CountNonZero(compMask);
            using var flooded = compMask.Clone();
            Cv2.FloodFill(flooded, new OpenCvSharp.Point(0, 0), new Scalar(255));
            using var allHoles = new Mat();
            Cv2.BitwiseNot(flooded, allHoles);

            using var hLabels = new Mat();
            using var hStats = new Mat();
            using var hCentroids = new Mat();
            int nh = Cv2.ConnectedComponentsWithStats(allHoles, hLabels, hStats, hCentroids, PixelConnectivity.Connectivity8);

            for (int hlbl = 1; hlbl < nh; hlbl++)
            {
                int holeArea = hStats.At<int>(hlbl, 4);
                if (holeArea < compArea * 0.20)
                {
                    // Small hole ? fill it in the component mask
                    using var holeMask = new Mat();
                    using var hScalar = new Mat(hLabels.Size(), hLabels.Type(), new Scalar(hlbl));
                    Cv2.Compare(hLabels, hScalar, holeMask, CmpType.EQ);
                    compMask.SetTo(new Scalar(255), holeMask);
                }
            }

            mask.Dispose();
            return compMask;
        }
        finally
        {
            if (!ReferenceEquals(bgResized, bg))
                bgResized.Dispose();
        }
    }

    /// <summary>
    /// Least-squares circle fit — port of fit_circle_lsq() from inspection_gui.py.
    /// </summary>
    private static (double cx, double cy, double radius) FitCircleLsq(OpenCvSharp.Point[] contour)
    {
        int n = contour.Length;
        double s_x = 0, s_y = 0, s_x2 = 0, s_y2 = 0, s_xy = 0;
        double s_x3 = 0, s_y3 = 0, s_x2y = 0, s_xy2 = 0;

        for (int i = 0; i < n; i++)
        {
            double x = contour[i].X, y = contour[i].Y;
            s_x += x; s_y += y;
            s_x2 += x * x; s_y2 += y * y; s_xy += x * y;
            s_x3 += x * x * x; s_y3 += y * y * y;
            s_x2y += x * x * y; s_xy2 += x * y * y;
        }

        double[,] A =
        {
            { 4 * s_x2, 4 * s_xy, 2 * s_x },
            { 4 * s_xy, 4 * s_y2, 2 * s_y },
            { 2 * s_x,  2 * s_y,  n        },
        };
        double[] b =
        [
            2 * (s_x3 + s_xy2),
            2 * (s_x2y + s_y3),
            s_x2 + s_y2,
        ];

        double[] result = SolveLinear3(A, b);
        double cx = result[0], cy = result[1], c = result[2];
        double radius = Math.Sqrt(Math.Max(c + cx * cx + cy * cy, 0.0));
        return (cx, cy, radius);
    }

    private static double[] SolveLinear3(double[,] A, double[] b)
    {
        double[,] aug = new double[3, 4];
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++) aug[i, j] = A[i, j];
            aug[i, 3] = b[i];
        }

        for (int col = 0; col < 3; col++)
        {
            int maxRow = col;
            for (int row = col + 1; row < 3; row++)
                if (Math.Abs(aug[row, col]) > Math.Abs(aug[maxRow, col]))
                    maxRow = row;

            if (maxRow != col)
                for (int j = 0; j < 4; j++)
                    (aug[col, j], aug[maxRow, j]) = (aug[maxRow, j], aug[col, j]);

            if (Math.Abs(aug[col, col]) < 1e-12) continue;

            for (int row = col + 1; row < 3; row++)
            {
                double factor = aug[row, col] / aug[col, col];
                for (int j = col; j < 4; j++)
                    aug[row, j] -= factor * aug[col, j];
            }
        }

        double[] x = new double[3];
        for (int i = 2; i >= 0; i--)
        {
            double sum = aug[i, 3];
            for (int j = i + 1; j < 3; j++)
                sum -= aug[i, j] * x[j];
            x[i] = Math.Abs(aug[i, i]) > 1e-12 ? sum / aug[i, i] : 0;
        }
        return x;
    }

    /// <summary>
    /// Convert System.Drawing.Bitmap to OpenCvSharp.Mat (BGR, 8UC3).
    /// </summary>
    private static Mat BitmapToMat(Bitmap bmp)
    {
        int w = bmp.Width, h = bmp.Height;
        var rect = new Rectangle(0, 0, w, h);
        var data = bmp.LockBits(rect, ImageLockMode.ReadOnly, PixelFormat.Format24bppRgb);
        try
        {
            var mat = new Mat(h, w, MatType.CV_8UC3);
            int srcStride = data.Stride;
            int dstStride = (int)mat.Step();
            int rowBytes = w * 3;

            for (int y = 0; y < h; y++)
            {
                IntPtr srcRow = data.Scan0 + y * srcStride;
                IntPtr dstRow = mat.Data + y * dstStride;
                byte[] row = new byte[rowBytes];
                Marshal.Copy(srcRow, row, 0, rowBytes);
                Marshal.Copy(row, 0, dstRow, rowBytes);
            }

            return mat;
        }
        finally
        {
            bmp.UnlockBits(data);
        }
    }

    /// <summary>
    /// Convert OpenCvSharp.Mat (BGR, 8UC3) to System.Drawing.Bitmap.
    /// </summary>
    private static Bitmap MatToBitmap(Mat mat)
    {
        int w = mat.Width, h = mat.Height;
        var bmp = new Bitmap(w, h, PixelFormat.Format24bppRgb);
        var rect = new Rectangle(0, 0, w, h);
        var data = bmp.LockBits(rect, ImageLockMode.WriteOnly, PixelFormat.Format24bppRgb);
        try
        {
            int srcStride = (int)mat.Step();
            int dstStride = data.Stride;
            int rowBytes = w * 3;

            for (int y = 0; y < h; y++)
            {
                IntPtr srcRow = mat.Data + y * srcStride;
                IntPtr dstRow = data.Scan0 + y * dstStride;
                byte[] row = new byte[rowBytes];
                Marshal.Copy(srcRow, row, 0, rowBytes);
                Marshal.Copy(row, 0, dstRow, rowBytes);
            }

            return bmp;
        }
        finally
        {
            bmp.UnlockBits(data);
        }
    }

    #endregion
}
