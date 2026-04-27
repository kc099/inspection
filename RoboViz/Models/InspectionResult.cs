using System.Collections.Generic;
using System.Drawing;

namespace RoboViz;

/// <summary>
/// Combined result from a single inspection pass (geometric + AI detection).
/// </summary>
public class InspectionResult
{
    public string Verdict { get; set; } = string.Empty;
    public string? ErrorMessage { get; set; }
    public List<string> FailReasons { get; set; } = [];

    // Geometric
    public GeometricResult? GeoResult { get; set; }
    public List<MetricEvalResult> MetricResults { get; set; } = [];

    // Mask R-CNN specific
    public bool HasDefect { get; set; }
    public List<Detection> Detections { get; set; } = [];
    public float TopScore { get; set; }
    public List<int> MaskPixelCounts { get; set; } = [];

    // Timing
    public long TotalMs { get; set; }
    public long GeoMs { get; set; }
    public long PrepMs { get; set; }
    public long TensorMs { get; set; }
    public long InferenceMs { get; set; }
    public long OverlayMs { get; set; }

    public Bitmap? OverlayImage { get; set; }
}
