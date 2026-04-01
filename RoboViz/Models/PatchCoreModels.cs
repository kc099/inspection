namespace RoboViz;

/// <summary>
/// Result from a single PatchCore anomaly detection inference pass.
/// </summary>
public class PatchCoreResult
{
    public float AnomalyScore { get; set; }
    public float[,] AnomalyMap { get; set; } = null!;
    public bool IsAnomaly { get; set; }
}

/// <summary>
/// Metadata from the JSON file produced by the PatchCore export script.
/// Supports both the original export JSON and the ablation JSON format.
/// </summary>
public class PatchCoreMetadata
{
    public string model_name { get; set; } = "";
    public string model { get; set; } = "";
    public string backbone { get; set; } = "";
    public int[]? input_shape { get; set; }
    public int resize { get; set; } = 660;
    public int center_crop { get; set; } = 640;
    public float? recommended_threshold { get; set; }
    public string onnx_file { get; set; } = "";
    public float onnx_size_mb { get; set; }

    // Ablation JSON fields (from fp16 validation)
    public float? good_max_fp16 { get; set; }
    public float? defect_min_fp16 { get; set; }
    public float? good_max_fp32 { get; set; }
    public float? defect_min_fp32 { get; set; }

    /// <summary>
    /// Compute a threshold from available metadata.
    /// Priority: recommended_threshold, then midpoint of (defect_min + good_max) fp16, then fallback.
    /// </summary>
    public float ComputeThreshold(float fallback = 15.0f)
    {
        if (recommended_threshold.HasValue)
            return recommended_threshold.Value;

        if (defect_min_fp16.HasValue && good_max_fp16.HasValue)
            return (defect_min_fp16.Value + good_max_fp16.Value) / 2f;

        if (defect_min_fp32.HasValue && good_max_fp32.HasValue)
            return (defect_min_fp32.Value + good_max_fp32.Value) / 2f;

        return fallback;
    }
}
