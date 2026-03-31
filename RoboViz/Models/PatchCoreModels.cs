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
/// </summary>
public class PatchCoreMetadata
{
    public string model_name { get; set; } = "";
    public string backbone { get; set; } = "";
    public int[]? input_shape { get; set; }
    public int resize { get; set; } = 660;
    public int center_crop { get; set; } = 640;
    public float? recommended_threshold { get; set; }
    public string onnx_file { get; set; } = "";
    public float onnx_size_mb { get; set; }
}
