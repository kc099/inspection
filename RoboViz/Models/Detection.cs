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
