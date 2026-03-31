namespace RoboViz;

/// <summary>
/// Per-camera slot configuration.
/// Defines which physical camera, detector, trigger group, and capture delay for each display slot.
/// </summary>
public class CameraSlotConfig
{
    /// <summary>Display slot index (0-3 ? CAM 1-4).</summary>
    public int Slot { get; set; }

    /// <summary>Physical device index from camera enumeration. -1 = not assigned.</summary>
    public int DeviceIndex { get; set; } = -1;

    /// <summary>"MaskRCNN" or "PatchCore".</summary>
    public string Detector { get; set; } = "MaskRCNN";

    /// <summary>Trigger group (1 or 2). Cameras on the same group fire together.</summary>
    public int TriggerGroup { get; set; } = 1;

    /// <summary>
    /// Delay (ms) after trigger signal before capturing this camera's frame.
    /// Accounts for different exposure times per camera.
    /// </summary>
    public int CaptureDelayMs { get; set; } = 50;
}
