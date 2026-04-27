using System;
using System.IO;
using System.Text.Json;

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

    /// <summary>Detector label for display (always "MaskRCNN").</summary>
    public string Detector { get; set; } = "MaskRCNN";

    /// <summary>
    /// When true, skip geometric measurement and go straight to MaskRCNN inference.
    /// Set for cameras that have no visible hole (e.g. slot 2 and 3 side-view cameras).
    /// </summary>
    public bool SkipGeoMeasurement { get; set; } = false;

    /// <summary>Trigger group (1 or 2). Cameras on the same group fire together.</summary>
    public int TriggerGroup { get; set; } = 1;

    /// <summary>
    /// Delay (ms) after trigger signal before capturing this camera's frame.
    /// Accounts for different exposure times per camera.
    /// </summary>
    public int CaptureDelayMs { get; set; } = 50;

    /// <summary>
    /// Hardware trigger source line on the camera (e.g. "Line0", "Line1").
    /// Used only when CameraTriggerMode is "hardware".
    /// </summary>
    public string TriggerSource { get; set; } = "Line0";

    /// <summary>
    /// Delay in microseconds applied inside the camera after the hardware trigger pulse
    /// before the exposure begins. Default 50 µs.
    /// </summary>
    public double TriggerDelayUs { get; set; } = 0.0;
    private static readonly string DefaultPath =
        Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Assets", "camera_slots.json");

    private static readonly JsonSerializerOptions JsonOpts = new()
    {
        WriteIndented = true,
    };

    /// <summary>Save camera slot configs to JSON.</summary>
    public static void Save(CameraSlotConfig[] configs, string? path = null)
    {
        path ??= DefaultPath;
        string json = JsonSerializer.Serialize(configs, JsonOpts);
        File.WriteAllText(path, json);
    }

    /// <summary>Load camera slot configs from JSON. Returns null if file doesn't exist.</summary>
    public static CameraSlotConfig[]? Load(string? path = null)
    {
        path ??= DefaultPath;
        if (!File.Exists(path)) return null;
        string json = File.ReadAllText(path);
        return JsonSerializer.Deserialize<CameraSlotConfig[]>(json);
    }
}
