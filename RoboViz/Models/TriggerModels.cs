using System;
using System.IO;
using System.Text.Json;

namespace RoboViz;

/// <summary>
/// Configuration loaded from trigger_config.json.
/// </summary>
public class TriggerConfig
{
    public string ComPort { get; set; } = "COM7";
    public int BaudRate { get; set; } = 115200;
    public byte SlaveId { get; set; } = 1;
    public ushort TriggerCoil_Cam13 { get; set; } = 0;
    public ushort TriggerCoil_Cam24 { get; set; } = 1;
    public ushort OutputCoilAddress { get; set; } = 10;
    public int CaptureDelayMs { get; set; } = 50;
    public int PollIntervalMs { get; set; } = 200;

    /// <summary>
    /// Legacy mode selector kept for configuration compatibility.
    /// "software": Modbus polling detects trigger, then captures frames.
    /// "hardware": sensor wired directly to camera trigger input, frames arrive automatically.
    /// </summary>
    public string CameraTriggerMode { get; set; } = "software";

    /// <summary>
    /// Rejection output coil configuration (addresses, delays, conflict priority).
    /// </summary>
    public OutputCoilConfig OutputCoils { get; set; } = new();

    /// <summary>
    /// Per-camera slot configuration. Default: 3 cameras on Trigger 1, 1 on Trigger 2.
    /// </summary>
    /// <summary>
    /// Per-camera slot configuration. Default mapping:
    ///   CAM 1 (slot 0)              ? Trigger 1, full geo (cam1 OpenCV pipeline) + MaskRCNN
    ///   CAM 2 (slot 1)              ? Trigger 2, YOLO-bbox geo + MaskRCNN
    ///   CAM 3, CAM 4 (slots 2, 3)   ? Trigger 2, NO geo (resize only) + MaskRCNN
    /// </summary>
    public CameraSlotConfig[] CameraSlots { get; set; } =
    [
        new() { Slot = 0, TriggerGroup = 1, Detector = "MaskRCNN", CaptureDelayMs = 50, SkipGeoMeasurement = false },
        new() { Slot = 1, TriggerGroup = 2, Detector = "MaskRCNN", CaptureDelayMs = 50, SkipGeoMeasurement = false },
        new() { Slot = 2, TriggerGroup = 2, Detector = "MaskRCNN", CaptureDelayMs = 50, SkipGeoMeasurement = true  },
        new() { Slot = 3, TriggerGroup = 2, Detector = "MaskRCNN", CaptureDelayMs = 50, SkipGeoMeasurement = true  },
    ];

    /// <summary>Coil address for trigger group 1.</summary>
    public ushort GetTriggerCoil(TriggerType type) =>
        type == TriggerType.Trigger1 ? TriggerCoil_Cam13 : TriggerCoil_Cam24;

    private static readonly string DefaultPath =
        Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Assets", "trigger_config.json");

    public static TriggerConfig Load(string? path = null)
    {
        path ??= DefaultPath;
        MaskRCNNDetector.LogDiag($"[TriggerConfig] Loading from: {path}");

        if (!File.Exists(path))
        {
            MaskRCNNDetector.LogDiag($"[TriggerConfig] File NOT FOUND at: {path} — using class defaults.");
            var cfg = new TriggerConfig();
            MaskRCNNDetector.LogDiag($"[TriggerConfig] Defaults: {cfg.ComPort} @ {cfg.BaudRate}, slave {cfg.SlaveId}");
            return cfg;
        }

        var json = File.ReadAllText(path);
        var config = JsonSerializer.Deserialize<TriggerConfig>(json) ?? new TriggerConfig();
        MaskRCNNDetector.LogDiag($"[TriggerConfig] Loaded: {config.ComPort} @ {config.BaudRate}, slave {config.SlaveId}, " +
            $"trigger coils {config.TriggerCoil_Cam13}/{config.TriggerCoil_Cam24}, output coil {config.OutputCoilAddress}, " +
            $"poll {config.PollIntervalMs}ms");
        return config;
    }

    public void Save(string? path = null)
    {
        path ??= DefaultPath;
        var json = JsonSerializer.Serialize(this, new JsonSerializerOptions { WriteIndented = true });
        File.WriteAllText(path, json);
    }
}

/// <summary>
/// Output coil addresses, delays, and conflict priority for conveyor rejection.
/// </summary>
public class OutputCoilConfig
{
    /// <summary>Coil address: CAM 1 rework (sensor 3001).</summary>
    public ushort Cam1_ReworkCoil { get; set; } = 3010;
    /// <summary>Coil address: CAM 1 reject (sensor 3001).</summary>
    public ushort Cam1_RejectCoil { get; set; } = 3011;
    /// <summary>Coil address: CAM 2/3/4 reject (sensor 3002).</summary>
    public ushort Cam234_RejectCoil { get; set; } = 3012;
    /// <summary>Coil address: CAM 2 rework (sensor 3002).</summary>
    public ushort Cam2_ReworkCoil { get; set; } = 3013;

    /// <summary>Delay (ms) before activating CAM 1 rework coil after verdict.</summary>
    public int Cam1_ReworkDelayMs { get; set; } = 500;
    /// <summary>Delay (ms) before activating CAM 1 reject coil after verdict.</summary>
    public int Cam1_RejectDelayMs { get; set; } = 500;
    /// <summary>Delay (ms) before activating CAM 2/3/4 reject coil after verdict.</summary>
    public int Cam234_RejectDelayMs { get; set; } = 500;
    /// <summary>Delay (ms) before activating CAM 2 rework coil after verdict.</summary>
    public int Cam2_ReworkDelayMs { get; set; } = 500;

    /// <summary>How long (ms) the coil stays ON before being turned OFF.</summary>
    public int CoilOnDurationMs { get; set; } = 200;

    /// <summary>
    /// Conflict resolution when CAM 2 gives REWORK but CAM 3/4 gives REJECT.
    /// "reject" (default): only reject coil fires. "rework": both fire.
    /// </summary>
    public string ConflictPriority { get; set; } = "reject";
}

/// <summary>
/// Which trigger group fired.
/// </summary>
public enum TriggerType { Trigger1, Trigger2 }

/// <summary>
/// A trigger event produced by the active trigger source.
/// In the current hardware-trigger pipeline it is queued by a sentinel camera waiter thread.
/// </summary>
public readonly record struct TriggerEvent(TriggerType Type, DateTime Timestamp);

/// <summary>
/// Status of a single poll cycle from the producer thread.
/// Reported to the UI for live read feedback.
/// </summary>
/// <param name="Success">True if both coils were read successfully.</param>
/// <param name="Coil1Value">Current value of trigger coil 1 (only valid when Success=true).</param>
/// <param name="Coil2Value">Current value of trigger coil 2 (only valid when Success=true).</param>
/// <param name="ConsecutiveFailures">Number of consecutive read failures (0 when Success=true).</param>
/// <param name="ErrorMessage">Error message on failure, null on success.</param>
public readonly record struct TriggerPollStatus(
    bool Success,
    bool Coil1Value,
    bool Coil2Value,
    int ConsecutiveFailures,
    string? ErrorMessage);

/// <summary>
/// Result posted from the consumer back to the UI.
/// </summary>
public class TriggerResultEvent
{
    public required TriggerType Type { get; init; }
    public required InspectionResult[] Results { get; init; }
    /// <summary>Camera slot indices corresponding to each result.</summary>
    public required int[] Slots { get; init; }
    public required long BatchMs { get; init; }
    public bool ModbusWriteOk { get; init; }
    public string? ModbusError { get; init; }
    /// <summary>Which output coils were activated (e.g. "Cam1_Rework@3010").</summary>
    public string? CoilsActivated { get; init; }
}
