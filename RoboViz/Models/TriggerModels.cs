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
    /// Per-camera slot configuration. Default: 3 cameras on Trigger 1, 1 on Trigger 2.
    /// </summary>
    public CameraSlotConfig[] CameraSlots { get; set; } =
    [
        new() { Slot = 0, TriggerGroup = 1, Detector = "MaskRCNN",  CaptureDelayMs = 50 },
        new() { Slot = 1, TriggerGroup = 1, Detector = "MaskRCNN",  CaptureDelayMs = 50 },
        new() { Slot = 2, TriggerGroup = 1, Detector = "PatchCore", CaptureDelayMs = 50 },
        new() { Slot = 3, TriggerGroup = 2, Detector = "PatchCore", CaptureDelayMs = 50 },
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
/// Which trigger group fired.
/// </summary>
public enum TriggerType { Trigger1, Trigger2 }

/// <summary>
/// A trigger event produced by the polling thread.
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
}
