using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Text.Json;
using System.Threading;

namespace RoboViz;

/// <summary>
/// Configuration loaded from trigger_config.json.
/// </summary>
public class TriggerConfig
{
    public string ComPort { get; set; } = "COM3";
    public int BaudRate { get; set; } = 9600;
    public byte SlaveId { get; set; } = 1;
    public ushort TriggerCoil_Cam13 { get; set; } = 0;
    public ushort TriggerCoil_Cam24 { get; set; } = 1;
    public ushort OutputCoilAddress { get; set; } = 10;
    public int CaptureDelayMs { get; set; } = 50;
    public int PollIntervalMs { get; set; } = 200;

    private static readonly string DefaultPath =
        Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "trigger_config.json");

    public static TriggerConfig Load(string? path = null)
    {
        path ??= DefaultPath;
        if (!File.Exists(path))
        {
            var cfg = new TriggerConfig();
            cfg.Save(path);
            return cfg;
        }

        var json = File.ReadAllText(path);
        return JsonSerializer.Deserialize<TriggerConfig>(json) ?? new TriggerConfig();
    }

    public void Save(string? path = null)
    {
        path ??= DefaultPath;
        var json = JsonSerializer.Serialize(this, new JsonSerializerOptions { WriteIndented = true });
        File.WriteAllText(path, json);
    }
}

/// <summary>
/// Which camera pair was triggered.
/// </summary>
public enum TriggerType { Cam13, Cam24 }

/// <summary>
/// A trigger event produced by the polling thread.
/// </summary>
public readonly record struct TriggerEvent(TriggerType Type, DateTime Timestamp);

/// <summary>
/// Result posted from the consumer back to the UI.
/// </summary>
public class TriggerResultEvent
{
    public required TriggerType Type { get; init; }
    public required InspectionResult[] Results { get; init; }
    public required long BatchMs { get; init; }
    public bool ModbusWriteOk { get; init; }
    public string? ModbusError { get; init; }
}

/// <summary>
/// Producer-consumer trigger pipeline.
///   Producer: polls two Modbus input coils every PollIntervalMs, edge-detects LOW?HIGH.
///   Consumer: captures camera frames, runs inference, writes output coils.
/// </summary>
public class TriggerService : IDisposable
{
    private readonly ModbusService _modbus;
    private readonly CameraManager? _cameras;
    private readonly InspectionService? _inspection;
    private readonly TriggerConfig _config;
    private readonly Action<TriggerResultEvent> _onResult;
    private readonly Func<int, string> _getDetectorForCamera;

    private readonly BlockingCollection<TriggerEvent> _queue = new(boundedCapacity: 16);
    private Thread? _producerThread;
    private Thread? _consumerThread;
    private volatile bool _running;

    // Edge detection: only trigger on LOW ? HIGH transition
    private bool _prevCam13;
    private bool _prevCam24;

    public bool IsRunning => _running;

    /// <param name="modbus">Already-connected ModbusService instance.</param>
    /// <param name="cameras">CameraManager instance, or null if cameras not streaming.</param>
    /// <param name="inspection">InspectionService, or null if model not loaded.</param>
    /// <param name="config">Trigger configuration from file.</param>
    /// <param name="getDetectorForCamera">Returns "MaskRCNN" or "PatchCore" for camera index 0-3.</param>
    /// <param name="onResult">Callback invoked on the consumer thread with results (caller must marshal to UI).</param>
    public TriggerService(
        ModbusService modbus,
        CameraManager? cameras,
        InspectionService? inspection,
        TriggerConfig config,
        Func<int, string> getDetectorForCamera,
        Action<TriggerResultEvent> onResult)
    {
        _modbus = modbus;
        _cameras = cameras;
        _inspection = inspection;
        _config = config;
        _getDetectorForCamera = getDetectorForCamera;
        _onResult = onResult;
    }

    public void Start()
    {
        if (_running) return;
        _running = true;
        _prevCam13 = false;
        _prevCam24 = false;

        _producerThread = new Thread(ProducerLoop) { IsBackground = true, Name = "TriggerProducer" };
        _consumerThread = new Thread(ConsumerLoop) { IsBackground = true, Name = "TriggerConsumer" };
        _producerThread.Start();
        _consumerThread.Start();

        MaskRCNNDetector.LogDiag("[Trigger] Producer-consumer started.");
    }

    public void Stop()
    {
        if (!_running) return;
        _running = false;
        _queue.CompleteAdding();

        _producerThread?.Join(2000);
        _consumerThread?.Join(5000);
        _producerThread = null;
        _consumerThread = null;

        // Re-create queue for potential restart
        MaskRCNNDetector.LogDiag("[Trigger] Producer-consumer stopped.");
    }

    // ???? Producer ?????????????????????????????????????????????????

    private void ProducerLoop()
    {
        ushort addr13 = _config.TriggerCoil_Cam13;
        ushort addr24 = _config.TriggerCoil_Cam24;

        Debug.WriteLine($"[Trigger] Producer started: ReadCoils addr13={addr13} addr24={addr24} poll={_config.PollIntervalMs}ms");

        int consecutiveFailures = 0;

        while (_running)
        {
            try
            {
                var bits13 = _modbus.ReadCoils(addr13, 1);
                var bits24 = _modbus.ReadCoils(addr24, 1);

                if (bits13 == null || bits24 == null)
                {
                    consecutiveFailures++;
                    if (consecutiveFailures <= 3 || consecutiveFailures % 50 == 0)
                        Debug.WriteLine($"[Trigger] ReadCoils FAILED (x{consecutiveFailures}): {_modbus.LastError}");
                    int backoff = Math.Min(_config.PollIntervalMs * 5, 2000);
                    Thread.Sleep(consecutiveFailures <= 3 ? _config.PollIntervalMs : backoff);
                    continue;
                }

                bool cam13Now = bits13[0];
                bool cam24Now = bits24[0];

                // Successful read — reset backoff
                if (consecutiveFailures > 0)
                {
                    Debug.WriteLine($"[Trigger] Read recovered after {consecutiveFailures} failures.");
                    consecutiveFailures = 0;
                }

                // Print current bit status every poll
                Debug.WriteLine($"[Trigger] Coil13={( cam13Now ? 1 : 0 )} Coil24={( cam24Now ? 1 : 0 )}");

                // Edge detect: LOW ? HIGH
                if (cam13Now && !_prevCam13)
                {
                    _queue.TryAdd(new TriggerEvent(TriggerType.Cam13, DateTime.UtcNow));
                    Debug.WriteLine($"[Trigger] >>> CAM 1+3 HIGH at {DateTime.Now:HH:mm:ss.fff}");
                    MaskRCNNDetector.LogDiag("[Trigger] CAM 1+3 trigger detected.");
                }

                if (cam24Now && !_prevCam24)
                {
                    _queue.TryAdd(new TriggerEvent(TriggerType.Cam24, DateTime.UtcNow));
                    Debug.WriteLine($"[Trigger] >>> CAM 2+4 HIGH at {DateTime.Now:HH:mm:ss.fff}");
                    MaskRCNNDetector.LogDiag("[Trigger] CAM 2+4 trigger detected.");
                }

                _prevCam13 = cam13Now;
                _prevCam24 = cam24Now;
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"[Trigger] Producer exception: {ex.Message}");
                MaskRCNNDetector.LogDiag($"[Trigger] Producer error: {ex.Message}");
            }

            Thread.Sleep(_config.PollIntervalMs);
        }
    }

    // ???? Consumer ?????????????????????????????????????????????????

    private void ConsumerLoop()
    {
        foreach (var trigger in _queue.GetConsumingEnumerable())
        {
            if (!_running) break;

            try
            {
                ProcessTrigger(trigger);
            }
            catch (Exception ex)
            {
                MaskRCNNDetector.LogDiag($"[Trigger] Consumer error: {ex.Message}");
            }
        }
    }

    private void ProcessTrigger(TriggerEvent trigger)
    {
        string pair = trigger.Type == TriggerType.Cam13 ? "CAM 1+3" : "CAM 2+4";
        Debug.WriteLine($"[Trigger] Processing {pair} at {DateTime.Now:HH:mm:ss.fff}");

        // If cameras or model not available, just report the trigger and return
        if (_cameras == null || _inspection == null)
        {
            Debug.WriteLine($"[Trigger] {pair} coil HIGH — cameras/model not ready, skipping inference.");
            _onResult(new TriggerResultEvent
            {
                Type = trigger.Type,
                Results = [],
                BatchMs = 0,
                ModbusWriteOk = false,
                ModbusError = "No cameras/model",
            });
            return;
        }

        // Optional capture delay (e.g., wait for part to be in position)
        if (_config.CaptureDelayMs > 0)
            Thread.Sleep(_config.CaptureDelayMs);

        // Determine which camera slots to capture
        int[] slots = trigger.Type == TriggerType.Cam13 ? [0, 2] : [1, 3];

        // Capture frames
        var frames = new Bitmap?[2];
        for (int i = 0; i < 2; i++)
            frames[i] = _cameras.GetLatestFrame(slots[i]);

        if (frames[0] == null || frames[1] == null)
        {
            Debug.WriteLine($"[Trigger] {pair} coil HIGH — camera frame not available.");
            MaskRCNNDetector.LogDiag(
                $"[Trigger] Skipped {trigger.Type}: camera frame not available.");
            frames[0]?.Dispose();
            frames[1]?.Dispose();
            _onResult(new TriggerResultEvent
            {
                Type = trigger.Type,
                Results = [],
                BatchMs = 0,
                ModbusWriteOk = false,
                ModbusError = "Frame not available",
            });
            return;
        }

        // Run inference
        var results = new InspectionResult[2];
        var sw = Stopwatch.StartNew();

        for (int i = 0; i < 2; i++)
        {
            string detector = _getDetectorForCamera(slots[i]);
            results[i] = detector == "MaskRCNN"
                ? _inspection.InspectMaskRCNN(frames[i]!)
                : _inspection.InspectPatchCore(frames[i]!);
        }

        long batchMs = sw.ElapsedMilliseconds;

        // Dispose frames that aren't used as overlay
        for (int i = 0; i < 2; i++)
        {
            if (!ReferenceEquals(frames[i], results[i].OverlayImage))
                frames[i]!.Dispose();
        }

        // Write output coils — single attempt, no retry
        bool modbusOk = false;
        string? modbusError = null;

        if (_modbus.IsConnected)
        {
            bool pass0 = results[0].Verdict == "PASS";
            bool pass1 = results[1].Verdict == "PASS";
            bool bothPass = pass0 && pass1;

            // For CAM 1+3 ? output coil+0, for CAM 2+4 ? output coil+1
            ushort coilAddr = (ushort)(_config.OutputCoilAddress +
                (trigger.Type == TriggerType.Cam13 ? 0 : 1));

            modbusOk = _modbus.WriteSingleCoil(coilAddr, !bothPass);
            if (!modbusOk) modbusError = _modbus.LastError;
        }

        // Post result to UI callback
        _onResult(new TriggerResultEvent
        {
            Type = trigger.Type,
            Results = results,
            BatchMs = batchMs,
            ModbusWriteOk = modbusOk,
            ModbusError = modbusError,
        });
    }

    public void Dispose()
    {
        Stop();
        _queue.Dispose();
    }
}
