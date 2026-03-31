using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Threading;

namespace RoboViz;

/// <summary>
/// Producer-consumer trigger pipeline.
///   Producer: polls two Modbus trigger coils, edge-detects LOW?HIGH.
///   Consumer: captures camera frames (per-camera delays), runs inference, posts results.
///   Supports flexible camera-to-trigger mapping via CameraSlotConfig[].
/// </summary>
public class TriggerService : IDisposable
{
    private readonly ModbusService _modbus;
    private readonly CameraManager? _cameras;
    private readonly InspectionService? _inspection;
    private readonly TriggerConfig _config;
    private readonly CameraSlotConfig[] _cameraSlots;
    private readonly Action<TriggerResultEvent> _onResult;

    private readonly BlockingCollection<TriggerEvent> _queue = new(boundedCapacity: 16);
    private Thread? _producerThread;
    private Thread? _consumerThread;
    private volatile bool _running;

    // Edge detection: only trigger on LOW ? HIGH transition
    private bool _prevTrigger1;
    private bool _prevTrigger2;

    public bool IsRunning => _running;

    /// <param name="modbus">Already-connected ModbusService instance.</param>
    /// <param name="cameras">CameraManager instance, or null if cameras not streaming.</param>
    /// <param name="inspection">InspectionService, or null if model not loaded.</param>
    /// <param name="config">Trigger configuration from file.</param>
    /// <param name="cameraSlots">Per-camera configuration (detector, trigger group, delay).</param>
    /// <param name="onResult">Callback invoked on the consumer thread with results (caller must marshal to UI).</param>
    public TriggerService(
        ModbusService modbus,
        CameraManager? cameras,
        InspectionService? inspection,
        TriggerConfig config,
        CameraSlotConfig[] cameraSlots,
        Action<TriggerResultEvent> onResult)
    {
        _modbus = modbus;
        _cameras = cameras;
        _inspection = inspection;
        _config = config;
        _cameraSlots = cameraSlots;
        _onResult = onResult;

        string camStatus = cameras == null ? "NULL" : (cameras.IsStreaming ? "streaming" : "initialized");
        string inspStatus = inspection == null ? "NULL" : "loaded";
        int t1Count = cameraSlots.Count(c => c.TriggerGroup == 1 && c.DeviceIndex >= 0);
        int t2Count = cameraSlots.Count(c => c.TriggerGroup == 2 && c.DeviceIndex >= 0);
        Debug.WriteLine($"[TriggerService] cameras={camStatus}, inspection={inspStatus}, T1={t1Count} cams, T2={t2Count} cams");
        MaskRCNNDetector.LogDiag($"[TriggerService] Created: cameras={camStatus}, inspection={inspStatus}, T1={t1Count}, T2={t2Count}");
    }

    public void Start()
    {
        if (_running) return;
        _running = true;
        _prevTrigger1 = false;
        _prevTrigger2 = false;

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

        MaskRCNNDetector.LogDiag("[Trigger] Producer-consumer stopped.");
    }

    // ?? Producer ??????????????????????????????????????????????

    private void ProducerLoop()
    {
        ushort addrT1 = _config.TriggerCoil_Cam13;
        ushort addrT2 = _config.TriggerCoil_Cam24;

        Debug.WriteLine($"[Trigger] Producer started: T1 coil={addrT1}, T2 coil={addrT2}, poll={_config.PollIntervalMs}ms");

        int consecutiveFailures = 0;

        while (_running)
        {
            try
            {
                var bitsT1 = _modbus.ReadCoils(addrT1, 1);
                var bitsT2 = _modbus.ReadCoils(addrT2, 1);

                if (bitsT1 == null || bitsT2 == null)
                {
                    consecutiveFailures++;
                    if (consecutiveFailures <= 3 || consecutiveFailures % 50 == 0)
                        Debug.WriteLine($"[Trigger] ReadCoils FAILED (x{consecutiveFailures}): {_modbus.LastError}");
                    int backoff = Math.Min(_config.PollIntervalMs * 5, 2000);
                    Thread.Sleep(consecutiveFailures <= 3 ? _config.PollIntervalMs : backoff);
                    continue;
                }

                bool t1Now = bitsT1[0];
                bool t2Now = bitsT2[0];

                if (consecutiveFailures > 0)
                {
                    Debug.WriteLine($"[Trigger] Read recovered after {consecutiveFailures} failures.");
                    consecutiveFailures = 0;
                }

                Debug.WriteLine($"[Trigger] T1={( t1Now ? 1 : 0 )} T2={( t2Now ? 1 : 0 )}");

                // Edge detect: LOW ? HIGH
                if (t1Now && !_prevTrigger1)
                {
                    _queue.TryAdd(new TriggerEvent(TriggerType.Trigger1, DateTime.UtcNow));
                    Debug.WriteLine($"[Trigger] >>> TRIGGER 1 HIGH at {DateTime.Now:HH:mm:ss.fff}");
                    MaskRCNNDetector.LogDiag("[Trigger] Trigger 1 detected.");
                }

                if (t2Now && !_prevTrigger2)
                {
                    _queue.TryAdd(new TriggerEvent(TriggerType.Trigger2, DateTime.UtcNow));
                    Debug.WriteLine($"[Trigger] >>> TRIGGER 2 HIGH at {DateTime.Now:HH:mm:ss.fff}");
                    MaskRCNNDetector.LogDiag("[Trigger] Trigger 2 detected.");
                }

                _prevTrigger1 = t1Now;
                _prevTrigger2 = t2Now;
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"[Trigger] Producer exception: {ex.Message}");
                MaskRCNNDetector.LogDiag($"[Trigger] Producer error: {ex.Message}");
            }

            Thread.Sleep(_config.PollIntervalMs);
        }
    }

    // ?? Consumer ??????????????????????????????????????????????

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
        int triggerGroup = trigger.Type == TriggerType.Trigger1 ? 1 : 2;
        string label = $"Trigger {triggerGroup}";

        // Find camera slots assigned to this trigger group
        var slotsForTrigger = _cameraSlots
            .Where(c => c.TriggerGroup == triggerGroup && c.DeviceIndex >= 0)
            .OrderBy(c => c.CaptureDelayMs)
            .ToArray();

        if (slotsForTrigger.Length == 0)
        {
            Debug.WriteLine($"[Trigger] {label}: no cameras assigned.");
            _onResult(new TriggerResultEvent
            {
                Type = trigger.Type,
                Results = [],
                Slots = [],
                BatchMs = 0,
                ModbusWriteOk = false,
                ModbusError = $"No cameras on {label}",
            });
            return;
        }

        Debug.WriteLine($"[Trigger] Processing {label} ({slotsForTrigger.Length} cameras) at {DateTime.Now:HH:mm:ss.fff}");

        if (_cameras == null || _inspection == null)
        {
            string reason = _cameras == null && _inspection == null ? "cameras AND model null"
                : _cameras == null ? "cameras null"
                : "model null";
            Debug.WriteLine($"[Trigger] {label} — {reason}, skipping inference.");
            _onResult(new TriggerResultEvent
            {
                Type = trigger.Type,
                Results = [],
                Slots = [],
                BatchMs = 0,
                ModbusWriteOk = false,
                ModbusError = $"Not ready: {reason}",
            });
            return;
        }

        // On-demand camera start
        if (!_cameras.IsStreaming)
        {
            Debug.WriteLine($"[Trigger] {label} — cameras not streaming, starting now...");
            try
            {
                _cameras.StartStreaming();
                Thread.Sleep(500);
                Debug.WriteLine($"[Trigger] {label} — cameras started and warmed up.");
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"[Trigger] {label} — camera start failed: {ex.Message}");
                _onResult(new TriggerResultEvent
                {
                    Type = trigger.Type,
                    Results = [],
                    Slots = [],
                    BatchMs = 0,
                    ModbusWriteOk = false,
                    ModbusError = $"Camera start failed: {ex.Message}",
                });
                return;
            }
        }

        // Capture frames with per-camera delays (sorted by delay ascending)
        var triggerTime = trigger.Timestamp;
        var frames = new Bitmap?[slotsForTrigger.Length];

        for (int i = 0; i < slotsForTrigger.Length; i++)
        {
            var slotCfg = slotsForTrigger[i];

            // Wait until this camera's delay has elapsed since trigger
            int elapsed = (int)(DateTime.UtcNow - triggerTime).TotalMilliseconds;
            int remaining = slotCfg.CaptureDelayMs - elapsed;
            if (remaining > 0)
                Thread.Sleep(remaining);

            // Grab latest frame with retry
            const int maxRetries = 10;
            const int retryDelayMs = 200;
            for (int attempt = 0; attempt < maxRetries; attempt++)
            {
                frames[i] = _cameras.GetLatestFrame(slotCfg.Slot);
                if (frames[i] != null) break;
                if (attempt < maxRetries - 1)
                    Thread.Sleep(retryDelayMs);
            }

            Debug.WriteLine($"[Trigger] CAM {slotCfg.Slot + 1} (delay {slotCfg.CaptureDelayMs}ms): {(frames[i] != null ? "OK" : "NULL")}");
        }

        // Run inference on captured frames
        var resultList = new System.Collections.Generic.List<InspectionResult>();
        var frameList = new System.Collections.Generic.List<Bitmap>();
        var slotList = new System.Collections.Generic.List<int>();
        var sw = Stopwatch.StartNew();

        for (int i = 0; i < slotsForTrigger.Length; i++)
        {
            if (frames[i] == null) continue;
            var slotCfg = slotsForTrigger[i];

            Debug.WriteLine($"[Trigger] CAM {slotCfg.Slot + 1} frame: {frames[i]!.Width}x{frames[i]!.Height} px, detector={slotCfg.Detector}");
            var result = slotCfg.Detector == "MaskRCNN"
                ? _inspection.InspectMaskRCNN(frames[i]!)
                : _inspection.InspectPatchCore(frames[i]!);
            resultList.Add(result);
            frameList.Add(frames[i]!);
            slotList.Add(slotCfg.Slot);

            Debug.WriteLine($"[Trigger] CAM {slotCfg.Slot + 1} [{slotCfg.Detector}] => {result.Verdict} | " +
                $"total={result.TotalMs}ms geo={result.GeoMs}ms infer={result.InferenceMs}ms" +
                (slotCfg.Detector == "MaskRCNN"
                    ? $" | defect={result.HasDefect} topScore={result.TopScore:F3}"
                    : $" | anomalyScore={result.AnomalyScore:F3} threshold={result.AnomalyThreshold:F3}") +
                (result.FailReasons.Count > 0 ? $" | reasons=[{string.Join(", ", result.FailReasons)}]" : "") +
                (result.ErrorMessage != null ? $" | ERROR: {result.ErrorMessage}" : ""));
        }

        var results = resultList.ToArray();
        long batchMs = sw.ElapsedMilliseconds;
        Debug.WriteLine($"[Trigger] {label} batch complete: {batchMs}ms | verdicts=[{string.Join(", ", results.Select(r => r.Verdict))}]");

        // Dispose frames not used as overlay
        for (int i = 0; i < frameList.Count; i++)
        {
            if (!ReferenceEquals(frameList[i], results[i].OverlayImage))
                frameList[i].Dispose();
        }

        // Write output coils — DISABLED for now (read-only trigger mode)
        bool modbusOk = false;
        string? modbusError = "write disabled";

        _onResult(new TriggerResultEvent
        {
            Type = trigger.Type,
            Results = results,
            Slots = [.. slotList],
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
