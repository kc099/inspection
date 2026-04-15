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
    private readonly Action<TriggerPollStatus>? _onPollStatus;
    private readonly bool _ownsModbus;

    private BlockingCollection<TriggerEvent> _queue = new(boundedCapacity: 16);
    private Thread? _producerThread;
    private Thread? _consumerThread;
    private volatile bool _running;

    // Edge detection: only trigger on LOW?HIGH transition
    private bool _prevTrigger1;
    private bool _prevTrigger2;

    public bool IsRunning => _running;

    /// <param name="modbus">Already-connected ModbusService instance.</param>
    /// <param name="cameras">CameraManager instance, or null if cameras not streaming.</param>
    /// <param name="inspection">InspectionService, or null if model not loaded.</param>
    /// <param name="config">Trigger configuration from file.</param>
    /// <param name="cameraSlots">Per-camera configuration (detector, trigger group, delay).</param>
    /// <param name="onResult">Callback invoked on the consumer thread with results (caller must marshal to UI).</param>
    /// <param name="onPollStatus">Optional callback invoked every poll cycle with read status (caller must marshal to UI).</param>
    /// <param name="ownsModbus">If true (default), Dispose will also dispose the ModbusService.</param>
    public TriggerService(
        ModbusService modbus,
        CameraManager? cameras,
        InspectionService? inspection,
        TriggerConfig config,
        CameraSlotConfig[] cameraSlots,
        Action<TriggerResultEvent> onResult,
        Action<TriggerPollStatus>? onPollStatus = null,
        bool ownsModbus = true)
    {
        _modbus = modbus;
        _cameras = cameras;
        _inspection = inspection;
        _config = config;
        _cameraSlots = cameraSlots;
        _onResult = onResult;
        _onPollStatus = onPollStatus;
        _ownsModbus = ownsModbus;

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

        try { _queue.CompleteAdding(); } catch { }

        // Join on a background thread to avoid freezing the UI.
        // The producer may be stuck in a 1-second Modbus read timeout.
        var producer = _producerThread;
        var consumer = _consumerThread;
        _producerThread = null;
        _consumerThread = null;

        if (producer != null || consumer != null)
        {
            _ = Task.Run(() =>
            {
                producer?.Join(3000);
                consumer?.Join(3000);
                Debug.WriteLine("[Trigger] Background threads joined.");
            });
        }

        MaskRCNNDetector.LogDiag("[Trigger] Producer-consumer stopped.");
    }

    // ?? Producer ??????????????????????????????????????????????

    private void ProducerLoop()
    {
        ushort addrT1 = _config.TriggerCoil_Cam13;
        ushort addrT2 = _config.TriggerCoil_Cam24;

        Debug.WriteLine($"[Trigger] Producer started: T1 coil={addrT1}, T2 coil={addrT2}, poll={_config.PollIntervalMs}ms");

        int consecutiveFailures = 0;
        const int FlushThreshold = 5;     // flush + rebuild transport
        const int ReconnectThreshold = 15; // full close + reopen COM port

        while (_running)
        {
            try
            {
                var bitsT1 = _modbus.ReadCoils(addrT1, 1);
                var bitsT2 = _modbus.ReadCoils(addrT2, 1);

                if (bitsT1 == null || bitsT2 == null)
                {
                    consecutiveFailures++;

                    if (consecutiveFailures <= 3 || consecutiveFailures % 20 == 0)
                        Debug.WriteLine($"[Trigger] ReadCoils FAILED (x{consecutiveFailures}): {_modbus.LastError}");

                    // Stage 1: flush buffers + rebuild NModbus transport
                    if (consecutiveFailures == FlushThreshold)
                    {
                        Debug.WriteLine($"[Trigger] {consecutiveFailures} failures — flushing bus...");
                        _onPollStatus?.Invoke(new TriggerPollStatus(
                            false, false, false, consecutiveFailures, "Flushing bus..."));
                        if (_modbus.TryRecover())
                        {
                            Debug.WriteLine("[Trigger] Bus flush succeeded, resuming polls.");
                            _prevTrigger1 = false;
                            _prevTrigger2 = false;
                            consecutiveFailures = 0;
                            Thread.Sleep(_config.PollIntervalMs);
                            continue;
                        }
                    }

                    // Stage 2: full serial port close + reopen
                    if (consecutiveFailures >= ReconnectThreshold &&
                        consecutiveFailures % ReconnectThreshold == 0)
                    {
                        Debug.WriteLine($"[Trigger] {consecutiveFailures} failures — full reconnect...");
                        _onPollStatus?.Invoke(new TriggerPollStatus(
                            false, false, false, consecutiveFailures, "Reconnecting COM port..."));
                        if (_modbus.TryReconnect())
                        {
                            Debug.WriteLine("[Trigger] Full reconnect succeeded, resuming polls.");
                            _prevTrigger1 = false;
                            _prevTrigger2 = false;
                            consecutiveFailures = 0;
                            Thread.Sleep(_config.PollIntervalMs);
                            continue;
                        }
                    }

                    // Periodic flush retry between reconnect attempts
                    if (consecutiveFailures > FlushThreshold &&
                        consecutiveFailures < ReconnectThreshold &&
                        consecutiveFailures % FlushThreshold == 0)
                    {
                        _modbus.TryRecover();
                    }

                    _onPollStatus?.Invoke(new TriggerPollStatus(
                        false, false, false, consecutiveFailures, _modbus.LastError));

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

                _onPollStatus?.Invoke(new TriggerPollStatus(
                    true, t1Now, t2Now, 0, null));

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
            bool skipGeo = slotCfg.Detector != "MaskRCNN";
            var result = _inspection.InspectMaskRCNN(frames[i]!, triggerGroup, skipGeo, slot: slotCfg.Slot);
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

        // Write rejection output coils based on verdicts
        var (modbusOk, modbusError, coilsActivated) = WriteRejectionCoils(triggerGroup, results, slotList);

        _onResult(new TriggerResultEvent
        {
            Type = trigger.Type,
            Results = results,
            Slots = [.. slotList],
            BatchMs = batchMs,
            ModbusWriteOk = modbusOk,
            ModbusError = modbusError,
            CoilsActivated = coilsActivated,
        });
    }

    /// <summary>
    /// Evaluate verdicts and write the appropriate rejection/rework coils.
    /// Each coil is turned ON after its configured delay, held for CoilOnDurationMs, then turned OFF.
    /// </summary>
    private (bool ok, string? error, string? coilsActivated) WriteRejectionCoils(
        int triggerGroup, InspectionResult[] results, System.Collections.Generic.List<int> slotList)
    {
        var oc = _config.OutputCoils;
        var activated = new System.Collections.Generic.List<string>();
        bool allOk = true;
        string? lastError = null;

        // Build slot?verdict map
        var verdictBySlot = new System.Collections.Generic.Dictionary<int, string>();
        for (int i = 0; i < results.Length && i < slotList.Count; i++)
            verdictBySlot[slotList[i]] = results[i].Verdict;

        if (triggerGroup == 1)
        {
            // Sensor 3001: CAM 1 (slot 0)
            verdictBySlot.TryGetValue(0, out string? cam1Verdict);

            if (cam1Verdict == "REWORK")
            {
                if (ActivateCoil(oc.Cam1_ReworkCoil, oc.Cam1_ReworkDelayMs, oc.CoilOnDurationMs, out string? err))
                    activated.Add($"Cam1_Rework@{oc.Cam1_ReworkCoil}");
                else { allOk = false; lastError = err; }
            }
            else if (cam1Verdict == "REJECT")
            {
                if (ActivateCoil(oc.Cam1_RejectCoil, oc.Cam1_RejectDelayMs, oc.CoilOnDurationMs, out string? err))
                    activated.Add($"Cam1_Reject@{oc.Cam1_RejectCoil}");
                else { allOk = false; lastError = err; }
            }
        }
        else if (triggerGroup == 2)
        {
            // Sensor 3002: CAM 2 (slot 1, top), CAM 3 (slot 2, side), CAM 4 (slot 3, side)
            verdictBySlot.TryGetValue(1, out string? cam2Verdict);
            verdictBySlot.TryGetValue(2, out string? cam3Verdict);
            verdictBySlot.TryGetValue(3, out string? cam4Verdict);

            bool anyReject = cam2Verdict == "REJECT" || cam3Verdict == "REJECT" || cam4Verdict == "REJECT";
            bool cam2Rework = cam2Verdict == "REWORK";

            bool rejectPriority = !string.Equals(oc.ConflictPriority, "rework", StringComparison.OrdinalIgnoreCase);

            if (rejectPriority)
            {
                // "reject" priority: if any reject, fire reject coil only; rework only if no reject
                if (anyReject)
                {
                    if (ActivateCoil(oc.Cam234_RejectCoil, oc.Cam234_RejectDelayMs, oc.CoilOnDurationMs, out string? err))
                        activated.Add($"Cam234_Reject@{oc.Cam234_RejectCoil}");
                    else { allOk = false; lastError = err; }
                }
                else if (cam2Rework)
                {
                    if (ActivateCoil(oc.Cam2_ReworkCoil, oc.Cam2_ReworkDelayMs, oc.CoilOnDurationMs, out string? err))
                        activated.Add($"Cam2_Rework@{oc.Cam2_ReworkCoil}");
                    else { allOk = false; lastError = err; }
                }
            }
            else
            {
                // "rework" priority: both can fire
                if (anyReject)
                {
                    if (ActivateCoil(oc.Cam234_RejectCoil, oc.Cam234_RejectDelayMs, oc.CoilOnDurationMs, out string? err))
                        activated.Add($"Cam234_Reject@{oc.Cam234_RejectCoil}");
                    else { allOk = false; lastError = err; }
                }
                if (cam2Rework)
                {
                    if (ActivateCoil(oc.Cam2_ReworkCoil, oc.Cam2_ReworkDelayMs, oc.CoilOnDurationMs, out string? err))
                        activated.Add($"Cam2_Rework@{oc.Cam2_ReworkCoil}");
                    else { allOk = false; lastError = err; }
                }
            }
        }

        string? coilsSummary = activated.Count > 0 ? string.Join(", ", activated) : null;
        bool ok = activated.Count > 0 ? allOk : false;
        string? error = activated.Count == 0 ? "no coil needed" : lastError;

        Debug.WriteLine($"[Trigger] Coils: group={triggerGroup}, activated=[{coilsSummary ?? "none"}], ok={ok}, err={error}");
        return (ok, error, coilsSummary);
    }

    /// <summary>
    /// Activate a single coil: wait delay ? turn ON ? wait duration ? turn OFF.
    /// Runs on the consumer thread (blocking is acceptable here).
    /// </summary>
    private bool ActivateCoil(ushort coilAddress, int delayMs, int durationMs, out string? error)
    {
        try
        {
            if (delayMs > 0)
                Thread.Sleep(delayMs);

            if (!_modbus.WriteSingleCoil(coilAddress, true))
            {
                error = $"Write ON failed @{coilAddress}: {_modbus.LastError}";
                Debug.WriteLine($"[Trigger] {error}");
                return false;
            }

            Debug.WriteLine($"[Trigger] Coil {coilAddress} ON (delay={delayMs}ms)");

            if (durationMs > 0)
                Thread.Sleep(durationMs);

            if (!_modbus.WriteSingleCoil(coilAddress, false))
            {
                error = $"Write OFF failed @{coilAddress}: {_modbus.LastError}";
                Debug.WriteLine($"[Trigger] {error}");
                return false;
            }

            Debug.WriteLine($"[Trigger] Coil {coilAddress} OFF (held {durationMs}ms)");
            error = null;
            return true;
        }
        catch (Exception ex)
        {
            error = $"Coil {coilAddress} exception: {ex.Message}";
            Debug.WriteLine($"[Trigger] {error}");
            return false;
        }
    }

    public void Dispose()
    {
        Stop();

        try { _queue.Dispose(); } catch { }

        if (_ownsModbus)
        {
            _modbus.Dispose();
            Debug.WriteLine("[Trigger] Owned ModbusService disposed.");
        }
    }
}
