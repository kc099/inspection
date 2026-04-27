using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Threading;

namespace RoboViz;

/// <summary>
/// Hardware-trigger producer-consumer pipeline.
///   Producer: one sentinel waiter per trigger group blocks on camera frame arrival.
///   Consumer: captures grouped frames, runs inference, posts results, and schedules output coils.
///   Supports flexible camera-to-trigger mapping via <see cref="CameraSlotConfig"/>.
/// </summary>
public class TriggerService : IDisposable
{
    private readonly ModbusService _modbus;
    private readonly CameraManager? _cameras;
    private readonly InspectionService? _inspection;
    private readonly TriggerConfig _config;
    private readonly CameraSlotConfig[] _cameraSlots;
    private readonly Action<TriggerResultEvent> _onResult;
    private readonly bool _ownsModbus;

    // One queue + one consumer per trigger group, so groups process in parallel.
    private readonly Dictionary<int, BlockingCollection<TriggerEvent>> _queues = [];
    private Thread?[] _producerThreads = [];
    private Thread?[] _consumerThreads = [];
    private volatile bool _running;

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
        _ = onPollStatus; // Poll callback retained in the public signature for backward compatibility.
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

        if (_cameras == null)
        {
            Debug.WriteLine("[TriggerService] No cameras — trigger events will be empty.");
            MaskRCNNDetector.LogDiag("[Trigger] Started (no cameras).");
            return;
        }

        // One sentinel waiter + one consumer per trigger group (parallel pipelines).
        var groups = _cameraSlots
            .Where(c => c.DeviceIndex >= 0)
            .Select(c => c.TriggerGroup)
            .Distinct()
            .OrderBy(g => g)
            .ToArray();

        _producerThreads = new Thread[groups.Length];
        _consumerThreads = new Thread[groups.Length];
        for (int i = 0; i < groups.Length; i++)
        {
            int group = groups[i];
            var queue = new BlockingCollection<TriggerEvent>(boundedCapacity: 16);
            _queues[group] = queue;

            _consumerThreads[i] = new Thread(() => ConsumerLoop(queue))
            {
                IsBackground = true,
                Name = $"TriggerConsumer_G{group}"
            };
            _consumerThreads[i].Start();

            _producerThreads[i] = new Thread(() => SentinelWaiterLoop(group))
            {
                IsBackground = true,
                Name = $"TriggerProducer_G{group}"
            };
            _producerThreads[i].Start();
            Debug.WriteLine($"[Trigger] Producer + consumer started for trigger group {group}.");
        }

        MaskRCNNDetector.LogDiag($"[Trigger] Started: {groups.Length} parallel pipeline(s) [hardware trigger mode].");
    }

    /// <summary>
    /// Redirect to Start() — producer-consumer now handles camera hardware triggers.
    /// Kept for backward compatibility.
    /// </summary>
    public void StartHardwareMode()
    {
        Debug.WriteLine("[Trigger] StartHardwareMode() ? redirecting to Start().");
        Start();
    }

    public void Stop()
    {
        if (!_running) return;
        _running = false;

        foreach (var q in _queues.Values)
        {
            try { q.CompleteAdding(); } catch { }
        }

        var producers = _producerThreads;
        var consumers = _consumerThreads;
        _producerThreads = [];
        _consumerThreads = [];

        _ = Task.Run(() =>
        {
            foreach (var p in producers)
                p?.Join(3000);
            foreach (var c in consumers)
                c?.Join(3000);
            Debug.WriteLine("[Trigger] All background threads joined.");
        });

        MaskRCNNDetector.LogDiag("[Trigger] Stopped.");
    }

    // ?? Sentinel Waiter (Producer) ??????????????????????????????????????????????????????????????

    /// <summary>
    /// One thread per trigger group. Blocks on WaitForNewFrame for the sentinel camera.
    /// When the 24 V signal arrives on the camera's opto-in, the camera captures a frame;
    /// this thread unblocks, records the arrival timestamp, and enqueues a TriggerEvent.
    /// </summary>
    private void SentinelWaiterLoop(int triggerGroup)
    {
        var sentinel = _cameraSlots
            .Where(c => c.TriggerGroup == triggerGroup && c.DeviceIndex >= 0)
            .OrderBy(c => c.Slot)
            .FirstOrDefault();

        if (sentinel == null)
        {
            Debug.WriteLine($"[Trigger] Group {triggerGroup}: no cameras assigned, waiter exiting.");
            return;
        }

        var triggerType = triggerGroup == 1 ? TriggerType.Trigger1 : TriggerType.Trigger2;
        Debug.WriteLine($"[Trigger] Group {triggerGroup} sentinel: CAM {sentinel.Slot + 1} (device {sentinel.DeviceIndex})");

        while (_running)
        {
            // BLOCKING wait — returns when camera hardware trigger fires and a frame arrives.
            // 2 s internal timeout used only for clean shutdown; no spurious processing on timeout.
            var frame = _cameras?.WaitForNewFrame(sentinel.Slot, timeoutMs: 2000);

            if (frame == null) continue; // timeout — check _running and wait again

            frame.Dispose(); // full frames grabbed later in ProcessTrigger

            var arrivalTime = DateTime.Now;
            Debug.WriteLine($"[Profiling] Group {triggerGroup} CAM {sentinel.Slot + 1}: frame arrived {arrivalTime:HH:mm:ss.fff}");

            try
            {
                if (_queues.TryGetValue(triggerGroup, out var queue))
                {
                    queue.Add(new TriggerEvent(triggerType, arrivalTime));
                    Debug.WriteLine($"[Profiling] Group {triggerGroup}: TriggerEvent queued at {DateTime.Now:HH:mm:ss.fff}");
                }
            }
            catch (InvalidOperationException)
            {
                break; // queue completed — shutting down
            }
        }

        Debug.WriteLine($"[Trigger] Sentinel waiter for group {triggerGroup} stopped.");
    }

    // ?? Consumer ????????????????????????????????????????????????????????????????????????????????

    private void ConsumerLoop(BlockingCollection<TriggerEvent> queue)
    {
        foreach (var trigger in queue.GetConsumingEnumerable())
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
        var tDequeue = DateTime.Now;
        double queueLatencyMs = (tDequeue - trigger.Timestamp).TotalMilliseconds;
        int triggerGroup = trigger.Type == TriggerType.Trigger1 ? 1 : 2;
        string label = $"Trigger {triggerGroup}";

        Debug.WriteLine($"[Profiling] ???? {label} dequeued {tDequeue:HH:mm:ss.fff}  (queue latency: {queueLatencyMs:F1} ms) ????");

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

        if (!_cameras.IsStreaming || !_cameras.IsHardwareTrigger)
        {
            string reason = !_cameras.IsStreaming
                ? "cameras are not armed"
                : "cameras are not in hardware-trigger mode";
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

        // Capture latest hardware-triggered frames with per-camera delays (sorted ascending).
        var triggerTime = trigger.Timestamp;
        var tCaptureStart = DateTime.Now;
        var frames = new Bitmap?[slotsForTrigger.Length];
        var frameCaptureTimes = new long[slotsForTrigger.Length];

        for (int i = 0; i < slotsForTrigger.Length; i++)
        {
            var slotCfg = slotsForTrigger[i];

            // Wait until this camera's delay has elapsed since trigger
            int elapsed = (int)(DateTime.Now - triggerTime).TotalMilliseconds;
            int remaining = slotCfg.CaptureDelayMs - elapsed;
            if (remaining > 0)
                Thread.Sleep(remaining);

            // Grab latest frame with retry
            var swCapture = Stopwatch.StartNew();
            const int maxRetries = 10;
            const int retryDelayMs = 200;
            for (int attempt = 0; attempt < maxRetries; attempt++)
            {
                frames[i] = _cameras.GetLatestFrame(slotCfg.Slot);
                if (frames[i] != null) break;
                if (attempt < maxRetries - 1)
                    Thread.Sleep(retryDelayMs);
            }
            frameCaptureTimes[i] = swCapture.ElapsedMilliseconds;

            Debug.WriteLine($"[Trigger] CAM {slotCfg.Slot + 1} (delay {slotCfg.CaptureDelayMs}ms): {(frames[i] != null ? "OK" : "NULL")} [{frameCaptureTimes[i]}ms]");
        }

        var totalCaptureMs = (DateTime.Now - tCaptureStart).TotalMilliseconds;
        Debug.WriteLine($"[Profiling] Frame capture done: {totalCaptureMs:F1}ms | per-cam: [{string.Join(", ", frameCaptureTimes.Select(t => $"{t}ms"))}]");

        // Run inference on captured frames
        // NOTE: Sequential execution means total time = sum of all inspection times.
        // CAM 2 geometry is slower than CAM 1 because BuildMaskCam2 uses:
        //   - Ratio normalization (floating-point division per pixel)
        //   - GaussianBlur 21×21 kernel
        //   - AdaptiveThreshold 151×151 block (VERY expensive: ~23K pixels per output pixel)
        //   - Morphology: 3× Close(35×35) + 1× Open(15×15) vs CAM 1's 2× Close(7×7) + 1× Open(7×7)
        //   - Connected components with circularity scoring + hole filling
        // Typical CAM 1 geo: ~100-120ms | CAM 2 geo: ~1400-1600ms
        var tInspStart = DateTime.Now;
        var resultList = new System.Collections.Generic.List<InspectionResult>();
        var frameList = new System.Collections.Generic.List<Bitmap>();
        var slotList = new System.Collections.Generic.List<int>();
        var sw = Stopwatch.StartNew();
        var perCamTimes = new System.Collections.Generic.List<long>();

        for (int i = 0; i < slotsForTrigger.Length; i++)
        {
            if (frames[i] == null) continue;
            var slotCfg = slotsForTrigger[i];

            Debug.WriteLine($"[Trigger] CAM {slotCfg.Slot + 1} frame: {frames[i]!.Width}x{frames[i]!.Height} px, detector={slotCfg.Detector}");
            var swInspect = Stopwatch.StartNew();
            bool skipGeo = slotCfg.SkipGeoMeasurement;
            var result = _inspection.InspectMaskRCNN(frames[i]!, triggerGroup, skipGeo, slot: slotCfg.Slot);
            long inspectMs = swInspect.ElapsedMilliseconds;
            perCamTimes.Add(inspectMs);
            resultList.Add(result);
            frameList.Add(frames[i]!);
            slotList.Add(slotCfg.Slot);

            Debug.WriteLine($"[Trigger] CAM {slotCfg.Slot + 1} => {result.Verdict} | " +
                $"total={result.TotalMs}ms geo={result.GeoMs}ms infer={result.InferenceMs}ms (wall: {inspectMs}ms)" +
                $" | defect={result.HasDefect} topScore={result.TopScore:F3}" +
                (result.FailReasons.Count > 0 ? $" | reasons=[{string.Join(", ", result.FailReasons)}]" : "") +
                (result.ErrorMessage != null ? $" | ERROR: {result.ErrorMessage}" : ""));
        }

        var results = resultList.ToArray();
        long batchMs = sw.ElapsedMilliseconds;
        var totalInspMs = (DateTime.Now - tInspStart).TotalMilliseconds;
        Debug.WriteLine($"[Profiling] Inspection done: {totalInspMs:F1}ms (batch timer: {batchMs}ms) | per-cam: [{string.Join(", ", perCamTimes.Select(t => $"{t}ms"))}]");
        Debug.WriteLine($"[Profiling] Sequential sum: {perCamTimes.Sum()}ms | parallel potential: max({string.Join(", ", perCamTimes)}) = {(perCamTimes.Count > 0 ? perCamTimes.Max() : 0)}ms (speedup: {(perCamTimes.Sum() > 0 ? (double)perCamTimes.Sum() / (perCamTimes.Count > 0 ? perCamTimes.Max() : 1) : 0):F1}x)");
        Debug.WriteLine($"[Trigger] {label} batch complete: {batchMs}ms | verdicts=[{string.Join(", ", results.Select(r => r.Verdict))}]");

        // Dispose frames not used as overlay
        for (int i = 0; i < frameList.Count; i++)
        {
            if (!ReferenceEquals(frameList[i], results[i].OverlayImage))
                frameList[i].Dispose();
        }

        // Send results to UI immediately — don't block the consumer on Modbus coil writes
        _onResult(new TriggerResultEvent
        {
            Type = trigger.Type,
            Results = results,
            Slots = [.. slotList],
            BatchMs = batchMs,
            ModbusWriteOk = true,
            ModbusError = null,
            CoilsActivated = null,
        });

        // Write rejection output coils fire-and-forget so consumer is never blocked
        // by Modbus timeouts (~1.5 s per timeout would starve the trigger queue)
        int capturedGroup = triggerGroup;
        var capturedResults = results;
        var capturedSlots = slotList.ToList();
        _ = Task.Run(() =>
        {
            var (ok, err, coils) = WriteRejectionCoils(capturedGroup, capturedResults, capturedSlots);
            Debug.WriteLine($"[Trigger] Coils: group={capturedGroup}, activated=[{coils ?? "none"}], ok={ok}, err={err ?? "no coil needed"}");
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

        foreach (var q in _queues.Values)
        {
            try { q.Dispose(); } catch { }
        }
        _queues.Clear();

        if (_ownsModbus)
        {
            _modbus.Dispose();
            Debug.WriteLine("[Trigger] Owned ModbusService disposed.");
        }
    }
}
