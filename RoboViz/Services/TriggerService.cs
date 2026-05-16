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
    private readonly OutputCommunicationService _outputs;
    private readonly CameraManager? _cameras;
    private readonly InspectionService? _inspection;
    private readonly TriggerConfig _config;
    private readonly CameraSlotConfig[] _cameraSlots;
    private readonly Action<TriggerResultEvent> _onResult;

    // One queue + one consumer per trigger group, so groups process in parallel.
    private readonly Dictionary<int, BlockingCollection<TriggerEvent>> _queues = [];
    private Thread?[] _producerThreads = [];
    private Thread?[] _consumerThreads = [];
    private volatile bool _running;

    public bool IsRunning => _running;

    /// <param name="outputs">Shared output transport for Modbus or HTTP writes.</param>
    /// <param name="cameras">CameraManager instance, or null if cameras not streaming.</param>
    /// <param name="inspection">InspectionService, or null if model not loaded.</param>
    /// <param name="config">Trigger configuration from file.</param>
    /// <param name="cameraSlots">Per-camera configuration (detector, trigger group, delay).</param>
    /// <param name="onResult">Callback invoked on the consumer thread with results (caller must marshal to UI).</param>
    /// <param name="onPollStatus">Optional callback invoked every poll cycle with read status (caller must marshal to UI).</param>
    public TriggerService(
        OutputCommunicationService outputs,
        CameraManager? cameras,
        InspectionService? inspection,
        TriggerConfig config,
        CameraSlotConfig[] cameraSlots,
        Action<TriggerResultEvent> onResult,
        Action<TriggerPollStatus>? onPollStatus = null)
    {
        _outputs = outputs;
        _cameras = cameras;
        _inspection = inspection;
        _config = config;
        _cameraSlots = cameraSlots;
        _onResult = onResult;
        _ = onPollStatus; // Poll callback retained in the public signature for backward compatibility.

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

            // Initial state of the per-group READY handshake = 1 (system is idle
            // and willing to accept the first trigger). The coil drops to 0 the
            // moment ProcessTrigger starts work for this group, and goes back to
            // 1 when the batch is done.
            SetReady(group, true);
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

        // Drop READY for all groups that we drove high — system is no longer
        // accepting triggers. Errors are swallowed: we may be tearing down
        // because Modbus is already dead.
        foreach (var group in _queues.Keys)
        {
            try { SetReady(group, false); } catch { }
        }

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

    /// <summary>
    /// Drive the per-group READY handshake coil.
    ///   • READY = 1  ? this trigger group is idle and willing to accept the
    ///                  next hardware trigger. The PLC ANDs both groups' coils
    ///                  to drive the operator buzzer / "place next part" lamp.
    ///   • READY = 0  ? a batch is currently being processed for this group.
    /// Coil address 0 disables the handshake for that group.
    /// Modbus serialisation is provided by ModbusService._busLock, so this can
    /// be called concurrently with rejection-coil writes without corrupting
    /// the RTU framing.
    /// </summary>
    private void SetReady(int triggerGroup, bool ready)
    {
        var channel = triggerGroup == 1 ? OutputChannel.ReadyT1 : OutputChannel.ReadyT2;

        // For HTTP mode: fire-and-forget so the consumer thread is never blocked
        // waiting for PCB response during camera streaming.
        // For Modbus mode: keep synchronous (bus lock already serialises writes).
        if (string.Equals(_config.CommunicationMode, "http", StringComparison.OrdinalIgnoreCase))
        {
            _ = Task.Run(() =>
            {
                if (!_outputs.Write(channel, ready))
                    Debug.WriteLine($"[Trigger] Ready_T{triggerGroup}={(ready ? 1 : 0)} HTTP write FAILED: {_outputs.LastError}");
                else
                    Debug.WriteLine($"[Trigger] Ready_T{triggerGroup}={(ready ? 1 : 0)} HTTP OK");
            });
            return;
        }

        if (!_outputs.Write(channel, ready))
        {
            Debug.WriteLine($"[Trigger] Ready_T{triggerGroup}={(ready ? 1 : 0)} write FAILED: {_outputs.LastError}");
        }
        else
        {
            Debug.WriteLine($"[Trigger] Ready_T{triggerGroup}={(ready ? 1 : 0)}");
        }
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
        Debug.WriteLine($"[Pipeline-G{triggerGroup}] STAGE 1 — Sentinel armed: CAM {sentinel.Slot + 1} (device {sentinel.DeviceIndex}), blocking on WaitForNewFrame...");

        while (_running)
        {
            // BLOCKING wait — returns when camera hardware trigger fires and a frame arrives.
            // 2 s internal timeout used only for clean shutdown; no spurious processing on timeout.
            var frame = _cameras?.WaitForNewFrame(sentinel.Slot, timeoutMs: 2000);

            if (frame == null)
            {
                Debug.WriteLine($"[Pipeline-G{triggerGroup}] STAGE 1 — WaitForNewFrame timeout (2000ms), _running={_running}, re-waiting...");
                continue;
            }

            frame.Dispose(); // full frames grabbed later in ProcessTrigger

            var arrivalTime = DateTime.Now;
            Debug.WriteLine($"[Pipeline-G{triggerGroup}] STAGE 1 — HW trigger fired, frame arrived at {arrivalTime:HH:mm:ss.fff}");

            try
            {
                if (_queues.TryGetValue(triggerGroup, out var queue))
                {
                    queue.Add(new TriggerEvent(triggerType, arrivalTime));
                    Debug.WriteLine($"[Pipeline-G{triggerGroup}] STAGE 1 — TriggerEvent enqueued at {DateTime.Now:HH:mm:ss.fff} (queue depth ~{queue.Count})");
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

        Debug.WriteLine($"[Pipeline-G{triggerGroup}] STAGE 2 — Event dequeued at {tDequeue:HH:mm:ss.fff} (queue latency: {queueLatencyMs:F1}ms)");
        Debug.WriteLine($"[Pipeline-G{triggerGroup}] STAGE 2 — Sending READY=0 (system busy)");
        SetReady(triggerGroup, false);

        try
        {
            ProcessTriggerCore(trigger, triggerGroup, label, tDequeue, queueLatencyMs);
        }
        finally
        {
            Debug.WriteLine($"[Pipeline-G{triggerGroup}] STAGE 6 — Batch complete, sending READY=1 (system idle)");
            SetReady(triggerGroup, true);
        }
    }

    private void ProcessTriggerCore(TriggerEvent trigger, int triggerGroup, string label,
        DateTime tDequeue, double queueLatencyMs)
    {
        Debug.WriteLine($"[Pipeline-G{triggerGroup}] STAGE 2 — ProcessTriggerCore entered (queue latency: {queueLatencyMs:F1}ms)");

        // Find camera slots assigned to this trigger group
        var slotsForTrigger = _cameraSlots
            .Where(c => c.TriggerGroup == triggerGroup && c.DeviceIndex >= 0)
            .OrderBy(c => c.CaptureDelayMs)
            .ToArray();

        Debug.WriteLine($"[Pipeline-G{triggerGroup}] STAGE 3 — Starting frame capture for {slotsForTrigger.Length} camera(s)");

        if (slotsForTrigger.Length == 0)
        {
            Debug.WriteLine($"[Pipeline-G{triggerGroup}] STAGE 3 — ABORT: no cameras assigned to this trigger group.");
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

        Debug.WriteLine($"[Pipeline-G{triggerGroup}] STAGE 3 — cameras={(_cameras == null ? "NULL" : "OK")}, inspection={(_inspection == null ? "NULL" : "OK")}, IsStreaming={_cameras?.IsStreaming}, IsHardwareTrigger={_cameras?.IsHardwareTrigger}");

        if (_cameras == null || _inspection == null)
        {
            string reason = _cameras == null && _inspection == null ? "cameras AND model null"
                : _cameras == null ? "cameras null"
                : "model null";
            Debug.WriteLine($"[Pipeline-G{triggerGroup}] STAGE 3 — ABORT: {reason}, skipping inference.");
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
            Debug.WriteLine($"[Pipeline-G{triggerGroup}] STAGE 3 — ABORT: {reason}.");
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
            Debug.WriteLine($"[Pipeline-G{triggerGroup}] STAGE 3 — CAM {slotCfg.Slot + 1}: elapsed={elapsed}ms, configured delay={slotCfg.CaptureDelayMs}ms, sleeping {Math.Max(0, remaining)}ms");
            if (remaining > 0)
                Thread.Sleep(remaining);

            // Grab latest frame with retry
            var swCapture = Stopwatch.StartNew();
            const int maxRetries = 10;
            const int retryDelayMs = 200;
            for (int attempt = 0; attempt < maxRetries; attempt++)
            {
                frames[i] = _cameras.GetLatestFrame(slotCfg.Slot);
                if (frames[i] != null)
                {
                    Debug.WriteLine($"[Pipeline-G{triggerGroup}] STAGE 3 — CAM {slotCfg.Slot + 1}: frame grabbed on attempt {attempt + 1}");
                    break;
                }
                Debug.WriteLine($"[Pipeline-G{triggerGroup}] STAGE 3 — CAM {slotCfg.Slot + 1}: GetLatestFrame null (attempt {attempt + 1}/{maxRetries}), retrying in {retryDelayMs}ms...");
                if (attempt < maxRetries - 1)
                    Thread.Sleep(retryDelayMs);
            }
            frameCaptureTimes[i] = swCapture.ElapsedMilliseconds;

            Debug.WriteLine($"[Pipeline-G{triggerGroup}] STAGE 3 — CAM {slotCfg.Slot + 1}: result={(frames[i] != null ? $"OK ({frames[i]!.Width}x{frames[i]!.Height})" : "NULL — no frame after all retries")} [{frameCaptureTimes[i]}ms]");
        }

        var totalCaptureMs = (DateTime.Now - tCaptureStart).TotalMilliseconds;
        Debug.WriteLine($"[Pipeline-G{triggerGroup}] STAGE 3 — All frames captured in {totalCaptureMs:F1}ms | per-cam: [{string.Join(", ", frameCaptureTimes.Select(t => $"{t}ms"))}]");

        // Run inference on captured frames.
        // Inferences run in PARALLEL across slots:
        //   • MaskRCNNDetector uses a ThreadLocal<float[]> input buffer and ORT's
        //     InferenceSession.Run is thread-safe.
        //   • OringMeasurement.Measure / MeasureCam2 / YoloContourDetector.Detect
        //     all use only locals + a read-only static background, so they are
        //     reentrant.
        // For T2 (3 cams) this collapses ~3×inference into ~max(inference) on
        // GPU, modulo CUDA-stream contention (~20–30%).
        var tInspStart = DateTime.Now;
        var sw = Stopwatch.StartNew();

        int n = slotsForTrigger.Length;
        var resultsByIdx = new InspectionResult?[n];
        var perCamTimes = new long[n];

        Debug.WriteLine($"[Pipeline-G{triggerGroup}] STAGE 4 — Starting parallel inference on {n} camera(s) at {tInspStart:HH:mm:ss.fff}");

        Parallel.For(0, n, i =>
        {
            if (frames[i] == null)
            {
                Debug.WriteLine($"[Pipeline-G{triggerGroup}] STAGE 4 — CAM {slotsForTrigger[i].Slot + 1}: SKIPPED (frame is null)");
                return;
            }
            var slotCfg = slotsForTrigger[i];

            Debug.WriteLine($"[Pipeline-G{triggerGroup}] STAGE 4 — CAM {slotCfg.Slot + 1}: inference START ({frames[i]!.Width}x{frames[i]!.Height}px, skipGeo={slotCfg.SkipGeoMeasurement})");
            var swInspect = Stopwatch.StartNew();
            var result = _inspection.InspectMaskRCNN(frames[i]!, triggerGroup,
                slotCfg.SkipGeoMeasurement, slot: slotCfg.Slot);
            long inspectMs = swInspect.ElapsedMilliseconds;

            perCamTimes[i] = inspectMs;
            resultsByIdx[i] = result;

            Debug.WriteLine($"[Pipeline-G{triggerGroup}] STAGE 4 — CAM {slotCfg.Slot + 1}: inference DONE in {inspectMs}ms => verdict={result.Verdict}" +
                $" | geo={result.GeoMs}ms infer={result.InferenceMs}ms | defect={result.HasDefect} topScore={result.TopScore:F3}" +
                (result.FailReasons.Count > 0 ? $" | reasons=[{string.Join(", ", result.FailReasons)}]" : "") +
                (result.ErrorMessage != null ? $" | ERROR: {result.ErrorMessage}" : ""));
        });

        // Re-assemble ordered output lists, skipping slots whose frame was null.
        var resultList = new System.Collections.Generic.List<InspectionResult>(n);
        var frameList = new System.Collections.Generic.List<Bitmap>(n);
        var slotList = new System.Collections.Generic.List<int>(n);
        for (int i = 0; i < n; i++)
        {
            if (resultsByIdx[i] == null) continue;
            resultList.Add(resultsByIdx[i]!);
            frameList.Add(frames[i]!);
            slotList.Add(slotsForTrigger[i].Slot);
        }

        var results = resultList.ToArray();
        long batchMs = sw.ElapsedMilliseconds;
        var totalInspMs = (DateTime.Now - tInspStart).TotalMilliseconds;
        long sumMs = perCamTimes.Sum();
        long maxMs = n > 0 ? perCamTimes.Max() : 0;
        double speedup = maxMs > 0 ? (double)sumMs / maxMs : 0;
        Debug.WriteLine($"[Pipeline-G{triggerGroup}] STAGE 4 — Inference complete: wall={totalInspMs:F1}ms batch={batchMs}ms | sequential-equiv={sumMs}ms | speedup={speedup:F1}x");
        Debug.WriteLine($"[Pipeline-G{triggerGroup}] STAGE 4 — Verdicts: [{string.Join(", ", results.Select(r => $"CAM{slotList[Array.IndexOf(results, r)] + 1}={r.Verdict}"))}]");

        // Dispose frames not used as overlay
        for (int i = 0; i < frameList.Count; i++)
        {
            if (!ReferenceEquals(frameList[i], results[i].OverlayImage))
                frameList[i].Dispose();
        }

        Debug.WriteLine($"[Pipeline-G{triggerGroup}] STAGE 5 — Posting {results.Length} result(s) to UI");
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
        Debug.WriteLine($"[Pipeline-G{triggerGroup}] STAGE 5 — UI callback done, firing rejection coil writes (fire-and-forget)");

        int capturedGroup = triggerGroup;
        var capturedResults = results;
        var capturedSlots = slotList.ToList();
        _ = Task.Run(() =>
        {
            Debug.WriteLine($"[Pipeline-G{capturedGroup}] STAGE 5 — WriteRejectionCoils starting");
            var (ok, err, coils) = WriteRejectionCoils(capturedGroup, capturedResults, capturedSlots);
            Debug.WriteLine($"[Pipeline-G{capturedGroup}] STAGE 5 — WriteRejectionCoils done: activated=[{coils ?? "none"}], ok={ok}, err={err ?? "no coil needed"}");
        });
    }

    /// <summary>
    /// Evaluate verdicts and write the appropriate rejection/rework coils.
    /// Each coil is turned ON after its configured delay, held for CoilOnDurationMs, then turned OFF.
    /// </summary>
    private (bool ok, string? error, string? coilsActivated) WriteRejectionCoils(
        int triggerGroup, InspectionResult[] results, System.Collections.Generic.List<int> slotList)
    {
        var activated = new System.Collections.Generic.List<string>();
        bool allOk = true;
        string? lastError = null;

        // Build slot?verdict map
        var verdictBySlot = new System.Collections.Generic.Dictionary<int, string>();
        for (int i = 0; i < results.Length && i < slotList.Count; i++)
            verdictBySlot[slotList[i]] = results[i].Verdict;

        bool Activate(OutputChannel channel, int delayMs, int durationMs, string label, out string? err)
        {
            try
            {
                if (delayMs > 0)
                    Thread.Sleep(delayMs);

                if (!_outputs.Write(channel, true))
                {
                    err = $"Write ON failed for {label}: {_outputs.LastError}";
                    Debug.WriteLine($"[Trigger] {err}");
                    return false;
                }

                Debug.WriteLine($"[Trigger] {label} ON (delay={delayMs}ms)");

                if (durationMs > 0)
                    Thread.Sleep(durationMs);

                if (!_outputs.Write(channel, false))
                {
                    err = $"Write OFF failed for {label}: {_outputs.LastError}";
                    Debug.WriteLine($"[Trigger] {err}");
                    return false;
                }

                Debug.WriteLine($"[Trigger] {label} OFF (held {durationMs}ms)");
                err = null;
                return true;
            }
            catch (Exception ex)
            {
                err = $"{label} exception: {ex.Message}";
                Debug.WriteLine($"[Trigger] {err}");
                return false;
            }
        }

        if (triggerGroup == 1)
        {
            verdictBySlot.TryGetValue(0, out string? cam1Verdict);

            if (cam1Verdict == "REWORK")
            {
                if (Activate(OutputChannel.Cam1Rework, _config.OutputCoils.Cam1_ReworkDelayMs, _config.OutputCoils.CoilOnDurationMs, "Cam1_Rework", out string? err))
                    activated.Add($"Cam1_Rework@{_config.OutputCoils.Cam1_ReworkCoil}");
                else { allOk = false; lastError = err; }
            }
            else if (cam1Verdict == "REJECT")
            {
                if (Activate(OutputChannel.Cam1Reject, _config.OutputCoils.Cam1_RejectDelayMs, _config.OutputCoils.CoilOnDurationMs, "Cam1_Reject", out string? err))
                    activated.Add($"Cam1_Reject@{_config.OutputCoils.Cam1_RejectCoil}");
                else { allOk = false; lastError = err; }
            }
        }
        else if (triggerGroup == 2)
        {
            verdictBySlot.TryGetValue(1, out string? cam2Verdict);
            verdictBySlot.TryGetValue(2, out string? cam3Verdict);
            verdictBySlot.TryGetValue(3, out string? cam4Verdict);

            bool anyReject = cam2Verdict == "REJECT" || cam3Verdict == "REJECT" || cam4Verdict == "REJECT";
            bool cam2Rework = cam2Verdict == "REWORK";
            bool rejectPriority = !string.Equals(_config.OutputCoils.ConflictPriority, "rework", StringComparison.OrdinalIgnoreCase);

            if (rejectPriority)
            {
                if (anyReject)
                {
                    if (Activate(OutputChannel.Cam234Reject, _config.OutputCoils.Cam234_RejectDelayMs, _config.OutputCoils.CoilOnDurationMs, "Cam234_Reject", out string? err))
                        activated.Add($"Cam234_Reject@{_config.OutputCoils.Cam234_RejectCoil}");
                    else { allOk = false; lastError = err; }
                }
                else if (cam2Rework)
                {
                    if (Activate(OutputChannel.Cam2Rework, _config.OutputCoils.Cam2_ReworkDelayMs, _config.OutputCoils.CoilOnDurationMs, "Cam2_Rework", out string? err))
                        activated.Add($"Cam2_Rework@{_config.OutputCoils.Cam2_ReworkCoil}");
                    else { allOk = false; lastError = err; }
                }
            }
            else
            {
                if (anyReject)
                {
                    if (Activate(OutputChannel.Cam234Reject, _config.OutputCoils.Cam234_RejectDelayMs, _config.OutputCoils.CoilOnDurationMs, "Cam234_Reject", out string? err))
                        activated.Add($"Cam234_Reject@{_config.OutputCoils.Cam234_RejectCoil}");
                    else { allOk = false; lastError = err; }
                }
                if (cam2Rework)
                {
                    if (Activate(OutputChannel.Cam2Rework, _config.OutputCoils.Cam2_ReworkDelayMs, _config.OutputCoils.CoilOnDurationMs, "Cam2_Rework", out string? err))
                        activated.Add($"Cam2_Rework@{_config.OutputCoils.Cam2_ReworkCoil}");
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

    public void Dispose()
    {
        Stop();

        foreach (var q in _queues.Values)
        {
            try { q.Dispose(); } catch { }
        }
        _queues.Clear();
    }
}
