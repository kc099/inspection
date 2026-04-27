# RoboViz — Hardware Trigger Migration: Deployment Reference Guide

**Date:** 2025  
**Purpose:** Replaces Modbus input polling with camera hardware trigger (opto-in Line0).  
**For:** Self-debugging at deployment site without internet access.

---

## Table of Contents

1. [What Changed and Why](#1-what-changed-and-why)
2. [File-by-File Change Details](#2-file-by-file-change-details)
3. [Complete Data Flow](#3-complete-data-flow)
4. [Deployment Troubleshooting](#4-deployment-troubleshooting)
5. [Debug Output Reference](#5-debug-output-reference)
6. [Key Configuration Values](#6-key-configuration-values)

---

## 1. What Changed and Why

### Before (Old System)
```
24V Switch ON
    ?
Modbus coil 3001/3002 goes HIGH
    ?
ProducerLoop reads coil every 200ms  ? polling, adds delay
    ?
Edge detected (LOW?HIGH) ? queue trigger event
    ?
Consumer grabs camera frame
    ?
Run inspection
    ?
Write output coils (BLOCKED consumer for ~1.5s on timeout)
    ?
UI updated
```

### After (New System)
```
24V Switch ON
    ?
Camera opto-in (Line0) receives signal directly
    ? (TriggerDelay = 50µs inside camera hardware)
Camera captures frame
    ?
SentinelWaiterLoop unblocks (was blocking on WaitForNewFrame)
    ?
Queue trigger event ? Consumer processes
    ?
Run inspection (~100ms)
    ?
UI updated IMMEDIATELY
    ? (parallel, background thread)
Write output coils (never blocks consumer)
```

### Key Benefits
- No Modbus polling latency (was up to 200ms delay)
- Consumer never blocked by Modbus timeouts
- Triggers preserved in FIFO queue even if multiple arrive quickly
- Each trigger processed in order regardless of timing

---

## 2. File-by-File Change Details

---

### File 1: `RoboViz\Models\CameraSlotConfig.cs`

**Change:** Added one new property `TriggerDelayUs`

```csharp
// ADDED — hardware delay inside camera before exposure starts
public double TriggerDelayUs { get; set; } = 50.0;
// Unit: microseconds
// Default: 50µs
// Meaning: after 24V signal arrives on opto-in,
//          camera waits 50µs THEN captures frame
// Can be changed per-camera in camera_slots.json
```

**Existing property kept (same as before):**
```csharp
public string TriggerSource { get; set; } = "Line0";
// Line0 = opto-in connector on camera
// Change to "Line1", "Line2" etc. if wired differently
```

---

### File 2: `RoboViz\Services\CameraManager.cs`

#### Change A — `StartHardwareTrigger()`: added RisingEdge and TriggerDelay

```csharp
// OLD (missing TriggerActivation and TriggerDelay):
device.Parameters.SetEnumValueByString("TriggerMode", "On");
device.Parameters.SetEnumValueByString("TriggerSource", "Line0");

// NEW (complete hardware trigger setup):
device.Parameters.SetEnumValueByString("AcquisitionMode", "Continuous");
device.Parameters.SetEnumValueByString("TriggerMode", "On");
device.Parameters.SetEnumValueByString("TriggerSource", trigSrc);         // from config (default "Line0")
device.Parameters.SetEnumValueByString("TriggerActivation", "RisingEdge"); // 24V LOW?HIGH fires camera
device.Parameters.SetFloatValue("TriggerDelay", (float)trigDelayUs);       // 50µs default
```

**Why:** Camera was missing `TriggerActivation=RisingEdge`. Without it, the camera may not respond to the opto-in signal correctly.

#### Change B — `WaitForNewFrame()`: supports infinite timeout

```csharp
// OLD: always had a timeout, could not block forever
_frameArrived[slot].Wait(timeoutMs);

// NEW: -1 (Timeout.Infinite) blocks until frame arrives, no spurious wakeups
bool arrived = timeoutMs == Timeout.Infinite
    ? _frameArrived[slot].Wait(Timeout.Infinite)
    : _frameArrived[slot].Wait(timeoutMs);
```

**Why:** SentinelWaiterLoop needs to block indefinitely waiting for camera signal. The 2000ms timeout used in practice is only for clean shutdown checking, not for production logic.

#### Change C — `Stream_Click` and `StartCameraStreamAsync` in `MainWindow.xaml.cs`

```csharp
// OLD: started cameras in free-run continuous mode (frames every ~100ms regardless)
_cameraManager.StartStreaming(progress);

// NEW: starts cameras in hardware trigger mode (frames ONLY on 24V signal)
_cameraManager.StartHardwareTrigger(_cameraSlots, progress);
```

**Why:** This was the main bug that caused continuous frame flooding. The camera must be armed in hardware trigger mode from the start.

---

### File 3: `RoboViz\Services\TriggerService.cs`

#### Change A — Fields: replaced single producer thread with array

```csharp
// OLD:
private Thread? _producerThread;    // single thread polling Modbus
private bool _prevTrigger1;         // edge detection state
private bool _prevTrigger2;         // edge detection state

// NEW:
private Thread?[] _producerThreads = [];  // one thread per trigger group
// _prevTrigger1 and _prevTrigger2 REMOVED (camera hardware handles edge detection)
```

#### Change B — `Start()` method: spawns sentinel waiter threads

```csharp
// OLD Start(): one ProducerLoop thread polling Modbus every 200ms
_producerThread = new Thread(ProducerLoop);
_producerThread.Start();

// NEW Start(): one SentinelWaiterLoop thread per trigger group
// Each thread BLOCKS on camera frame arrival (no polling)
var groups = _cameraSlots
    .Where(c => c.DeviceIndex >= 0)
    .Select(c => c.TriggerGroup)
    .Distinct()
    .OrderBy(g => g)
    .ToArray();

_producerThreads = new Thread[groups.Length];
for (int i = 0; i < groups.Length; i++)
{
    int group = groups[i];
    _producerThreads[i] = new Thread(() => SentinelWaiterLoop(group));
    _producerThreads[i].Start();
}
// Thread names visible in debugger: "TriggerProducer_G1", "TriggerProducer_G2"
```

#### Change C — `ProducerLoop` REMOVED, replaced with `SentinelWaiterLoop`

```csharp
// OLD ProducerLoop (REMOVED):
private void ProducerLoop()
{
    while (_running)
    {
        var bits = _modbus.ReadCoils(addr, 1);  // poll Modbus every 200ms
        if (t1Now && !_prevTrigger1)            // software edge detection
            _queue.Add(TriggerEvent);
        Thread.Sleep(200);                       // polling interval
    }
}

// NEW SentinelWaiterLoop (REPLACEMENT):
private void SentinelWaiterLoop(int triggerGroup)
{
    // "sentinel" = first camera assigned to this trigger group
    var sentinel = _cameraSlots
        .Where(c => c.TriggerGroup == triggerGroup && c.DeviceIndex >= 0)
        .OrderBy(c => c.Slot)
        .FirstOrDefault();

    while (_running)
    {
        // BLOCKS HERE — no polling, no CPU usage while waiting
        // Returns only when camera hardware trigger fires and delivers a frame
        // 2000ms timeout is ONLY for clean shutdown — not a poll interval
        var frame = _cameras.WaitForNewFrame(sentinel.Slot, timeoutMs: 2000);

        if (frame == null) continue;  // timeout ? check _running, wait again

        frame.Dispose();              // sentinel copy discarded
                                      // real frame grabbed later in ProcessTrigger

        var arrivalTime = DateTime.Now;
        _queue.Add(new TriggerEvent(triggerType, arrivalTime));
        // Consumer will pick this up and process it
    }
}
```

**Sentinel concept:** Each trigger group has a "sentinel" camera — the first camera assigned to that group (by slot number). When the sentinel detects a frame (= hardware trigger fired), it queues an event. The consumer then grabs frames from ALL cameras in that group.

#### Change D — `StartHardwareMode()` simplified

```csharp
// OLD: had its own separate HardwareWatcherLoop threads, bypassed producer-consumer queue
public void StartHardwareMode()
{
    // spawned HardwareWatcherLoop threads (separate from consumer)
    // did NOT use the BlockingCollection queue
}

// NEW: just calls Start() — everything goes through producer-consumer queue
public void StartHardwareMode()
{
    Start(); // ? unified, same queue, same consumer
}
```

#### Change E — `ProcessTrigger()`: DateTime timezone fix

```csharp
// OLD (BUG): mixed UTC and local time
// For UTC+5:30 (India): elapsed = -19,800,000ms ? sleep for 5.5 hours ? hangs forever
int elapsed = (int)(DateTime.UtcNow - triggerTime).TotalMilliseconds;
//                   ? UTC              ? Local time = WRONG KIND mismatch

// NEW (FIXED): both use local time consistently
int elapsed = (int)(DateTime.Now - triggerTime).TotalMilliseconds;
//                   ? Local       ? Local time = CORRECT
```

**Why this mattered:** `SentinelWaiterLoop` stores `arrivalTime = DateTime.Now` (local time). When `ProcessTrigger` subtracted `DateTime.UtcNow` from that local time value, the tick difference was wrong by the UTC offset (5.5 hours in India = ~19.8 million milliseconds). This caused `Thread.Sleep(19800000)` — a 5.5 hour sleep — making the pipeline never complete.

#### Change F — `ProcessTrigger()`: coil writes moved to background Task

```csharp
// OLD: WriteRejectionCoils ran SYNCHRONOUSLY in consumer thread
// Modbus timeout (~1.5s) blocked consumer, delayed next trigger by 1.5s
var (ok, err, coils) = WriteRejectionCoils(triggerGroup, results, slotList);
_onResult(...);  // UI updated 1.5 seconds late

// NEW: UI updated immediately, coil writes run in background
_onResult(new TriggerResultEvent
{
    Type = trigger.Type,
    Results = results,
    Slots = [.. slotList],
    BatchMs = batchMs,
    ModbusWriteOk = true,   // coil status not tracked in real-time
    ModbusError = null,
    CoilsActivated = null,
});

// Fire-and-forget: consumer not blocked even if Modbus times out
int capturedGroup = triggerGroup;
var capturedResults = results;
var capturedSlots = slotList.ToList();
_ = Task.Run(() =>
{
    var (ok, err, coils) = WriteRejectionCoils(capturedGroup, capturedResults, capturedSlots);
    Debug.WriteLine($"[Trigger] Coils: group={capturedGroup}, activated=[{coils ?? "none"}], ok={ok}");
});
```

#### Change G — Profiling logs added to `ProcessTrigger`

These `Debug.WriteLine` lines are visible in Visual Studio Output window (Debug tab):

```
[Profiling] Group 1 CAM 1: frame arrived 18:06:43.258       ? sentinel detected frame
[Profiling] Group 1: TriggerEvent queued at 18:06:43.260    ? added to queue
[Profiling] ??? Trigger 1 dequeued 18:06:43.282 (queue latency: 23.8 ms) ???
[Trigger] Processing Trigger 1 (1 cameras) at 18:06:43.283
[Trigger] CAM 1 (delay 0ms): OK                             ? frame grabbed
[Profiling] Frame capture done: 25.6 ms
[Trigger] CAM 1 frame: 2448x2048 px, detector=MaskRCNN
[Trigger] CAM 1 [MaskRCNN] => REWORK | total=116ms geo=85ms infer=0ms
[Profiling] Inspection done: 119.8 ms
[Trigger] Trigger 1 batch complete: 119ms | verdicts=[REWORK]
[Trigger] Coils: group=1, activated=[...], ok=True
```

---

### File 4: `RoboViz\Views\MainWindow.xaml.cs`

#### Change — `StartTriggerMode()` rewritten

```csharp
// OLD:
// - REQUIRED Modbus connection, threw exception if COM port not found
// - Had if/else for "hardware" vs "software" mode from trigger_config.json
// - If hardware mode: called StartHardwareTrigger() + StartHardwareMode()
// - If software mode: called Start() with Modbus polling

// NEW:
// 1. Try Modbus connect for OUTPUT coils only (optional, won't throw)
var availablePorts = ModbusService.GetAvailablePorts();
if (availablePorts.Contains(config.ComPort))
{
    modbusOk = triggerModbus.Connect(config.ComPort, config.BaudRate, config.SlaveId);
    // if fails ? Debug.WriteLine, continues WITHOUT Modbus
}

// 2. Always start cameras in hardware trigger mode
if (_cameraManager != null && !_cameraManager.IsStreaming)
{
    _cameraManager.StartHardwareTrigger(_cameraSlots);
    _isStreaming = true;
}

// 3. Always call Start() — no more mode branching
_triggerService.Start();
// Start() internally uses SentinelWaiterLoop (camera-based, not Modbus)
```

---

## 3. Complete Data Flow

```
HARDWARE SIDE:
  Conveyor sensor / 24V switch
      ?
      ? 24V signal on opto-in wire
      ?
  Camera Line0 (opto-in connector)
      ?
      ? TriggerDelay = 50µs (inside camera hardware)
      ?
  Camera captures frame (exposure + readout)
      ?
      ? USB/GigE transfer to PC
      ?

SOFTWARE SIDE (CameraManager):
  GrabThreadProc (thread: "CamGrab_HW_0"):
      GetImageBuffer(5000ms timeout) ? blocks until frame arrives
      ? stores frame in _latestFrames[0]
      ? increments _frameSequence[0]
      ? signals _frameArrived[0].Set()
      ?
      ?

TriggerService PRODUCER:
  SentinelWaiterLoop (thread: "TriggerProducer_G1"):
      WaitForNewFrame(slot=0, timeout=2000ms) ? blocks
      ? _frameArrived[0].Wait() unblocks
      ? records arrivalTime = DateTime.Now
      ? frame.Dispose() (sentinel copy)
      ? _queue.Add(TriggerEvent{type=Trigger1, timestamp=arrivalTime})
      ? loops back to WaitForNewFrame()
      ?
      ?

BlockingCollection<TriggerEvent> _queue (capacity=16, FIFO)
      ?
      ?

TriggerService CONSUMER:
  ConsumerLoop (thread: "TriggerConsumer"):
      _queue.TryTake() ? blocks if queue empty
      ? calls ProcessTrigger(triggerEvent)
            ?
            ?? find slotsForTrigger (cameras in group 1 with DeviceIndex >= 0)
            ?? check _cameras.IsStreaming
            ?? Thread.Sleep(CaptureDelayMs) if > 0  ? default 0ms
            ?? GetLatestFrame(slot) from _latestFrames[0]
            ?? InspectMaskRCNN() ? verdict (PASS/REWORK/REJECT/ERROR)
            ?? _onResult(TriggerResultEvent) ? Dispatcher ? UI updated
            ?? Task.Run(WriteRejectionCoils) ? background thread
      ? loops back to _queue.TryTake()
      ?
      ?

UI THREAD (Dispatcher):
  OnTriggerResult():
      ? shows verdict on camera tile
      ? updates TriggerStatusText
      ? updates metrics table

BACKGROUND TASK (coil write):
  WriteRejectionCoils():
      ? Thread.Sleep(delayMs)
      ? _modbus.WriteSingleCoil(address, true)
      ? Thread.Sleep(durationMs)
      ? _modbus.WriteSingleCoil(address, false)
      ? Debug.WriteLine result
```

---

## 4. Deployment Troubleshooting

### Problem: Frames arriving continuously (not just on trigger)

**Symptom in Debug Output:**
```
[Profiling] Group 1 CAM 1: frame arrived 18:00:00.100
[Profiling] Group 1 CAM 1: frame arrived 18:00:00.200
[Profiling] Group 1 CAM 1: frame arrived 18:00:00.300
? frames every 100ms without any switch press
```

**Cause:** Camera started in continuous free-run mode, not hardware trigger mode.

**How to check:** Look for this line at startup:
```
[CameraManager] CAM 1: TriggerMode=On, Source=Line0, Activation=RisingEdge, Delay=50µs
```
If you see `[CAM 1: streaming]` instead, camera is in continuous mode.

**Root cause in code:** `StartStreaming()` was called instead of `StartHardwareTrigger()`.  
**File:** `MainWindow.xaml.cs` ? `Stream_Click()` and `StartCameraStreamAsync()`  
**Fix:** Ensure both call `_cameraManager.StartHardwareTrigger(_cameraSlots, progress)`

---

### Problem: No frames ever (only timeouts)

**Symptom in Debug Output:**
```
[CameraManager] WaitForNewFrame slot 0: timeout (2000ms)
[CameraManager] WaitForNewFrame slot 0: timeout (2000ms)
? repeating forever even when switch is pressed
```

**Possible Causes:**

| Cause | How to Check | Fix |
|-------|-------------|-----|
| Wrong trigger source line | Check wiring vs config | Change `TriggerSource` in `camera_slots.json` from `"Line0"` to `"Line1"` etc. |
| Camera not configured for HW trigger | Look for `TriggerMode=On` in logs | Ensure `StartHardwareTrigger()` is being called |
| 24V signal not reaching camera opto-in | Check wiring with multimeter | Hardware check |
| Camera DeviceIndex wrong | Check `[TriggerService] T1=0 cams` | Reconfigure camera slots via Setup dialog |
| `_cameraManager.IsStreaming=True` skips trigger setup | See log `Cameras already streaming` | Stop stream, reconfigure, restart |

---

### Problem: Pipeline runs but takes 4+ seconds with 2000ms delay

**Symptom:**
```
[Profiling] Trigger 1 dequeued ... (queue latency: 1464.1 ms)
? next trigger dequeued 1.5 seconds late
```

**Cause:** `WriteRejectionCoils` is running synchronously (blocking consumer).  
**File:** `TriggerService.cs` ? `ProcessTrigger()`  
**Check:** Ensure coil write uses `Task.Run(...)` not inline call.

```csharp
// Should look like this (fire-and-forget):
_ = Task.Run(() =>
{
    var (ok, err, coils) = WriteRejectionCoils(capturedGroup, capturedResults, capturedSlots);
    Debug.WriteLine(...);
});

// NOT like this (blocking):
var (ok, err, coils) = WriteRejectionCoils(triggerGroup, results, slotList); // ? wrong
```

---

### Problem: Frame captured but inspection never runs (pipeline hangs)

**Symptom:**
```
[Trigger] CAM 1 (delay 0ms): OK
[Profiling] Frame capture done: 25.6 ms
? then nothing, no Inspection done log
```

**Cause:** DateTime timezone mismatch causing enormous `Thread.Sleep`.  
**File:** `TriggerService.cs` ? `ProcessTrigger()`  
**Check this line:**

```csharp
// CORRECT (must use DateTime.Now for both):
int elapsed = (int)(DateTime.Now - triggerTime).TotalMilliseconds;

// WRONG (will sleep for hours in non-UTC timezones):
int elapsed = (int)(DateTime.UtcNow - triggerTime).TotalMilliseconds;
```

---

### Problem: "Cameras already streaming or not available (IsStreaming=True)"

**Symptom:** `StartHardwareTrigger()` is skipped, cameras stay in previous mode.

**Cause:** `_cameraManager.IsStreaming` is already `true` when `StartTriggerMode()` runs.

**Explanation:** `StartHardwareTrigger()` has a guard: `if (IsStreaming) return;`  
This protects against double-init but also means if stream was already started (e.g., via `Stream_Click`), `StartHardwareTrigger()` won't re-configure cameras.

**Fix:** Stop stream first, then restart:
1. Click "Stop Stream" in UI
2. Click "Start Stream" again (will call `StartHardwareTrigger`)
3. Or restart the application

---

### Problem: "No cameras assigned" — trigger detected but no inspection

**Symptom:**
```
[Trigger] Trigger 1: no cameras assigned.
```

**Cause:** `_cameraSlots` has no entries with `TriggerGroup == 1` AND `DeviceIndex >= 0`.

**Fix:** Open Camera Setup dialog, assign physical device indices to slots, save.

---

### Problem: Modbus output coils not firing (REJECT/REWORK not signalling PLC)

**Note:** This does NOT affect inspection or UI — it's background only.

**Check Debug Output:**
```
[Trigger] Coils: group=1, activated=[none], ok=False, err=Write ON failed @4001: timeout
```

**Possible Causes:**
- Wrong coil address in `trigger_config.json` (OutputCoils section)
- Modbus disconnected (check `[Modbus] Connected:` log at startup)
- Wrong COM port, baud rate, or slave ID
- PLC not responding

**Modbus is optional** — if it fails, triggers and inspection still work normally.

---

## 5. Debug Output Reference

### Startup (healthy):
```
[CameraManager] CAM 1: TriggerMode=On, Source=Line0, Activation=RisingEdge, Delay=50µs
[Trigger] Modbus connected for output coils: COM6 @ 115200
[TriggerService] cameras=streaming, inspection=loaded, T1=3 cams, T2=1 cams
[Trigger] Sentinel waiter started for trigger group 1.
[Trigger] Sentinel waiter started for trigger group 2.
[Trigger] Group 1 sentinel: CAM 1 (device 0)
[Trigger] Group 2 sentinel: CAM 4 (device 3)
[MainWindow] Hardware trigger mode started.
```

### Per-trigger (healthy):
```
[Profiling] Group 1 CAM 1: frame arrived 18:06:43.258
[Profiling] Group 1: TriggerEvent queued at 18:06:43.260
[Profiling] ???? Trigger 1 dequeued 18:06:43.282  (queue latency: 23.8 ms) ????
[Trigger] Processing Trigger 1 (3 cameras) at 18:06:43.283
[Trigger] CAM 1 (delay 0ms): OK
[Trigger] CAM 2 (delay 0ms): OK
[Trigger] CAM 3 (delay 0ms): OK
[Profiling] Frame capture done: 25.6 ms
[Trigger] CAM 1 [MaskRCNN] => PASS | total=116ms geo=85ms infer=31ms
[Trigger] CAM 2 [MaskRCNN] => PASS | total=98ms geo=72ms infer=26ms
[Trigger] CAM 3 [PatchCore] => PASS | anomalyScore=0.312 threshold=0.500
[Profiling] Inspection done: 119.8 ms
[Trigger] Trigger 1 batch complete: 119ms | verdicts=[PASS, PASS, PASS]
[Trigger] Coils: group=1, activated=[none], ok=True, err=no coil needed
```

### Waiting (normal when no trigger):
```
[CameraManager] WaitForNewFrame slot 0: timeout (2000ms)
[CameraManager] WaitForNewFrame slot 0: timeout (2000ms)
? these repeat every 2 seconds while waiting — this is NORMAL
```

---

## 6. Key Configuration Values

### `Assets/camera_slots.json` — per-camera settings
```json
[
  {
    "Slot": 0,               // display slot 0 = CAM 1
    "DeviceIndex": 0,        // physical camera index (from enumeration order)
    "Detector": "MaskRCNN",  // "MaskRCNN" or "PatchCore"
    "TriggerGroup": 1,       // which 24V signal line (1 = sensor 3001, 2 = sensor 3002)
    "CaptureDelayMs": 0,     // software delay after trigger before reading frame (0 = immediate)
    "TriggerSource": "Line0",// camera input line for 24V signal
    "TriggerDelayUs": 50.0   // hardware delay inside camera (microseconds)
  }
]
```

### `Assets/trigger_config.json` — Modbus & timing
```json
{
  "ComPort": "COM6",         // Modbus RS-485 port for OUTPUT coils only
  "BaudRate": 115200,
  "SlaveId": 1,
  "OutputCoils": {
    "Cam1_ReworkCoil": 3010, // coil address for CAM 1 REWORK signal
    "Cam1_RejectCoil": 3011, // coil address for CAM 1 REJECT signal
    "Cam234_RejectCoil": 3012,
    "Cam2_ReworkCoil": 3013,
    "Cam1_ReworkDelayMs": 500,   // ms after verdict before activating coil
    "Cam1_RejectDelayMs": 500,
    "Cam234_RejectDelayMs": 500,
    "Cam2_ReworkDelayMs": 500,
    "CoilOnDurationMs": 200,     // ms coil stays ON before turning OFF
    "ConflictPriority": "reject" // if REWORK and REJECT both: "reject" wins
  }
}
```

### Important: `CaptureDelayMs` vs `TriggerDelayUs`

| Parameter | Location | Unit | What it does |
|-----------|----------|------|-------------|
| `TriggerDelayUs` | `camera_slots.json` | microseconds | **Inside camera hardware** — delay between receiving 24V signal and starting exposure. Set by camera firmware. |
| `CaptureDelayMs` | `camera_slots.json` | milliseconds | **Software delay** — after frame arrives at PC, how long to wait before reading it from buffer. Normally 0. |

**Recommendation:** Keep `CaptureDelayMs = 0` for hardware trigger mode. The camera's `TriggerDelayUs = 50` is sufficient.

---

## Thread Names (visible in Visual Studio Threads window)

| Thread Name | Purpose |
|-------------|---------|
| `CamGrab_HW_0` | Grab thread for CAM 1 (hardware trigger mode) |
| `CamGrab_HW_1` | Grab thread for CAM 2 |
| `CamGrab_HW_2` | Grab thread for CAM 3 |
| `CamGrab_HW_3` | Grab thread for CAM 4 |
| `TriggerProducer_G1` | Sentinel waiter for trigger group 1 |
| `TriggerProducer_G2` | Sentinel waiter for trigger group 2 |
| `TriggerConsumer` | Processes inspection pipeline queue |

---

*End of Deployment Reference Guide*
