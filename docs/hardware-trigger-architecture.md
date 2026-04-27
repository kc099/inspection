# Hardware Trigger Architecture Notes

## Scope
This note explains:
- the earlier mixed trigger architecture,
- the planned/implemented hardware-trigger architecture,
- why the change is needed,
- and the exact code locations changed for this edit.

## Present architecture before this edit
The codebase was already moving toward hardware-trigger capture, but it still had a mixed design:

1. `MainWindow` loaded the model and enumerated cameras.
2. `TriggerService` already used sentinel camera waiters to detect trigger activity from camera frame arrival.
3. `ProcessTrigger(...)` still contained a fallback that could start normal camera streaming with `StartStreaming()` if cameras were not armed.
4. Startup camera-slot mapping was applied in the manual stream flow, but not consistently in the auto-start / trigger-start path.
5. Comments and some model metadata still described the old polling-based design, which made verification harder.

### Present processing flow before this edit
```text
App start
  -> load model
  -> enumerate cameras
  -> start trigger mode
  -> sentinel waiter sees frame arrival from trigger group
  -> queue TriggerEvent
  -> consumer processes trigger
  -> if cameras not streaming, fallback could switch to normal StartStreaming()
  -> collect frames using latest-frame reads + per-camera delays
  -> run inference
  -> write PLC output coils
  -> update UI
```

### Problem in that design
The main issue was that the code path was not fully locked to the hardware-trigger architecture.
That meant:
- the runtime could accidentally fall back to continuous streaming,
- startup could use stale/default camera indices instead of persisted slot mapping,
- comments still pointed reviewers to the old polling model,
- and verification of trigger behavior was harder because the architecture was not described consistently in one place.

## Planned and implemented architecture
The implemented direction is a strict hardware-trigger pipeline:

1. External sensor signal goes directly to camera trigger input.
2. Camera captures the frame in hardware-trigger mode.
3. `CameraManager` grab thread stores the latest frame.
4. A sentinel waiter in `TriggerService` blocks on `WaitForNewFrame(...)` for each trigger group.
5. When the sentinel camera gets a new frame, `TriggerService` enqueues a `TriggerEvent`.
6. A single consumer thread processes the event, waits for configured capture offsets, reads the triggered frames, runs inspection, posts results to UI, and schedules output coil writes.
7. Modbus is used only for output coils in this path, not for trigger detection.

### Planned / current processing flow
```text
Sensor fire
  -> camera opto input receives trigger
  -> Hik camera captures frame in hardware-trigger mode
  -> CameraManager grab thread publishes frame
  -> TriggerService sentinel waiter detects new frame
  -> queue TriggerEvent
  -> consumer dequeues event
  -> apply per-camera capture delay
  -> collect latest hardware-triggered frames for slots in that trigger group
  -> run Mask R-CNN inspection
  -> return results to UI
  -> schedule PLC reject/rework coils asynchronously
```

## Why this change is needed
- Removes ambiguity between polling-based and hardware-trigger-based processing.
- Prevents `TriggerService` from silently switching to continuous streaming.
- Makes auto-start and manual start use the same persisted camera-slot mapping.
- Keeps trigger timing aligned with the physical camera trigger path.
- Makes debugging easier because comments, configuration notes, and runtime behavior now match.

## Code changes made for this edit

### 1. `RoboViz/Services/TriggerService.cs`

#### Architecture summary updated
- `RoboViz/Services/TriggerService.cs:11-16`
- The class summary now describes sentinel waiters + consumer processing instead of Modbus polling.

#### Backward-compatible constructor cleanup
- `RoboViz/Services/TriggerService.cs:42-59`
- The optional `onPollStatus` callback is retained only for compatibility, but explicitly ignored in the hardware-trigger flow.

#### Obsolete watcher state removed from stop path
- `RoboViz/Services/TriggerService.cs:119-137`
- Removed joins and state related to the old unused watcher path so only active producer threads and the consumer thread are managed.

#### Strict hardware-trigger readiness check added
- `RoboViz/Services/TriggerService.cs:243-277`
- `ProcessTrigger(...)` now fails fast unless cameras are both:
  - streaming, and
  - armed in hardware-trigger mode.
- This replaced the old fallback behavior that could call `StartStreaming()` and drift away from the intended architecture.

#### Capture-phase comment aligned with actual logic
- `RoboViz/Services/TriggerService.cs:279-309`
- Clarified that capture is reading the latest hardware-triggered frames after the trigger event.

#### Obsolete `HardwareWatcherLoop(...)` removed
- Removed from `RoboViz/Services/TriggerService.cs`
- This method was no longer used after the sentinel waiter + queue-based design became the active path.

### 2. `RoboViz/Views/MainWindow.xaml.cs`

#### Startup comments aligned with hardware-trigger arming
- `RoboViz/Views/MainWindow.xaml.cs:149-153`
- App startup now clearly documents that cameras are enumerated first and armed when trigger mode starts.

#### New helper to apply persisted slot-to-device mapping everywhere
- `RoboViz/Views/MainWindow.xaml.cs:188-196`
- Added `ApplyCameraSlotSelection()`.
- This pushes `_cameraSlots[].DeviceIndex` into `CameraManager.CameraIndices` before enumeration or arming.

#### Auto-start enumeration path now uses persisted slot mapping
- `RoboViz/Views/MainWindow.xaml.cs:202-208`
- `InitializeCamerasAsync(...)` now applies camera-slot mapping before creating/initializing `CameraManager`.

#### Shared start-stream path now uses persisted slot mapping
- `RoboViz/Views/MainWindow.xaml.cs:254-259`
- `StartCameraStreamAsync(...)` now also applies the same mapping before camera initialization.

#### Trigger start path now applies persisted slot mapping before arming
- `RoboViz/Views/MainWindow.xaml.cs:288-349`
- `StartTriggerMode()` now applies the slot mapping before trigger service creation and hardware-trigger arming.
- Status text was updated to say `waiting for camera-triggered frames...`.

#### Manual camera-setup flow now reuses the same helper
- `RoboViz/Views/MainWindow.xaml.cs:615-620`
- Replaced the inline `CameraManager.CameraIndices = ...` assignment with `ApplyCameraSlotSelection()`.

### 3. `RoboViz/Models/TriggerModels.cs`

#### Trigger mode comment clarified as legacy-compatible config
- `RoboViz/Models/TriggerModels.cs:21-26`
- Comment updated so reviewers know `CameraTriggerMode` is retained for compatibility.

#### `TriggerEvent` comment aligned with hardware-trigger source
- `RoboViz/Models/TriggerModels.cs:118-122`
- Comment now states that the event is produced by the active trigger source and currently comes from sentinel waiter threads.

## Verification checklist
1. Start the app with saved `camera_slots.json` and confirm correct device indices are used without opening the camera setup dialog.
2. Confirm `TriggerStatusText` shows `Running [HW TRIGGER]` and `waiting for camera-triggered frames...`.
3. Fire Trigger 1 and Trigger 2 physically and verify:
   - sentinel frame arrival is logged,
   - `TriggerEvent` is queued,
   - expected slots are processed,
   - verdicts appear in the correct UI tiles,
   - output coils are written only after inference.
4. Stop streaming and start trigger mode again. Verify the service does not switch to normal `StartStreaming()` implicitly.
5. If a camera is not armed or hardware-trigger mode is not active, verify the UI now reports `Not ready: cameras are not armed` or `Not ready: cameras are not in hardware-trigger mode`.

## Notes
- Conveyor reference from project instructions: Conveyor 1 length `112 cm`, travel time `7.82 s`, sensor-to-camera distance `140 mm`, belt speed `143.2 mm/s`.
- The per-camera `CaptureDelayMs` values are still the place to tune frame pickup timing relative to the hardware trigger event.
