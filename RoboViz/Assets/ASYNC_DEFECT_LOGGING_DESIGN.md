# Async Defect Logging Design

## Summary
This implementation moves defect image logging off the trigger consumer path into a dedicated logging worker thread.

### Goals
- Prevent disk I/O from blocking next trigger processing.
- Save defect image + predicted pixel mask.
- Keep existing defect-only policy (`PASS` not saved).

## What changed

### 1. New async log dispatcher
- Added `Services/AsyncFrameLogDispatcher.cs`
- Uses bounded `BlockingCollection` queue.
- Dedicated background thread (`FrameLogWorker`) consumes log jobs.
- Queue capacity controlled by `FrameLogging.QueueCapacity`.

### 2. Trigger path logging is now non-blocking
- `TriggerService` now enqueues logging jobs instead of saving synchronously.
- Trigger consumer thread no longer waits on file writes.

### 3. Manual load-image path uses same async mechanism
- `MainWindow` manual `Analyze` path enqueues defect logs.
- Logs are queued for:
  1. app log dir (`FrameLogging.LogsDirectory` under app base), and
  2. loaded image folder `logs` subfolder.

### 4. Pixel mask + image output
- Logging saves a single human-readable file per defect.
- Source image for logging is `InspectionResult.OverlayImage` (when available), which already includes predicted mask overlay for MaskRCNN defects.
- This ensures saved resolution matches the actual inspection resolution/output (for example 512x384 MaskRCNN input output), not the original uploaded frame size.
- No separate mask sidecar file is written.

## Configuration
`Assets/trigger_config.json` -> `FrameLogging`:

- `Enabled`: enable/disable logging
- `SaveDefectsOnly`: only non-PASS verdicts
- `LogsDirectory`: app log directory
- `ImageFormat`: defect image format (`jpg`/`png`)
- `SavePredictedMask`: enable mask file output
- `QueueCapacity`: async queue size

## File naming
- Saved overlay image: `yyyyMMdd_HHmmss_fff_{seq}_cam{N}_{VERDICT}.{ext}`

## Threading behavior
- Trigger producers/consumers continue independent of disk write latency.
- Logging worker performs actual disk writes.
- On shutdown, dispatcher completes queue and joins worker.

## Notes
- If queue is full, new log item is dropped with debug message (`[FrameLog] Queue full, dropping log item.`).
- This favors trigger throughput over unbounded memory growth.
