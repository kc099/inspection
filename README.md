# RoboViz - O-Ring Inspection System

WPF application for automated O-Ring defect detection on a conveyor line using multi-camera inspection, Mask R-CNN + PatchCore anomaly detection (ONNX), CUDA GPU acceleration, OpenCV geometric measurement, and Modbus RS-485 PLC integration for rejection output.

## Architecture

### Physical Layout

**Sensor 3001 (Trigger 1):**
- CAM 1 (top, MaskRCNN) - Geometric + Defect detection
  - REWORK -> coil 3010
  - REJECT -> coil 3011

**Sensor 3002 (Trigger 2):**
- CAM 2 (top, MaskRCNN) - Cam2 ratio-normalization pipeline
  - REWORK -> coil 3013
  - REJECT -> coil 3012
- CAM 3 (side, PatchCore) - REJECT -> coil 3012
- CAM 4 (side, PatchCore) - REJECT -> coil 3012

### Inspection Pipeline (per camera)

1. **Geometric Measurement** (OpenCvSharp)
   - CAM 1: bg subtraction -> morphology -> contours -> fit circles
   - CAM 2: ratio normalization (background.bmp) -> adaptive threshold -> morphology -> most circular component -> hole preservation
   - 6 metrics: outer/inner radius, circularity, center distance, eccentricity

2. **Threshold Evaluation** (per-sensor thresholds from CSV)
   - REWORK? (radius, circularity out of range) -> stop, activate rework coil
   - REJECT? (center distance, eccentricity out of range) -> stop, activate reject coil
   - PASS? -> continue to defect detection

3. **Defect Detection**
   - MaskRCNN: bin+crop 720x720 -> GPU inference -> REJECT if defects
   - PatchCore: crop -> ResNet50 features -> anomaly score -> REJECT if above threshold

4. **Rejection Output** (Modbus RS-485)
   - delay -> coil ON -> hold duration -> coil OFF

## Features

- **4-camera multi-sensor inspection** with configurable camera-to-trigger mapping
- **Dual detection engines**: Mask R-CNN (instance segmentation) + PatchCore (anomaly detection)
- **Cam2 geometric pipeline**: ratio normalization against background image (washer_inspector.py port)
- **Modbus RS-485 trigger mode**: edge-detect LOW->HIGH on PLC coils, auto-capture + inference
- **4-coil rejection output**: per-verdict coil activation with configurable delays and ON duration
- **Conflict priority**: configurable reject-vs-rework priority when cameras disagree
- **Model switching**: hot-swap between Model 1 and Model 2 thresholds at runtime
- **Auto-reconnect**: serial bus flush + full COM port reconnect on persistent Modbus failures
- **GPU acceleration**: CUDA EP with cuDNN EXHAUSTIVE algorithm search + 3-pass warmup

## Prerequisites

### 1. .NET 10 SDK

Download and install from https://dotnet.microsoft.com/download/dotnet/10.0

### 2. NVIDIA GPU Driver

| GPU | Min Driver | Download |
|-----|-----------|----------|
| RTX 5070/5080/5090 (Blackwell) | 572.xx+ | https://www.nvidia.com/Download/index.aspx |
| RTX 4060-4090 (Ada) | 528.xx+ | same link |
| RTX 3060-3090 (Ampere) | 472.xx+ | same link |

### 3. CUDA Toolkit 12.x

Download CUDA Toolkit **12.4 or later** from https://developer.nvidia.com/cuda-downloads

### 4. cuDNN 9

Install via pip: `pip install nvidia-cudnn-cu12`

Or download from https://developer.nvidia.com/cudnn-downloads and copy DLLs to CUDA bin folder.

## Build and Run

Open `RoboViz.sln` in Visual Studio 2022 17.14+ and press F5, or:

```
cd RoboViz
dotnet build
dotnet run
```

## Configuration

### Assets/trigger_config.json

Controls Modbus connection, trigger coils, and rejection output:

```json
{
  "ComPort": "COM6",
  "BaudRate": 115200,
  "SlaveId": 1,
  "TriggerCoil_Cam13": 3001,
  "TriggerCoil_Cam24": 3002,
  "OutputCoilAddress": 3003,
  "CaptureDelayMs": 400,
  "PollIntervalMs": 200,
  "OutputCoils": {
    "Cam1_ReworkCoil": 3010,
    "Cam1_RejectCoil": 3011,
    "Cam234_RejectCoil": 3012,
    "Cam2_ReworkCoil": 3013,
    "Cam1_ReworkDelayMs": 500,
    "Cam1_RejectDelayMs": 500,
    "Cam234_RejectDelayMs": 500,
    "Cam2_ReworkDelayMs": 500,
    "CoilOnDurationMs": 200,
    "ConflictPriority": "reject"
  }
}
```

| Field | Description |
|-------|-------------|
| `Cam1_ReworkCoil` | Coil address activated when CAM 1 verdict = REWORK |
| `Cam1_RejectCoil` | Coil address activated when CAM 1 verdict = REJECT |
| `Cam234_RejectCoil` | Coil address activated when any of CAM 2/3/4 verdict = REJECT |
| `Cam2_ReworkCoil` | Coil address activated when CAM 2 verdict = REWORK |
| `*DelayMs` | Delay (ms) after verdict before coil activation (conveyor travel time) |
| `CoilOnDurationMs` | How long (ms) the coil stays ON before turning OFF |
| `ConflictPriority` | `reject` = reject wins over rework; `rework` = both fire |

### Assets/camera_slots.json

Per-camera configuration (also editable via Camera Setup dialog):

| Field | Description |
|-------|-------------|
| `Slot` | Display slot (0-3 = CAM 1-4) |
| `TriggerGroup` | Which sensor triggers this camera (1 = 3001, 2 = 3002) |
| `Detector` | `MaskRCNN` or `PatchCore` |
| `DeviceIndex` | Physical camera device index (-1 = none) |
| `CaptureDelayMs` | Delay after trigger before frame capture |

## Project Files

### Services

| File | Description |
|------|-------------|
| `MaskRCNNDetector.cs` | ONNX Runtime GPU inference wrapper, CUDA/cuDNN setup |
| `PatchCoreDetector.cs` | PatchCore anomaly detection (ResNet50 feature extraction) |
| `YoloSegDetector.cs` | YOLO segmentation detector (alternative engine) |
| `OringMeasurement.cs` | OpenCvSharp geometric measurement - CAM 1 (bg subtraction) + CAM 2 (ratio normalization) |
| `InspectionService.cs` | Full pipeline: measure, evaluate, detect, verdict |
| `ThresholdConfig.cs` | JSON/CSV threshold loading, metric normalization, evaluation |
| `ModbusService.cs` | Modbus RTU master - read coils, write single/multiple coils, auto-reconnect |
| `TriggerService.cs` | Producer-consumer trigger pipeline - poll coils, capture frames, run inference, write rejection coils |
| `CameraManager.cs` | HIKVision camera SDK wrapper, multi-camera streaming |

### Models

| File | Description |
|------|-------------|
| `TriggerModels.cs` | TriggerConfig, OutputCoilConfig, TriggerResultEvent, TriggerPollStatus |
| `InspectionResult.cs` | Inspection result with verdict, metrics, overlay, timing |
| `GeometricResult.cs` | 6 geometric metrics + optional contour points (CAM 2) |
| `CameraSlotConfig.cs` | Per-camera configuration (slot, trigger group, detector, delay) |
| `Detection.cs` | Mask R-CNN detection result (box, mask, score, label) |
| `MetricModels.cs` | Metric thresholds, evaluation results |

### Views

| File | Description |
|------|-------------|
| `MainWindow.xaml/.cs` | Main UI - 4-camera grid, verdict banner, metrics table, Modbus panels |
| `CameraSetupDialog.xaml/.cs` | Camera-to-slot assignment dialog |

### Assets

| File | Description |
|------|-------------|
| `maskrcnn_oring_combined.onnx` | Mask R-CNN model |
| `patchcore_model*_resnet50_fp16.onnx` | PatchCore anomaly models |
| `model*_tuned_thresholds.json` | Tuned thresholds per model |
| `model*_*_measurements_stats.csv` | Per-sensor measurement statistics for threshold computation |
| `background.bmp` | CAM 2 background image for ratio normalization |
| `trigger_config.json` | Modbus + trigger + rejection coil configuration |

## NuGet Packages

| Package | Version | Purpose |
|---------|---------|---------|
| `Microsoft.ML.OnnxRuntime.Gpu` | 1.24.2 | ONNX inference with CUDA execution provider |
| `OpenCvSharp4` | 4.10.0 | OpenCV wrapper for geometric measurement |
| `OpenCvSharp4.runtime.win` | 4.10.0 | Native OpenCV binaries for Windows |
| `MvCameraControl.Net.Standard` | 4.4.1 | HIKVision camera SDK |
| `NModbus` | 3.0.81 | Modbus RTU protocol |
| `NModbus.Serial` | 3.0.81 | Serial port transport for NModbus |
| `System.IO.Ports` | 10.0.3 | .NET serial port access |

## GPU Compatibility

| GPU Family | Compute Capability | Status |
|------------|-------------------|--------|
| RTX 5070/5080/5090 (Blackwell) | SM 12.0 | Supported (CUDA 12.8+ recommended) |
| RTX 4060-4090 (Ada Lovelace) | SM 8.9 | Supported |
| RTX 3060-3090 (Ampere) | SM 8.6 | Supported |
| RTX 2060-2080 (Turing) | SM 7.5 | Supported |
| GTX 1060-1080 (Pascal) | SM 6.1 | Supported |

## Troubleshooting

### CPU (CUDA unavailable) shown in UI

1. **cuDNN not found** - run `pip install nvidia-cudnn-cu12` and restart
2. **CUDA toolkit not installed** - install CUDA 12.4+ from NVIDIA
3. **Driver too old** - update your GPU driver
4. **Wrong GPU** - integrated GPU selected; check device_id in MaskRCNNDetector.cs

### Modbus trigger not firing

1. Check `trigger_config.json` - verify COM port, baud rate, slave ID match your PLC
2. Check TriggerPollText in UI - should show `READ OK | coil 3001=low coil 3002=low`
3. If showing `READ FAIL` - check RS-485 wiring (A/B polarity), termination resistor
4. Auto-reconnect kicks in after 5 failures (flush) and 15 failures (full reconnect)

### Rejection coils not activating

1. Verify Modbus connection (green dot in RS-485 panel)
2. Check coil addresses in `trigger_config.json` match your PLC configuration
3. Set all `*DelayMs` to `0` for instant testing
4. Check ModbusStatusText / TriggerStatusText for write errors

### Typical Performance (RTX 3060 / CUDA 12.4)

| Phase | Time |
|-------|------|
| Geometric measurement | ~50 ms |
| Bin + crop to 720x720 | ~10 ms |
| Tensor preparation | ~5 ms |
| Mask R-CNN inference | ~400 ms |
| Overlay drawing | ~5 ms |
| **Total** | **~470 ms** |
