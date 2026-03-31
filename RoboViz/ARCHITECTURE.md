# RoboViz — O-Ring Inspection System: Architecture & Developer Guide

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Technology Stack](#2-technology-stack)
3. [Solution Structure](#3-solution-structure)
4. [File-by-File Reference](#4-file-by-file-reference)
5. [Inspection Pipeline — End to End](#5-inspection-pipeline--end-to-end)
6. [Geometric Measurement (Stage 1)](#6-geometric-measurement-stage-1)
7. [Threshold Evaluation (Stage 2)](#7-threshold-evaluation-stage-2)
8. [Defect Detection — Dual Detector Architecture (Stages 3–4)](#8-defect-detection--dual-detector-architecture-stages-34)
9. [Verdict Decision Matrix](#9-verdict-decision-matrix)
10. [Multi-Camera Batch Processing](#10-multi-camera-batch-processing)
11. [Live Camera Streaming](#11-live-camera-streaming)
12. [Modbus RS-485 PLC Integration](#12-modbus-rs-485-plc-integration)
13. [GPU / CUDA Setup & Diagnostics](#13-gpu--cuda-setup--diagnostics)
14. [Data Flow Diagram](#14-data-flow-diagram)
15. [Where to Start Reading](#15-where-to-start-reading)

---

## 1. Project Overview

RoboViz is a **WPF desktop application** for automated quality inspection of rubber O-Rings
in an industrial production line. It captures images from up to **4 HikRobot GigE/USB cameras**,
runs a multi-stage analysis pipeline, and communicates pass/reject decisions to a PLC
over **Modbus RTU (RS-485)**.

Given a high-resolution camera image (2448×2048 pixels), it produces one of three verdicts:

| Verdict    | Meaning                                                                 |
|------------|-------------------------------------------------------------------------|
| **PASS**   | O-Ring geometry is within tolerance and no surface defects were found.   |
| **REWORK** | O-Ring shape is deformed (radius or circularity out of range). May be salvageable. |
| **REJECT** | O-Ring is structurally unsound (off-center/eccentric) or has surface defects. Not recoverable. |

The system combines **three** independent analysis techniques:

- **Classical computer vision** (OpenCvSharp) — measures 6 geometric properties of the ring.
- **Mask R-CNN** (ONNX, GPU) — pixel-level defect segmentation (used on CAM 1 & 3).
- **PatchCore** (ONNX, GPU) — unsupervised anomaly detection with heatmap (used on CAM 2 & 4).

---

## 2. Technology Stack

| Component                | Technology                                              |
|--------------------------|---------------------------------------------------------|
| UI Framework             | WPF (.NET 10, C# 14)                                   |
| Geometric Measurement    | OpenCvSharp 4.10                                        |
| Defect Detection (seg.)  | Mask R-CNN (ONNX, 720×720 input)                       |
| Anomaly Detection        | PatchCore (ONNX FP16, ResNet-50 backbone, 640×640 input)|
| ML Inference Runtime     | Microsoft.ML.OnnxRuntime.Gpu 1.24.2                    |
| GPU Acceleration         | NVIDIA CUDA 12.x + cuDNN 9                             |
| Camera SDK               | MvCameraControl.Net.Standard 4.4.1 (HikRobot MVS)      |
| PLC Communication        | NModbus 3.0.81 (Modbus RTU over RS-485)                 |
| Serial Port              | System.IO.Ports 10.0.3                                  |
| Target OS                | Windows (x64)                                           |

---

## 3. Solution Structure

```
RoboViz/
├── RoboViz.sln                                     # Solution file
├── README.md                                        # Setup & prerequisites
├── RoboViz/
│   ├── RoboViz.csproj                               # Project file (net10.0-windows, WPF)
│   ├── App.xaml / App.xaml.cs                        # WPF application entry point
│   ├── MainWindow.xaml                               # UI layout (XAML)
│   ├── MainWindow.xaml.cs                            # UI code-behind (event handlers, display logic)
│   │
│   ├── InspectionService.cs                          # Pipeline orchestrator + InspectionResult model
│   ├── OringMeasurement.cs                           # OpenCV geometric measurement + image prep
│   ├── ThresholdConfig.cs                            # Metric definitions, threshold loading, evaluation
│   │
│   ├── MaskRCNNDetector.cs                           # Mask R-CNN ONNX inference (CUDA GPU) + diagnostics
│   ├── PatchCoreDetector.cs                          # PatchCore anomaly detection ONNX inference (CUDA GPU)
│   ├── maskrcnn_oring.onnx                           # Pre-trained Mask R-CNN model
│   ├── patchcore_model1_resnet50_fp16.onnx           # PatchCore FP16 model for Model 1
│   ├── patchcore_model2_resnet50_fp16.onnx           # PatchCore FP16 model for Model 2
│   ├── patchcore_model1_resnet50_fp16_ablation.json  # PatchCore metadata + threshold for Model 1
│   ├── patchcore_model2_resnet50_fp16_ablation.json  # PatchCore metadata + threshold for Model 2
│   ├── model1_tuned_thresholds.json                  # Geometric metric thresholds for Model 1
│   ├── model2_tuned_thresholds.json                  # Geometric metric thresholds for Model 2
│   │
│   ├── CameraManager.cs                              # HikRobot multi-camera streaming manager
│   ├── ModbusService.cs                              # Modbus RTU master for PLC rejection coils
│   │
│   ├── Fonts/                                        # Custom UI fonts (Orbitron, ShareTechMono, Montserrat)
│   └── Assets/                                       # UI assets (model1.png, model2.png)
```

---

## 4. File-by-File Reference

### `InspectionService.cs` — The Orchestrator

**Purpose:** Wires together the entire inspection pipeline. Now supports **two detector paths**.

| Member                              | Role                                                                                     |
|-------------------------------------|------------------------------------------------------------------------------------------|
| `InspectMaskRCNN(Bitmap rawImage)`  | Runs geometry → thresholds → Mask R-CNN (used for CAM 1, 3).                            |
| `InspectPatchCore(Bitmap rawImage)` | Runs geometry → thresholds → PatchCore anomaly detection (used for CAM 2, 4).            |
| `RunGeoEvaluation(...)`             | Shared helper: runs geometric measurement + threshold evaluation for both paths.         |
| `SwitchModel(string modelName)`     | Hot-swaps thresholds AND reloads the PatchCore ONNX model for the selected model.        |
| `DrawDefectOverlay(...)`            | Blends a red tint on Mask R-CNN mask pixels + draws score labels.                        |
| `BitmapToBitmapSource(...)`         | Converts GDI+ `Bitmap` to WPF `BitmapSource`. Freezes for thread safety.                |

**Also defines:** `InspectionResult` — carries verdict, timing, metric results, Mask R-CNN detections,
PatchCore anomaly score/threshold, and overlay image.

---

### `OringMeasurement.cs` — Geometric Analysis

**Purpose:** Classical computer vision to extract 6 measurable properties of the O-Ring.

| Method                       | Role                                                                                   |
|------------------------------|----------------------------------------------------------------------------------------|
| `Measure(Bitmap)`            | Full pipeline: grayscale → mask → contours → circle fit → 6 metrics.                   |
| `BinCrop720(Bitmap)`         | Downscales 2×2, crops to foreground bounding box, resizes/pads to 720×720.              |
| `DrawGeometricOverlay(...)` | Draws fitted circles (green=outer, red=inner) and a yellow center-distance line.        |
| `BuildMask(...)`             | Background subtraction → thresholding → morphological cleanup → largest component.      |
| `FitCircleLsq(...)`         | Least-squares circle fit from contour points (3×3 linear solve).                        |

**Also defines:** `GeometricResult` — holds 6 measured values and center coordinates.

---

### `ThresholdConfig.cs` — Metric Definitions & Evaluation

**Purpose:** Defines what is measured, acceptable ranges, and pass/fail judgment.

| Member                        | Role                                                                              |
|-------------------------------|-----------------------------------------------------------------------------------|
| `MetricDefs[]`                | Array of 6 metric definitions with display name, unit, threshold type, verdict category. |
| `LoadThresholds(jsonPath)`    | Reads lo/hi from JSON. Fills missing with defaults. Widens reject thresholds by 10%.     |
| `GetDefaultThresholds()`      | Fallback hardcoded thresholds if JSON is missing.                                 |
| `ComputeResolutionScale(...)` | Scale factor for normalizing pixel metrics to the reference 2448px resolution.    |
| `NormalizeMeasurements(...)`  | Divides linear metrics (radii, center distance) by the resolution scale.          |
| `Evaluate(...)`               | Compares each metric against thresholds → per-metric pass/fail + overall verdict. |

**Also defines:** `MetricDef`, `MetricThreshold`, `MetricEvalResult` (record types).

Note: The JSON threshold files (e.g., `model1_tuned_thresholds.json`) contain many more metrics
(18 total including `ring_thickness`, `thickness_cv`, `annular_area_k`, etc.), but **only 6 are used**
by the app — the rest are ignored at load time.

---

### `MaskRCNNDetector.cs` — Mask R-CNN Inference (CAM 1, 3)

**Purpose:** Loads and runs the Mask R-CNN ONNX model on GPU. Now includes full GPU diagnostics.

| Member                        | Role                                                                              |
|-------------------------------|-----------------------------------------------------------------------------------|
| Constructor                   | Multi-step GPU init: path setup → DLL probe → version check → CUDA session → warmup. Falls back to CPU on any failure. |
| `Detect(Bitmap, threshold)`   | Converts image to RGB float tensor → runs inference → parses boxes/scores/masks.  |
| `EnsureNvidiaLibsOnPath()`    | **Public static.** Adds pip-installed cuDNN/cuBLAS paths to `PATH`. Also used by `PatchCoreDetector`. |
| `ProbeNativeLibraries()`      | Loads every critical DLL (`cudart64_12`, `cublas64_12`, `cudnn64_9`, etc.) to verify they exist before creating the CUDA session. |
| `CheckVersionCompatibility()` | Calls CUDA/cuDNN version APIs and logs warnings (non-blocking).                   |
| `LogDiag(...)`                | **Public static.** Writes to `gpu_init.log` for troubleshooting.                 |
| `FillInputBuffer(...)`        | Converts BGR 24bpp Bitmap to CHW RGB float [0..1] planar format.                 |

**Also defines:** `Detection` — bounding box, confidence score, class label, per-pixel mask.

Model: Input `[1, 3, 720, 720]` → Output boxes/labels/scores/masks. Classes: 0=background, 1=defect. Max 5 detections.

---

### `PatchCoreDetector.cs` — PatchCore Anomaly Detection (CAM 2, 4) *(NEW)*

**Purpose:** Unsupervised anomaly detection using a PatchCore model (ResNet-50 backbone, FP16).

| Member                          | Role                                                                                 |
|---------------------------------|--------------------------------------------------------------------------------------|
| `LoadModel(path, threshold)`    | Loads ONNX model, resolves output names, runs 3-pass warmup. Disposes previous model.|
| `Unload()`                      | Disposes the current ONNX session to free GPU memory.                                |
| `Detect(Bitmap, out ...)`       | Runs inference → returns `PatchCoreResult` with anomaly score, heatmap, and boolean. |
| `Preprocess(Bitmap)`            | **Static.** Resize to 660×660 → center-crop to 640×640.                             |
| `DrawHeatmapOverlay(...)`       | **Static.** Blends a jet-colormap heatmap of the anomaly map onto the image.         |
| `LoadMetadata(jsonPath)`        | **Static.** Reads `PatchCoreMetadata` JSON (backbone, threshold, input shape).       |

**Also defines:**
- `PatchCoreResult` — anomaly score, 2D anomaly map, `IsAnomaly` flag.
- `PatchCoreMetadata` — JSON schema for model metadata (backbone, resize, crop, threshold).

Model: Input `[1, 3, 640, 640]` float RGB [0..1] → Output `anomaly_score [1]` + `anomaly_map [1, 1, 640, 640]`.
Handles both FP32 and FP16 output tensors automatically.

---

### `CameraManager.cs` — Multi-Camera Streaming *(NEW)*

**Purpose:** Manages up to 4 HikRobot GigE/USB cameras with continuous frame acquisition.

| Member                        | Role                                                                              |
|-------------------------------|-----------------------------------------------------------------------------------|
| `Initialize()`                | Enumerates all GigE/USB/GenTL cameras via MVS SDK.                                |
| `StartStreaming(progress)`    | Opens each camera, sets continuous acquisition mode, starts per-camera grab threads.|
| `StopStreaming()`             | Signals all grab threads to stop, waits for join, closes devices.                 |
| `GetLatestFrame(slot)`        | Returns a thread-safe copy of the most recent frame from a camera slot.           |
| `GetAllLatestFrames()`        | Returns latest frames from all 4 slots.                                           |
| `GrabThreadProc(slot)`        | Background thread: continuously calls `GetImageBuffer()` and updates latest frame.|

Each camera runs on its own background thread. The latest frame is always available via lock-protected
`Bitmap` references. The `MvsRuntimePath` static property points to the MVS native DLL directory.

---

### `ModbusService.cs` — PLC Communication *(NEW)*

**Purpose:** Sends pass/reject decisions to a PLC over Modbus RTU (RS-485 serial).

| Member                              | Role                                                                         |
|-------------------------------------|------------------------------------------------------------------------------|
| `Connect(comPort, baudRate, slaveId)`| Opens serial port + creates NModbus RTU master.                              |
| `Disconnect()`                      | Closes port and disposes master.                                             |
| `WriteRejectionCoils(...)`          | Writes 2 coils: Coil 0 = CAM 1+2 reject, Coil 1 = CAM 3+4 reject.         |
| `GetAvailablePorts()`               | **Static.** Lists available COM ports on the machine.                        |

Coil logic:
- **Coil 0 = 1** if either CAM 1 or CAM 2 verdict ≠ PASS
- **Coil 1 = 1** if either CAM 3 or CAM 4 verdict ≠ PASS
- **Coil 0/1 = 0** if both cameras in the pair pass

---

### `MainWindow.xaml.cs` — UI Code-Behind

**Purpose:** Handles user interaction, camera streaming, and result display.

| Feature                     | Implementation                                                                     |
|-----------------------------|------------------------------------------------------------------------------------|
| Load Image                  | `OpenFileDialog` → loads BMP into 2 or 4 frame displays.                          |
| Analyze                     | Runs `InspectMaskRCNN` (even CAMs) + `InspectPatchCore` (odd CAMs) in `Parallel.For`. |
| Live Stream                 | `CameraManager` grabs frames → `DispatcherTimer` polls at 500ms → auto-analyzes.  |
| Model Switching             | Toggle buttons swap Model 1 / Model 2 (reloads PatchCore model + thresholds).     |
| Camera Mode (2/4)           | Toggle between 2-camera and 4-camera layouts.                                      |
| Modbus Panel                | COM port selector, baud rate, slave ID, connect/disconnect, coil address config.   |
| Metric Table                | `ItemsControl` bound to `MetricRowViewModel` list showing value/lo/hi/status.      |
| Verdict Per-Frame           | Color-coded label on each camera tile.                                             |
| Timing Summary              | Batch total, per-frame average, per-camera breakdown, PatchCore anomaly scores.    |

---

## 5. Inspection Pipeline — End to End

`InspectionService` provides **two entry points** sharing the same geometric evaluation:

```
┌─────────────────────────────────────────────────────────────────┐
│  INPUT: Raw Bitmap (2448×2048) from camera                      │
└─────────────┬───────────────────────────────────────────────────┘
              │
              ▼
┌─────────────────────────────────────────────────────────────────┐
│  STAGE 1: Geometric Measurement (OringMeasurement.Measure)      │
│  • Grayscale → background subtraction → morphological cleanup   │
│  • Contour detection (RETR_CCOMP, 2-level hierarchy)            │
│  • Outer contour = largest by area                              │
│  • Inner contour = largest child of outer                       │
│  • Least-squares circle fit on both contours                    │
│  • Compute 6 metrics                                            │
│  ► If no contours found → return ERROR                          │
└─────────────┬───────────────────────────────────────────────────┘
              │ GeometricResult (6 metrics)
              ▼
┌─────────────────────────────────────────────────────────────────┐
│  STAGE 2: Threshold Evaluation (ThresholdConfig.Evaluate)       │
│  • Normalize pixel metrics by resolution scale                  │
│  • Compare each metric to its lo/hi range                       │
│  • Classify failures as "rework" or "reject"                    │
│  ► If any rework metric fails → return REWORK                   │
│  ► If any reject metric fails → return REJECT                   │
└─────────────┬───────────────────────────────────────────────────┘
              │ Geometric PASS → branch by detector type
              ▼
      ┌───────┴───────┐
      │               │
      ▼               ▼
┌──────────────┐ ┌──────────────────┐
│  CAM 1, 3    │ │  CAM 2, 4        │
│  Mask R-CNN  │ │  PatchCore       │
└──────┬───────┘ └──────┬───────────┘
       │                │
       ▼                ▼
┌──────────────┐ ┌──────────────────┐
│ BinCrop720   │ │ BinCrop720       │
│ 720×720      │ │ → Resize 660     │
│              │ │ → CenterCrop 640 │
└──────┬───────┘ └──────┬───────────┘
       │                │
       ▼                ▼
┌──────────────┐ ┌──────────────────┐
│ ONNX Infer.  │ │ ONNX Inference   │
│ boxes/masks  │ │ score + heatmap  │
│ score > 0.5? │ │ score > thresh?  │
└──────┬───────┘ └──────┬───────────┘
       │                │
       ▼                ▼
┌──────────────────────────────────┐
│  VERDICT:                        │
│  • Defects/anomaly → REJECT      │
│  • Clean           → PASS        │
└──────────────────────────────────┘
       │
       ▼
┌──────────────────────────────────┐
│  Modbus: Write rejection coils   │
│  Coil 0 = CAM1+2, Coil 1 = CAM3+4│
└──────────────────────────────────┘
```

---

## 6. Geometric Measurement (Stage 1)

### What Is Measured

| # | Metric               | How It's Computed                                                      | Unit |
|---|----------------------|------------------------------------------------------------------------|------|
| 1 | **Outer Radius**     | Least-squares circle fit on the largest contour                        | px   |
| 2 | **Inner Radius**     | Least-squares circle fit on the largest child contour of the outer     | px   |
| 3 | **Outer Circularity**| `4π × area / perimeter²` of the outer contour (1.0 = perfect circle)  | —    |
| 4 | **Inner Circularity**| `4π × area / perimeter²` of the inner contour                         | —    |
| 5 | **Center Distance**  | Euclidean distance between outer and inner fitted circle centers       | px   |
| 6 | **Eccentricity %**   | `center_distance / average_radius × 100`                              | %    |

### Image Processing Steps

1. Convert to grayscale.
2. Compute absolute difference from background value (default 24).
3. Threshold to binary mask.
4. If >75% foreground, invert (handles light-on-dark vs dark-on-light).
5. Morphological close (2 iterations) + open (1 iteration) with 7×7 elliptical kernel.
6. Keep only the largest connected component.
7. Find contours with `RETR_CCOMP` (2-level hierarchy: outer + holes).

### Circle Fitting

Uses an algebraic least-squares fit that solves a 3×3 linear system (Gaussian elimination with partial pivoting). This is a direct port of `fit_circle_lsq()` from the Python reference implementation.

---

## 7. Threshold Evaluation (Stage 2)

### The 6 Metrics and Their Verdict Categories

| Metric               | Category     | Threshold Type | Fail Condition                    |
|----------------------|-------------|----------------|-----------------------------------|
| `outer_radius`       | **rework**  | range          | Value > Hi (too large)            |
| `inner_radius`       | **rework**  | range          | Value < Lo (too small)            |
| `circularity_outer`  | **rework**  | min            | Value < Lo (not round enough)     |
| `circularity_inner`  | **rework**  | min            | Value < Lo (not round enough)     |
| `center_dist`        | **reject**  | max            | Value > Hi (centers too far apart)|
| `eccentricity_pct`   | **reject**  | max            | Value > Hi (too eccentric)        |

### Default Thresholds (reference resolution 2448px)

| Metric               | Lo     | Hi     |
|----------------------|--------|--------|
| `outer_radius`       | 650.0  | 680.0  |
| `inner_radius`       | 375.0  | 400.0  |
| `circularity_outer`  | 0.75   | 1.0    |
| `circularity_inner`  | 0.75   | 1.0    |
| `center_dist`        | 0.0    | 35.0   |
| `eccentricity_pct`   | 0.0    | 6.0    |

### Threshold JSON Files

The JSON files (e.g., `model1_tuned_thresholds.json`) contain **18 metrics** including
`ring_thickness`, `mean_thickness`, `thickness_cv`, `annular_area_k`, `edge_clearance`, etc.
However, **only 6 are loaded** by the app — the rest exist for the Python training pipeline
and are silently ignored.

### Threshold Loading Logic

1. Read `model1_tuned_thresholds.json` or `model2_tuned_thresholds.json` based on selected model.
2. Fill any missing metrics with defaults.
3. **Widen reject thresholds by 10%** (Lo × 0.9, Hi × 1.1) — matches the Python reference behavior.

### Resolution Normalization

Linear metrics (`outer_radius`, `inner_radius`, `center_dist`) are divided by a scale factor:

```
scale = max(image_width, image_height) / 2448
```

Dimensionless metrics (`circularity`, `eccentricity_pct`) are **not** scaled.

### Verdict Priority

```
if any rework metric fails → REWORK  (checked first)
else if any reject metric fails → REJECT
else → PASS (proceed to defect detection)
```

REWORK takes priority over REJECT because a geometrically deformed ring should be flagged for rework
before checking structural/eccentricity issues.

---

## 8. Defect Detection — Dual Detector Architecture (Stages 3–4)

The system uses **two different AI models** assigned to alternating cameras:

| Camera  | Detector        | Input Size | Detection Method               |
|---------|-----------------|------------|--------------------------------|
| CAM 1   | Mask R-CNN      | 720×720    | Instance segmentation (masks)  |
| CAM 2   | PatchCore       | 640×640    | Anomaly score + heatmap        |
| CAM 3   | Mask R-CNN      | 720×720    | Instance segmentation (masks)  |
| CAM 4   | PatchCore       | 640×640    | Anomaly score + heatmap        |

### 8a. Mask R-CNN Path (CAM 1, 3)

**When:** Geometry passes. **Image prep:** `BinCrop720` → 720×720.

1. Lock bitmap bits, extract BGR pixel data.
2. Convert to **CHW RGB float [0..1]** in a `ThreadLocal<float[]>` buffer.
3. Wrap as `DenseTensor<float>` shape `[1, 3, 720, 720]`.
4. Run `InferenceSession.Run()` (CUDA EP).
5. Parse outputs: `boxes[N,4]`, `labels[N]`, `scores[N]`, `masks[N,1,H,W]`.
6. Filter by `scoreThreshold` (0.5). Max 5 detections.
7. If any detection → **REJECT** with red-tinted mask overlay + score labels.

### 8b. PatchCore Path (CAM 2, 4)

**When:** Geometry passes. **Image prep:** `BinCrop720` → 720×720 → resize 660 → center-crop 640×640.

1. Same BGR → CHW RGB float conversion.
2. Tensor shape `[1, 3, 640, 640]`.
3. Run inference → outputs: `anomaly_score [1]` + `anomaly_map [1, 1, 640, 640]`.
4. Handles both FP32 and FP16 output tensors (model is FP16, runtime auto-converts).
5. If `anomaly_score > threshold` → **REJECT** with jet-colormap heatmap overlay.

PatchCore threshold is loaded from the model's JSON metadata file
(e.g., `patchcore_model2_resnet50_fp16_ablation.json` → `recommended_threshold`).

### Why Two Detectors?

Mask R-CNN is supervised (trained on labeled defect masks) — good for known defect types.
PatchCore is unsupervised (trained only on "good" images) — catches novel/unexpected anomalies.
Using both on different cameras provides complementary coverage.

---

## 9. Verdict Decision Matrix

| Condition                                                     | Verdict      | Overlay Shown                   |
|---------------------------------------------------------------|-------------|----------------------------------|
| Contours not detected                                         | **ERROR**   | Raw image                        |
| Outer radius too large                                        | **REWORK**  | Geometric overlay (fitted circles)|
| Inner radius too small                                        | **REWORK**  | Geometric overlay (fitted circles)|
| Outer or inner circularity too low                            | **REWORK**  | Geometric overlay (fitted circles)|
| Center distance too large                                     | **REJECT**  | Geometric overlay (fitted circles)|
| Eccentricity too high                                         | **REJECT**  | Geometric overlay (fitted circles)|
| Geometry OK + Mask R-CNN finds ≥1 defect (score > 0.5)       | **REJECT**  | Red-tinted defect mask overlay   |
| Geometry OK + PatchCore anomaly score > threshold             | **REJECT**  | Jet-colormap heatmap overlay     |
| Geometry OK + no defects/anomalies                            | **PASS**    | Raw image (unchanged)            |
| PatchCore model not loaded                                    | **ERROR**   | Raw image                        |

---

## 10. Multi-Camera Batch Processing

The system supports **2-camera** or **4-camera** modes (toggled via UI buttons).

### Static Image Analysis ("Analyze" button)

1. The loaded image is copied N times (N = 2 or 4).
2. `Parallel.For(0, N, ...)` runs inspection concurrently:
   - **Even indices (0, 2)** → `InspectMaskRCNN()`
   - **Odd indices (1, 3)** → `InspectPatchCore()`
3. Each frame gets its own overlay image and per-tile verdict label.
4. The **first frame's** detailed results populate the metric table.
5. Batch timing summary shows per-camera and average stats.

### Camera Assignment

```
CAM 1 (index 0) → Mask R-CNN   ┐
CAM 2 (index 1) → PatchCore    ┤ → Coil 0 (reject if either ≠ PASS)
                                │
CAM 3 (index 2) → Mask R-CNN   ┤
CAM 4 (index 3) → PatchCore    ┘ → Coil 1 (reject if either ≠ PASS)
```

---

## 11. Live Camera Streaming

### `CameraManager.cs` Architecture

- Enumerates all HikRobot cameras (GigE, USB, GenTL) via the MVS SDK.
- Opens each camera in **continuous acquisition mode** (no external trigger).
- Each camera gets a dedicated **background grab thread** (`GrabThreadProc`).
- Latest frame per slot is stored in a lock-protected `Bitmap` reference.
- GigE cameras get optimal packet size auto-configured.

### Streaming Loop

The UI uses a `DispatcherTimer` (500ms interval) that:

1. Polls `GetAllLatestFrames()` from the camera manager.
2. Displays raw frames in the UI.
3. If all frames are ready, runs the full analysis pipeline (`Parallel.For` with
   `InspectMaskRCNN`/`InspectPatchCore`).
4. Updates overlays, verdicts, metric table, timing.
5. Writes Modbus rejection coils.
6. Timer is paused during analysis to prevent re-entrant ticks.

### MVS Native Libraries

`CameraManager.EnsureMvsOnPath()` adds the MVS runtime directory
(`C:\Program Files (x86)\Common Files\MVS\Runtime\Win64_x64`) to `PATH` at process level.

---

## 12. Modbus RS-485 PLC Integration

### Protocol

- **Modbus RTU** over a USB-to-RS485 serial adapter.
- Default: 9600 baud, 8N1, slave ID 1.
- Configurable via UI panel (COM port dropdown, baud rate, slave ID, coil address).

### Coil Layout

| Coil Address | Meaning                                     | Value |
|-------------|---------------------------------------------|-------|
| Base + 0     | CAM 1 + CAM 2 rejection                    | 1 = reject (either not PASS), 0 = both PASS |
| Base + 1     | CAM 3 + CAM 4 rejection                    | 1 = reject (either not PASS), 0 = both PASS |

### Timing

Modbus writes are **fire-and-forget** on a background thread (`Task.Run`) to never block the UI.
The result (success/error) is posted back to the UI via `Dispatcher.BeginInvoke`.

---

## 13. GPU / CUDA Setup & Diagnostics

### Multi-Step GPU Initialization (`MaskRCNNDetector`)

The constructor follows a 5-step pipeline, aborting to CPU at any failure:

```
Step 1 → EnsureNvidiaLibsOnPath()      — add cuDNN/cuBLAS paths
Step 2 → ProbeNativeLibraries()        — LoadLibrary() on every critical DLL
Step 3 → CheckVersionCompatibility()   — call cuDriverGetVersion / cudnnGetVersion
Step 4 → Create InferenceSession       — with CUDA EP (may take 10-30s)
Step 5 → RunWarmUp (3 passes)          — trigger cuDNN algo search (may take 60s+)
```

All steps are logged to `gpu_init.log` in the application directory.

### Session Configuration (Both Detectors)

```
Mask R-CNN CUDA EP:
  device_id = 0
  cudnn_conv_algo_search = DEFAULT
  cudnn_conv_use_max_workspace = 1
  arena_extend_strategy = kSameAsRequested

PatchCore CUDA EP:
  device_id = 0
  cudnn_conv_algo_search = DEFAULT
  cudnn_conv_use_max_workspace = 1
  arena_extend_strategy = kSameAsRequested
```

### Library Discovery

`EnsureNvidiaLibsOnPath()` is a **public static** method on `MaskRCNNDetector` (shared with
`PatchCoreDetector`). It prepends pip-installed NVIDIA library paths (cuDNN, cuBLAS, cuFFT,
cuRAND, cuSOLVER, cuSPARSE) from `%USERPROFILE%\anaconda3\Lib\site-packages\nvidia\` to `PATH`.

### Diagnostic Logging

`MaskRCNNDetector.LogDiag(...)` writes timestamped lines to `gpu_init.log`. Both detectors
use this shared logger. The log is recreated on each application startup.

### Fallback

If any GPU initialization step fails, the detector falls back to CPU automatically.
`ActiveProvider` reflects the actual state (e.g., `"CPU (GPU warmup failed)"`).
`GpuError` stores the failure message for UI display.

---

## 14. Data Flow Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                         APPLICATION STARTUP                      │
│  1. Load Mask R-CNN ONNX (GPU init + warmup)                    │
│  2. Load PatchCore ONNX for selected model (GPU init + warmup)  │
│  3. Load geometric thresholds from JSON                          │
└──────────────────────────┬───────────────────────────────────────┘
                           │
              ┌────────────┴────────────┐
              ▼                         ▼
   ┌──────────────────┐     ┌────────────────────┐
   │  "Load Image"    │     │  "Start Stream"    │
   │  (file dialog)   │     │  (HikRobot cameras)│
   └────────┬─────────┘     └────────┬───────────┘
            │                        │
            ▼                        ▼
   ┌──────────────────┐     ┌────────────────────┐
   │  "Analyze"       │     │  DispatcherTimer   │
   │  (manual click)  │     │  (auto every 500ms)│
   └────────┬─────────┘     └────────┬───────────┘
            │                        │
            └────────────┬───────────┘
                         ▼
              ┌──────────────────────────────────┐
              │  Parallel.For(0, N)               │
              │  ┌─────────────────────────────┐  │
              │  │ Even CAM: InspectMaskRCNN() │  │
              │  │  ├─ Measure (OpenCV)        │  │
              │  │  ├─ Evaluate thresholds     │  │
              │  │  ├─ BinCrop720              │  │
              │  │  └─ Mask R-CNN (ONNX GPU)   │  │
              │  └─────────────────────────────┘  │
              │  ┌─────────────────────────────┐  │
              │  │ Odd CAM: InspectPatchCore() │  │
              │  │  ├─ Measure (OpenCV)        │  │
              │  │  ├─ Evaluate thresholds     │  │
              │  │  ├─ BinCrop720 + Crop640    │  │
              │  │  └─ PatchCore (ONNX GPU)    │  │
              │  └─────────────────────────────┘  │
              └──────────────┬───────────────────┘
                             │
                             ▼
              ┌──────────────────────────────────┐
              │  UI Thread:                       │
              │  ├─ Per-frame overlay images      │
              │  ├─ Per-frame verdict labels      │
              │  ├─ Metric table (CAM 1 details)  │
              │  ├─ PatchCore anomaly scores      │
              │  └─ Timing breakdown              │
              └──────────────┬───────────────────┘
                             │
                             ▼
              ┌──────────────────────────────────┐
              │  Modbus (background thread):      │
              │  ├─ Coil 0: CAM1+2 reject?       │
              │  └─ Coil 1: CAM3+4 reject?       │
              └──────────────────────────────────┘
```

---

## 15. Where to Start Reading

**Recommended reading order for a new developer:**

| Order | File                     | Why                                                                              |
|-------|--------------------------|----------------------------------------------------------------------------------|
| 1     | `InspectionService.cs`   | The **orchestrator**. Read `InspectMaskRCNN()` and `InspectPatchCore()` — they call everything else. `RunGeoEvaluation()` is the shared geometry stage. |
| 2     | `ThresholdConfig.cs`     | Understand the **6 metrics**, how thresholds work, and how REWORK/REJECT/PASS verdicts are decided from geometry alone. |
| 3     | `OringMeasurement.cs`    | See how the **6 metrics are computed** from raw images using OpenCV (contours, circle fitting, circularity). |
| 4     | `MaskRCNNDetector.cs`    | Understand the **Mask R-CNN inference** path: GPU init pipeline, tensor prep, output parsing. Also contains shared utilities (`LogDiag`, `EnsureNvidiaLibsOnPath`). |
| 5     | `PatchCoreDetector.cs`   | Understand the **PatchCore anomaly detection** path: model loading, FP16 handling, heatmap overlay. |
| 6     | `CameraManager.cs`       | See how **HikRobot cameras** are enumerated, opened, and streamed with per-camera grab threads. |
| 7     | `ModbusService.cs`       | See how **rejection decisions are sent to the PLC** over RS-485 serial. |
| 8     | `MainWindow.xaml.cs`     | See how the **UI ties everything together**: manual analysis, live streaming, model switching, Modbus panel. |
| 9     | `MainWindow.xaml`        | Understand the **UI layout** (4-camera grid, metric table, verdict banner, Modbus config panel). |
| 10    | `RoboViz.csproj`         | Check **dependencies** and build configuration. |
| 11    | `README.md`              | **Setup prerequisites** (CUDA, cuDNN, drivers, MVS SDK). |
