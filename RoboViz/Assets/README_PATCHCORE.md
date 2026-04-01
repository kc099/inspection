# PatchCore ONNX Export & WPF C# Integration Guide

This folder contains everything needed to export the trained PatchCore anomaly detection model to ONNX and run inference from a **WPF / C#** application.

> **Last updated**: April 2026 — 384×384 input, 1% coreset (~940 patches), ~100 MB ONNX files.

---

## Contents

| File | Description |
|------|-------------|
| `export_patchcore_onnx.py` | Python script — exports trained PatchCore to ONNX |
| `patchcore_model1_cropped_resnet50.onnx` | PatchCore ONNX for Model 1 (100.6 MB) |
| `patchcore_model2_cropped_resnet50.onnx` | PatchCore ONNX for Model 2 (100.7 MB) |
| `patchcore_model*_cropped_resnet50.json` | Metadata (input shape, thresholds, preprocessing) |
| `PatchCoreDetector.cs` | C# — low-level ONNX Runtime wrapper |
| `PatchCoreInspectionService.cs` | C# — preprocessing + inference + heatmap overlay |
| `PatchCoreMainWindow.xaml` | WPF UI layout |
| `PatchCoreMainWindow.xaml.cs` | WPF code-behind |
| `README_PATCHCORE.md` | This guide |

---

## 1.  Architecture

The exported ONNX model is **self-contained** — it includes:

1. **ImageNet normalization** — input only needs `[0, 1]` scaling
2. **ResNet-50 backbone** — layers 3 & 4 → 1024-d + 2048-d features
3. **Feature aggregation** — upsample + concatenate → 3072-d patch descriptors
4. **Memory bank** — 1% coreset embedded as a constant tensor (~940 × 3072)
5. **k-NN scoring** — decomposed squared-distance + top-9 nearest neighbors
6. **Gaussian smoothing** — 33×33 kernel (σ=4)
7. **Bilinear upsampling** — 24×24 anomaly map → 384×384

```
Input:  [1, 3, 384, 384]  float32, RGB, [0-1]
                    │
         ┌──────────┴──────────┐
         │  ImageNet normalize  │
         │  ResNet backbone     │
         │  Feature aggregate   │
         └──────────┬──────────┘
                    │  (576, 3072) patches
         ┌──────────┴──────────┐
         │  k-NN vs bank       │
         │  Gaussian smooth    │
         │  Upsample to 384²   │
         └──────────┬──────────┘
                    │
    Output: anomaly_score [1]  +  anomaly_map [1, 1, 384, 384]
```

---

## 2.  Exporting from Python

### Prerequisites

```
pip install torch torchvision onnx onnxruntime
```

### Run the export

```bash
cd "F:\standard elastomers"
conda activate dl

# Export from YOLO-cropped results
python onnx_export/export_patchcore_onnx.py --results-dir patchcore/results_cropped --model model1_cropped
python onnx_export/export_patchcore_onnx.py --results-dir patchcore/results_cropped --model model2_cropped

# Export all available models
python onnx_export/export_patchcore_onnx.py --results-dir patchcore/results_cropped --all
```

The script will:
1. Load the trained `.pkl` from `patchcore/results_cropped/`
2. Build the self-contained ONNX module (backbone + bank + k-NN)
3. Export to ONNX with **opset 17**
4. Validate the ONNX graph
5. Verify PyTorch vs ONNX Runtime scores match
6. Benchmark inference latency
7. Save a metadata JSON

### Expected output

| Model | Bank Size | ONNX Size | 2σ Threshold |
|-------|-----------|-----------|-------------|
| Model 1 ResNet-50 | 938 × 3072 | 100.6 MB | 29.76 |
| Model 2 ResNet-50 | 944 × 3072 | 100.7 MB | 30.60 |

> Bank size is small (~11 MB fp32) thanks to 1% coreset sampling. The bulk of the ONNX file is the ResNet-50 backbone (~90 MB).

---

## 3.  C# Project Setup

### Create a new WPF project

```
dotnet new wpf -n OringPatchCoreInspection --framework net8.0
cd OringPatchCoreInspection
```

### Install NuGet packages

**CPU inference:**
```
dotnet add package Microsoft.ML.OnnxRuntime --version 1.21.0
dotnet add package System.Drawing.Common --version 8.0.0
```

**GPU inference** (recommended — CUDA 12 + cuDNN 9):
```
dotnet add package Microsoft.ML.OnnxRuntime.Gpu --version 1.21.0
dotnet add package System.Drawing.Common --version 8.0.0
```

> Do **not** install both CPU and GPU packages — choose one.

### Add ONNX models to project

```xml
<ItemGroup>
  <None Update="patchcore_model1_cropped_resnet50.onnx">
    <CopyToOutputDirectory>PreserveNewest</CopyToOutputDirectory>
  </None>
  <None Update="patchcore_model2_cropped_resnet50.onnx">
    <CopyToOutputDirectory>PreserveNewest</CopyToOutputDirectory>
  </None>
</ItemGroup>
```

---

## 4.  Model Input / Output

### Input

| Name | Shape | Type | Notes |
|------|-------|------|-------|
| `image` | `[1, 3, 384, 384]` | float32 | RGB, scaled to [0, 1] |

**Preprocessing in C#:**
1. Take YOLO crop (variable size, typically ~370×260)
2. **Bicubic resize** to **384×384** (no padding)
3. BGR → **RGB** conversion
4. Scale pixel values to **[0, 1]** (divide by 255)
5. Reshape to **CHW** layout → `[1, 3, 384, 384]`

> ImageNet normalization is built into the ONNX model. C# only needs [0, 1] scaling.

### Output

| Name | Shape | Type | Notes |
|------|-------|------|-------|
| `anomaly_score` | `[1]` | float32 | Image-level anomaly score (max of heatmap) |
| `anomaly_map` | `[1, 1, 384, 384]` | float32 | Per-pixel anomaly heatmap |

**Decision logic:**
```
if anomaly_score > threshold → REJECT (defect detected)
else                         → PASS   (good part)
```

| Model | 2σ Threshold |
|-------|-------------|
| Model 1 | **29.76** |
| Model 2 | **30.60** |

---

## 5.  How the C# Code Works

### `PatchCoreDetector.cs`

Low-level ONNX Runtime wrapper:
- Creates `InferenceSession` (CPU or CUDA GPU)
- `BitmapToTensor()` — Bitmap → float32 NCHW tensor using `LockBits`
- `Detect(Bitmap)` — Runs inference, returns `PatchCoreResult` (score, map, verdict)

### `PatchCoreInspectionService.cs`

Production-ready service layer:
- `Preprocess(Bitmap)` — Bicubic resize crop to 384×384
- `Inspect(Bitmap)` — Full pipeline: preprocess → detect → heatmap overlay
- `DrawHeatmapOverlay()` — Jet colormap overlay (blue → green → yellow → red)
- `BitmapToBitmapSource()` — GDI+ → WPF-compatible `BitmapSource`

### `PatchCoreMainWindow.xaml.cs`

Example WPF UI:
- Auto-discovers `patchcore_*.onnx` models in the application directory
- Model dropdown for switching between Model 1 / Model 2
- Side-by-side display: original image + anomaly heatmap
- Threshold slider for adjusting sensitivity
- Async inference on background thread

> **Note**: The C# files reference the old 640×640 input size. Update the resize target to **384×384**, model filenames to `*_cropped_*`, and thresholds to 29.76 / 30.60.

---

## 6.  GPU Acceleration

### Requirements

- NVIDIA GPU with CUDA Compute Capability ≥ 6.0
- CUDA Toolkit 12.x + cuDNN 9.x
- `Microsoft.ML.OnnxRuntime.Gpu` NuGet package

### Expected performance

| Device | Latency |
|--------|---------|
| GPU (ONNX Runtime CUDA) | ~9-10 ms |
| CPU (ONNX Runtime) | ~100-200 ms |

---

## 7.  Deployment Checklist

- [ ] Export ONNX models: `python onnx_export/export_patchcore_onnx.py --results-dir patchcore/results_cropped --all`
- [ ] Copy `.onnx` + `.json` alongside the executable
- [ ] Install matching NuGet package (CPU or GPU)
- [ ] If GPU: install CUDA 12 + cuDNN 9 on target machine
- [ ] Test with sample images — verify scores match Python
- [ ] Tune threshold on validation set if needed
- [ ] Handle `Dispose()` — both `PatchCoreDetector` and service implement `IDisposable`
- [ ] Reuse `PatchCoreInspectionService` instance (don't recreate per frame)

---

## 8.  Troubleshooting

| Issue | Solution |
|-------|----------|
| ONNX export fails (protobuf size) | Use `--opset 17`; ensure onnx ≥ 1.14 |
| Score mismatch vs Python | Verify preprocessing: bicubic resize to 384×384, RGB [0-1] |
| `DllNotFoundException` | Ensure `onnxruntime.dll` is in output directory |
| Model dropdown empty | Place `patchcore_*_cropped_*.onnx` in the application directory |

---

## 9.  Quick Start Summary

```
1.  conda activate dl
2.  python onnx_export/export_patchcore_onnx.py --results-dir patchcore/results_cropped --all
3.  dotnet new wpf -n OringPatchCoreInspection
4.  dotnet add package Microsoft.ML.OnnxRuntime.Gpu
5.  dotnet add package System.Drawing.Common
6.  Copy PatchCore*.cs, *.xaml, *.onnx, *.json into project
7.  Update resize target to 384×384, thresholds to 29.76 / 30.60
8.  dotnet run
9.  Select model → Load Image → Analyze → see score + heatmap + verdict
```
