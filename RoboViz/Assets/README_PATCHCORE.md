# PatchCore ONNX Export & WPF C# Integration Guide

This folder contains everything needed to export the trained PatchCore anomaly detection model to ONNX and run inference from a **WPF / C#** application.

---

## Contents

| File | Description |
|------|-------------|
| `export_patchcore_onnx.py` | Python script — exports trained PatchCore to ONNX |
| `patchcore_model1_resnet50.onnx` | Exported ONNX model for Model 1 (after export) |
| `patchcore_model2_resnet50.onnx` | Exported ONNX model for Model 2 (after export) |
| `patchcore_*.json` | Metadata (thresholds, preprocessing params) |
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
4. **Memory bank** — coreset embedded as a constant tensor (~80k × 3072)
5. **k-NN scoring** — decomposed squared-distance + top-9 nearest neighbors
6. **Gaussian smoothing** — 33×33 kernel (σ=4)
7. **Bilinear upsampling** — 40×40 anomaly map → 640×640

```
Input:  [1, 3, 640, 640]  float32, RGB, [0-1]
                    │
         ┌──────────┴──────────┐
         │  ImageNet normalize  │
         │  ResNet backbone     │
         │  Feature aggregate   │
         └──────────┬──────────┘
                    │  (1600, 3072) patches
         ┌──────────┴──────────┐
         │  k-NN vs bank       │
         │  Gaussian smooth    │
         │  Upsample to 640²   │
         └──────────┬──────────┘
                    │
    Output: anomaly_score [1]  +  anomaly_map [1, 1, 640, 640]
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

# Export a specific model
python onnx_export/export_patchcore_onnx.py --model model1 --backbone resnet50
python onnx_export/export_patchcore_onnx.py --model model2 --backbone resnet50

# Export all available models
python onnx_export/export_patchcore_onnx.py --all
```

The script will:
1. Load the trained `.pkl` from `patchcore/results/`
2. Build the self-contained ONNX module (backbone + bank + k-NN)
3. Export to ONNX with **opset 17**
4. Validate the ONNX graph
5. Verify PyTorch vs ONNX Runtime scores match
6. Benchmark CPU and GPU inference latency
7. Save a metadata JSON with recommended threshold

### Expected output

| Model | Bank Size | ONNX Size | Threshold |
|-------|-----------|-----------|-----------|
| Model 1 ResNet-50 | 80,800 × 3072 | ~1.0 GB | ~15.3 |
| Model 2 ResNet-50 | 104,000 × 3072 | ~1.3 GB | ~13.5 |

> **Note:** The ONNX files are large because the memory bank (~940 MB for Model 1)
> is embedded as a constant. This is intentional — it makes deployment a single file
> and lets ONNX Runtime optimize the k-NN computation on GPU.

---

## 3.  C# Project Setup

### Create a new WPF project

```
dotnet new wpf -n OringPatchCoreInspection --framework net8.0
cd OringPatchCoreInspection
```

### Install NuGet packages

```

**GPU inference** (recommended — CUDA 12 + cuDNN 9):
```
dotnet add package Microsoft.ML.OnnxRuntime.Gpu --version 1.18.0
dotnet add package System.Drawing.Common --version 8.0.0
```
Group>
```

---

## 4.  Model Input / Output

### Input

| Name | Shape | Type | Notes |
|------|-------|------|-------|
| `image` | `[1, 3, 640, 640]` | float32 | RGB, scaled to [0, 1] |

**Preprocessing in C#** (mirrors Python pipeline):
1. Resize to **660×660** (bicubic interpolation)
2. Center-crop to **640×640** (remove 10px from each edge)
3. BGR → RGB conversion
4. Scale pixel values to **[0, 1]** (divide by 255)
5. Reshape to CHW layout

> ImageNet normalization (`mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]`)
> is built into the ONNX model. C# only needs the [0, 1] scaling.

### Output

| Name | Shape | Type | Notes |
|------|-------|------|-------|
| `anomaly_score` | `[1]` | float32 | Image-level anomaly score (max of heatmap) |
| `anomaly_map` | `[1, 1, 640, 640]` | float32 | Per-pixel anomaly heatmap |

**Decision logic:**
```
if anomaly_score > threshold → REJECT (defect detected)
else                         → PASS   (good part)
```

The `recommended_threshold` is provided in the `.json` metadata file,
computed as `(good_max + defect_min) / 2` from training results.

---

## 5.  How the C# Code Works

### `PatchCoreDetector.cs`

Low-level ONNX Runtime wrapper:
- Creates `InferenceSession` (CPU or CUDA GPU)
- `BitmapToTensor()` — Bitmap → float32 NCHW tensor using `LockBits`
- `Detect(Bitmap)` — Runs inference, returns `PatchCoreResult` (score, map, verdict)

### `PatchCoreInspectionService.cs`

Production-ready service layer:
- `Preprocess(Bitmap)` — Resize 660 → center-crop 640
- `Inspect(Bitmap)` — Full pipeline: preprocess → detect → heatmap overlay
- `DrawHeatmapOverlay()` — Jet colormap overlay (blue → green → yellow → red)
- `BitmapToBitmapSource()` — GDI+ → WPF-compatible `BitmapSource`
- `LoadMetadata()` — Reads threshold and settings from the export JSON

### `PatchCoreMainWindow.xaml.cs`

Example WPF UI:
- Auto-discovers `patchcore_*.onnx` models in the application directory
- Model dropdown for switching between Model 1 / Model 2
- Side-by-side display: original image + anomaly heatmap
- Threshold slider with live re-evaluation (no re-inference needed)
- Overlay opacity slider for heatmap blending
- Async inference on background thread (UI stays responsive)

---

## 6.  GPU Acceleration

### Requirements

- NVIDIA GPU with CUDA Compute Capability ≥ 6.0
- CUDA Toolkit 12.x + cuDNN 9.x
- `Microsoft.ML.OnnxRuntime.Gpu` NuGet package

### Enable in code

```csharp
// In PatchCoreMainWindow.xaml.cs, change:
_service = new PatchCoreInspectionService(entry.OnnxPath, entry.Threshold, useGpu: true);
```

### Expected performance

| Provider | Latency (640×640) | VRAM | Notes |
|----------|-------------------|------|-------|
| CPU (4 threads) | ~2–5 s | 0 | No GPU needed |
| CUDA GPU | ~100–200 ms | ~1.5–2 GB | RTX 3060 tested |
| TensorRT | ~50–100 ms | ~1.5 GB | Via `AppendExecutionProvider_Tensorrt` |

---

## 7.  Comparison: PatchCore vs Mask R-CNN

| Aspect | PatchCore | Mask R-CNN |
|--------|-----------|------------|
| Type | Anomaly detection (unsupervised) | Object detection (supervised) |
| Training | Only needs good/normal images | Needs annotated defect masks |
| Output | Anomaly score + heatmap | Bounding boxes + instance masks |
| Use case | "Is this part defective?" | "Where exactly is the defect?" |
| ONNX size | ~1 GB (large memory bank) | ~168 MB |
| Inference | Score-based threshold | Detection-based threshold |

Both models can run in the same WPF application. The existing Mask R-CNN files
(`MaskRCNNDetector.cs`, `InspectionService.cs`) remain unchanged.

---

## 8.  Deployment Checklist

- [ ] Export ONNX model: `python onnx_export/export_patchcore_onnx.py --model model1`
- [ ] Copy `.onnx` + `.json` alongside the executable
- [ ] Install matching NuGet package (CPU or GPU)
- [ ] If GPU: install CUDA 12 + cuDNN 9 on target machine
- [ ] Test with sample images — verify scores match Python
- [ ] Tune threshold on validation set if needed
- [ ] Handle `Dispose()` — both `PatchCoreDetector` and service implement `IDisposable`
- [ ] Reuse `PatchCoreInspectionService` instance (don't recreate per frame)

---

## 9.  Troubleshooting

| Issue | Solution |
|-------|----------|
| ONNX export fails (protobuf size) | Use `--opset 17`; ensure onnx ≥ 1.14 |
| ONNX Runtime out of memory (GPU) | Reduce `coreset_ratio` during training, or use CPU |
| Score mismatch vs Python | Verify preprocessing: resize 660 → crop 640, RGB [0-1] |
| Very slow CPU inference | Expected ~2-5s; use GPU for production |
| `DllNotFoundException` | Ensure `onnxruntime.dll` is in output directory |
| Model dropdown empty | Place `patchcore_*.onnx` in the application directory |

---

## 10.  Quick Start Summary

```
1.  conda activate dl
2.  python onnx_export/export_patchcore_onnx.py --all
3.  dotnet new wpf -n OringPatchCoreInspection
4.  dotnet add package Microsoft.ML.OnnxRuntime
5.  dotnet add package System.Drawing.Common
6.  Copy PatchCore*.cs, *.xaml, *.onnx, *.json into project
7.  dotnet run
8.  Select model → Load Image → Analyze → see score + heatmap + verdict
```
