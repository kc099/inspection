# O-Ring Inspection — ONNX Deployment Guide (WPF / C#)

Complete pipeline: **Camera capture → 4×4 bin → YOLO segmentation → Crop → Resize 384×384 → PatchCore anomaly detection**.

> **Last updated**: April 2026 — YOLO at 512×384 (no letterbox), PatchCore at 384×384, 1% coreset.

---

## ONNX Models

| File | Input Shape | Size | Purpose |
|------|-------------|------|---------|
| `yolo11_seg_oring.onnx` | `[1, 3, 384, 512]` | 85.4 MB | YOLO11m-seg — segments O-ring from background |
| `patchcore_model1_cropped_resnet50.onnx` | `[1, 3, 384, 384]` | 100.6 MB | PatchCore — anomaly detection for Model 1 O-rings |
| `patchcore_model2_cropped_resnet50.onnx` | `[1, 3, 384, 384]` | 100.7 MB | PatchCore — anomaly detection for Model 2 O-rings |

---

## Pipeline Overview

```
  Camera (2048×1536 BMP)
         │
    ┌────┴─────────────────────────────────┐
    │  Step 1: 4×4 bin → 512×384           │
    │  (INTER_AREA, exact integer divide)  │
    └────┬─────────────────────────────────┘
         │
    ┌────┴─────────────────────────────────┐
    │  Step 2: YOLO11-seg ONNX inference   │
    │  Input: [1, 3, 384, 512]  (no pad!) │
    │  → segmentation mask of O-ring       │
    └────┬─────────────────────────────────┘
         │
    ┌────┴─────────────────────────────────┐
    │  Step 3: Extract bounding box of     │
    │  mask on the 512×384 image           │
    │  → crop that region (no padding)     │
    └────┬─────────────────────────────────┘
         │
    ┌────┴─────────────────────────────────┐
    │  Step 4: PatchCore preprocessing     │
    │  Bicubic resize crop → 384×384       │
    │  Scale pixels to [0, 1]              │
    └────┬─────────────────────────────────┘
         │
    ┌────┴─────────────────────────────────┐
    │  Step 5: PatchCore ONNX inference    │
    │  → anomaly_score + anomaly_map       │
    └────┬─────────────────────────────────┘
         │
    ┌────┴─────────────────────────────────┐
    │  Step 6: Threshold decision          │
    │  score > threshold → DEFECT          │
    └──────────────────────────────────────┘
```

---

## Step-by-Step Preprocessing Details

### Step 1 — 4×4 Binning (2048×1536 → 512×384)

The camera produces 2048×1536 BMP images. We downsample by exactly 4× in each direction using area interpolation (equivalent to 4×4 pixel binning).

| Property | Value |
|----------|-------|
| Original size | 2048 × 1536 (from camera) |
| Target size | **512 × 384** (exact ÷4) |
| Method | **Area interpolation** (INTER_AREA) |
| Color space | BGR → **RGB** for both models |

```csharp
// C# — 4×4 bin original 2048×1536 → 512×384
// For best quality, use area interpolation (average of 4×4 blocks)
Bitmap binned = new Bitmap(512, 384);
using (Graphics g = Graphics.FromImage(binned))
{
    g.InterpolationMode = InterpolationMode.HighQualityBilinear;
    g.DrawImage(original, 0, 0, 512, 384);
}
```

> **No letterbox padding.** The YOLO model was trained on 512×384 rectangular images. No black padding is needed.

---

### Step 2 — YOLO Segmentation Inference

| Property | Value |
|----------|-------|
| Model | `yolo11_seg_oring.onnx` |
| Input name | `images` |
| Input shape | `[1, 3, 384, 512]` float32 **(H=384, W=512)** |
| Input format | **RGB**, scaled to **[0, 1]** (divide by 255) |
| Output 0 | `output0`: `[1, 37, 4032]` — detection boxes + class scores + mask coefficients |
| Output 1 | `output1`: `[1, 32, 96, 128]` — prototype mask features |
| Confidence threshold | 0.25 |

```csharp
// C# — prepare YOLO input tensor [1, 3, 384, 512]
float[] yoloInput = new float[1 * 3 * 384 * 512];
for (int y = 0; y < 384; y++)
{
    for (int x = 0; x < 512; x++)
    {
        Color pixel = binned.GetPixel(x, y);
        yoloInput[0 * 384 * 512 + y * 512 + x] = pixel.R / 255.0f;  // R
        yoloInput[1 * 384 * 512 + y * 512 + x] = pixel.G / 255.0f;  // G
        yoloInput[2 * 384 * 512 + y * 512 + x] = pixel.B / 255.0f;  // B
    }
}
```

**Output parsing:**
- `output0` contains 4032 candidate detections. Each detection has 37 values:
  - `[0:4]` — bounding box (cx, cy, w, h) in **512×384** coordinates
  - `[4:5]` — class confidence (1 class: "oring")
  - `[5:37]` — 32 mask coefficients
- `output1` contains 32 prototype masks at **96×128** resolution (¼ of 384×512)
- To get the final mask: multiply mask coefficients × prototype masks, then sigmoid, then resize to 384×512
- **No letterbox offset** — coordinates map directly to the 512×384 image

---

### Step 3 — Crop O-Ring Region

After obtaining the segmentation mask, extract the tight bounding rectangle.

| Property | Value |
|----------|-------|
| Coordinate space | **512 × 384** (direct, no offset needed) |
| Padding | **0 px** (exact mask bounding box) |
| Crop source | The 512×384 binned image |

```csharp
// Find bounding box of mask pixels
int x1 = maskMinX;
int y1 = maskMinY;
int x2 = maskMaxX;
int y2 = maskMaxY;

// Clamp to image bounds
x1 = Math.Max(0, x1);
y1 = Math.Max(0, y1);
x2 = Math.Min(512, x2);
y2 = Math.Min(384, y2);

// Crop from the 512×384 binned image
Bitmap crop = binned.Clone(new Rectangle(x1, y1, x2 - x1, y2 - y1), binned.PixelFormat);
```

**Typical crop sizes:** Model1 avg ~381×249, Model2 avg ~367×268 (varies per O-ring position).

---

### Step 4 — PatchCore Preprocessing (on the crop)

The variable-size crop is resized to a fixed **384×384** square using bicubic interpolation. No padding — pure resize.

| Step | Operation | Details |
|------|-----------|---------|
| 4a | **Resize** | Bicubic resize to **384 × 384** |
| 4b | **Scale** | Divide pixel values by **255.0** → **[0, 1]** range |
| 4c | **Channel order** | **RGB** (not BGR) |
| 4d | **Tensor layout** | **CHW** — `[1, 3, 384, 384]` |

**ImageNet normalization is NOT needed** — it is already embedded inside the ONNX model.

```csharp
// 4a — Bicubic resize crop to 384×384
Bitmap finalCrop = new Bitmap(384, 384);
using (Graphics g = Graphics.FromImage(finalCrop))
{
    g.InterpolationMode = InterpolationMode.HighQualityBicubic;
    g.DrawImage(crop, 0, 0, 384, 384);
}

// 4b + 4c + 4d — To float tensor [1, 3, 384, 384], RGB, [0-1]
float[] patchcoreInput = new float[1 * 3 * 384 * 384];
for (int y = 0; y < 384; y++)
{
    for (int x = 0; x < 384; x++)
    {
        Color pixel = finalCrop.GetPixel(x, y);
        patchcoreInput[0 * 384 * 384 + y * 384 + x] = pixel.R / 255.0f;
        patchcoreInput[1 * 384 * 384 + y * 384 + x] = pixel.G / 255.0f;
        patchcoreInput[2 * 384 * 384 + y * 384 + x] = pixel.B / 255.0f;
    }
}
```

---

### Step 5 — PatchCore Inference

| Property | Value |
|----------|-------|
| Model 1 | `patchcore_model1_cropped_resnet50.onnx` |
| Model 2 | `patchcore_model2_cropped_resnet50.onnx` |
| Input name | `image` |
| Input shape | `[1, 3, 384, 384]` float32 |
| Input format | **RGB**, **[0, 1]** scaled (no ImageNet normalization needed) |
| Output 0 | `anomaly_score`: `[1]` float32 — image-level anomaly score |
| Output 1 | `anomaly_map`: `[1, 1, 384, 384]` float32 — spatial heatmap |

---

### Step 6 — Threshold Decision

| Model | 2σ Threshold | Verdict |
|-------|-------------|---------|
| Model 1 | **29.76** | score > 29.76 → **DEFECT** |
| Model 2 | **30.60** | score > 30.60 → **DEFECT** |

The `anomaly_map` output can be used to generate a heatmap overlay showing where the defect is located.

---

## Quick Reference — Input/Output Summary

### YOLO11-seg (`yolo11_seg_oring.onnx`)

```
Input:   "images"   [1, 3, 384, 512]  float32, RGB, [0-1]
Output:  "output0"  [1, 37, 4032]     float32   (detections)
         "output1"  [1, 32, 96, 128]  float32   (mask prototypes)
```

**Preprocessing**: 4×4 bin 2048×1536 → 512×384 → RGB → ÷ 255 (no letterbox)

### PatchCore (`patchcore_model*_cropped_resnet50.onnx`)

```
Input:   "image"          [1, 3, 384, 384]  float32, RGB, [0-1]
Output:  "anomaly_score"  [1]               float32
         "anomaly_map"    [1, 1, 384, 384]  float32
```

**Preprocessing**: Crop from YOLO mask bbox → Bicubic resize to 384×384 → RGB → ÷ 255

---

## NuGet Packages Required

```xml
<!-- CPU inference -->
<PackageReference Include="Microsoft.ML.OnnxRuntime" Version="1.21.*" />
<!-- OR GPU inference (recommended, requires CUDA 12 + cuDNN 9) -->
<PackageReference Include="Microsoft.ML.OnnxRuntime.Gpu" Version="1.21.*" />

<PackageReference Include="System.Drawing.Common" Version="8.0.*" />
```

> Do **not** install both CPU and GPU packages — choose one.

---

## Existing C# Reference Files

| File | Description |
|------|-------------|
| `PatchCoreDetector.cs` | Low-level ONNX Runtime wrapper for PatchCore |
| `PatchCoreInspectionService.cs` | Full preprocessing + inference + heatmap overlay |
| `PatchCoreMainWindow.xaml` | WPF UI layout |
| `PatchCoreMainWindow.xaml.cs` | WPF code-behind |

> **Note**: The C# files reference the previous 640×640 model. Update to **384×384** input, new model filenames (`*_cropped_*`), and new thresholds (29.76 / 30.60). Also remove the letterbox padding step — the YOLO model now takes 384×512 directly.
