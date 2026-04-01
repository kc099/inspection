# O-Ring Inspection — ONNX Deployment Guide (WPF / C#)

Complete pipeline: **Camera capture → YOLO segmentation → Crop → PatchCore anomaly detection**.

---

## ONNX Models

| File | Size | Purpose |
|------|------|---------|
| `yolo11_seg_oring.onnx` | 85.5 MB | YOLO11m-seg — segments O-ring from background |
| `patchcore_model1_cropped_resnet50.onnx` | 181.3 MB | PatchCore — anomaly detection for Model 1 O-rings |
| `patchcore_model2_cropped_resnet50.onnx` | 181.9 MB | PatchCore — anomaly detection for Model 2 O-rings |

---

## Pipeline Overview

```
  Camera (2048×1536 BMP)
         │
    ┌────┴─────────────────────────────────┐
    │  Step 1: Resize to 640×480           │
    └────┬─────────────────────────────────┘
         │
    ┌────┴─────────────────────────────────┐
    │  Step 2: Letterbox pad to 640×640    │
    │          (80px black top + bottom)   │
    └────┬─────────────────────────────────┘
         │
    ┌────┴─────────────────────────────────┐
    │  Step 3: YOLO11-seg ONNX inference   │
    │  → segmentation mask of O-ring       │
    └────┬─────────────────────────────────┘
         │
    ┌────┴─────────────────────────────────┐
    │  Step 4: Extract bounding box of     │
    │  mask on the 640×480 resized image   │
    │  → crop that region (no padding)     │
    └────┬─────────────────────────────────┘
         │
    ┌────┴─────────────────────────────────┐
    │  Step 5: PatchCore preprocessing     │
    │  Resize crop → 640×640 (direct)      │
    │  Scale pixels to [0, 1]              │
    └────┬─────────────────────────────────┘
         │
    ┌────┴─────────────────────────────────┐
    │  Step 6: PatchCore ONNX inference    │
    │  → anomaly_score + anomaly_map       │
    └────┬─────────────────────────────────┘
         │
    ┌────┴─────────────────────────────────┐
    │  Step 7: Threshold decision          │
    │  score > threshold → DEFECT          │
    └──────────────────────────────────────┘
```

---

## Step-by-Step Preprocessing Details

### Step 1 — Resize Original Image

| Property | Value |
|----------|-------|
| Original size | 2048 × 1536 (from camera) |
| Target size | **640 × 480** |
| Method | Bilinear or Area interpolation |
| Color space | BGR (as captured) → **RGB** for both models |

```csharp
// C# — resize original 2048×1536 → 640×480
Bitmap resized = new Bitmap(original, new Size(640, 480));
```

---

### Step 2 — Letterbox for YOLO (640×480 → 640×640)

YOLO expects a **square** 640×640 input. The 640×480 image must be letterboxed (padded with black pixels).

| Property | Value |
|----------|-------|
| Image | 640 × 480 (from Step 1) |
| Padded size | **640 × 640** |
| Pad location | **80 px top, 80 px bottom** (centered vertically) |
| Pad color | Black (0, 0, 0) |

```
Before letterbox:          After letterbox:
┌──────────────┐           ┌──────────────┐
│              │           │  black (80px) │
│  640 × 480   │     →     ├──────────────┤
│              │           │  640 × 480    │
└──────────────┘           ├──────────────┤
                           │  black (80px) │
                           └──────────────┘
                              640 × 640
```

```csharp
// C# — letterbox 640×480 → 640×640
float[] inputTensor = new float[1 * 3 * 640 * 640]; // zero-initialized = black padding
int yOffset = 80; // (640 - 480) / 2

for (int y = 0; y < 480; y++)
{
    for (int x = 0; x < 640; x++)
    {
        Color pixel = resized.GetPixel(x, y);
        int yPad = y + yOffset;
        inputTensor[0 * 640 * 640 + yPad * 640 + x] = pixel.R / 255.0f;  // R channel
        inputTensor[1 * 640 * 640 + yPad * 640 + x] = pixel.G / 255.0f;  // G channel
        inputTensor[2 * 640 * 640 + yPad * 640 + x] = pixel.B / 255.0f;  // B channel
    }
}
```

---

### Step 3 — YOLO Segmentation Inference

| Property | Value |
|----------|-------|
| Model | `yolo11_seg_oring.onnx` |
| Input name | `images` |
| Input shape | `[1, 3, 640, 640]` float32 |
| Input format | **RGB**, scaled to **[0, 1]** (divide by 255) |
| Output 0 | `output0`: `[1, 37, 8400]` — detection boxes + class scores + mask coefficients |
| Output 1 | `output1`: `[1, 32, 160, 160]` — prototype mask features |
| Confidence threshold | 0.25 |

**Output parsing:**
- `output0` contains 8400 candidate detections. Each detection has 37 values:
  - `[0:4]` — bounding box (cx, cy, w, h) in 640×640 coordinates
  - `[4:5]` — class confidence (1 class: "oring")
  - `[5:37]` — 32 mask coefficients
- `output1` contains 32 prototype masks at 160×160 resolution
- To get the final mask: multiply mask coefficients × prototype masks, then sigmoid, then resize to 640×640
- **Important**: The bounding box and mask coordinates are in the 640×640 letterboxed space. Subtract the Y offset (80px) to map back to the 640×480 image.

---

### Step 4 — Crop O-Ring Region

After obtaining the segmentation mask, extract the tight bounding rectangle of the mask.

| Property | Value |
|----------|-------|
| Coordinate space | **640 × 480** (after removing letterbox offset) |
| Padding | **0 px** (exact mask bounding box, no extra padding) |
| Crop source | The 640×480 resized image (not the letterboxed one) |

```csharp
// Convert mask coordinates from 640×640 letterbox → 640×480
// Subtract yOffset=80 from Y coordinates

// Find bounding box of mask pixels
int x1 = maskMinX;      // leftmost mask pixel
int y1 = maskMinY - 80;  // topmost mask pixel (adjust for letterbox)
int x2 = maskMaxX;      // rightmost mask pixel
int y2 = maskMaxY - 80;  // bottommost mask pixel (adjust for letterbox)

// Clamp to image bounds
x1 = Math.Max(0, x1);
y1 = Math.Max(0, y1);
x2 = Math.Min(640, x2);
y2 = Math.Min(480, y2);

// Crop from the 640×480 resized image
Bitmap crop = resized.Clone(new Rectangle(x1, y1, x2 - x1, y2 - y1), resized.PixelFormat);
```

**Typical crop sizes:** ~370–520 × 280–470 px (varies per O-ring position).

---

### Step 5 — PatchCore Preprocessing (on the crop)

The variable-size crop is resized directly to 640×640. No data is lost.

| Step | Operation | Details |
|------|-----------|---------|
| 5a | **Resize** | Resize directly to **640 × 640** (bicubic interpolation) |
| 5b | **Scale** | Divide pixel values by **255.0** to get **[0, 1]** range |
| 5c | **Channel order** | **RGB** (not BGR) |
| 5d | **Tensor layout** | **CHW** — `[1, 3, 640, 640]` |

**ImageNet normalization is NOT needed** — it is already embedded inside the ONNX model.

```csharp
// 5a — Resize crop directly to 640×640
Bitmap finalCrop = new Bitmap(crop, new Size(640, 640));

// 5b + 5c + 5d — To float tensor [1, 3, 640, 640], RGB, [0-1]
float[] patchcoreInput = new float[1 * 3 * 640 * 640];
for (int y = 0; y < 640; y++)
{
    for (int x = 0; x < 640; x++)
    {
        Color pixel = finalCrop.GetPixel(x, y);
        patchcoreInput[0 * 640 * 640 + y * 640 + x] = pixel.R / 255.0f;
        patchcoreInput[1 * 640 * 640 + y * 640 + x] = pixel.G / 255.0f;
        patchcoreInput[2 * 640 * 640 + y * 640 + x] = pixel.B / 255.0f;
    }
}
```

---

### Step 6 — PatchCore Inference

| Property | Value |
|----------|-------|
| Model 1 | `patchcore_model1_cropped_resnet50.onnx` |
| Model 2 | `patchcore_model2_cropped_resnet50.onnx` |
| Input name | `image` |
| Input shape | `[1, 3, 640, 640]` float32 |
| Input format | **RGB**, **[0, 1]** scaled (no ImageNet normalization needed) |
| Output 0 | `anomaly_score`: `[1]` float32 — image-level anomaly score |
| Output 1 | `anomaly_map`: `[1, 1, 640, 640]` float32 — spatial heatmap |

---

### Step 7 — Threshold Decision

| Model | 2σ Threshold | Verdict |
|-------|-------------|---------|
| Model 1 | **22.71** | score > 22.71 → **DEFECT** |
| Model 2 | **23.70** | score > 23.70 → **DEFECT** |

The `anomaly_map` output can be used to generate a heatmap overlay showing where the defect is located.

---

## Quick Reference — Input/Output Summary

### YOLO11-seg (`yolo11_seg_oring.onnx`)

```
Input:   "images"   [1, 3, 640, 640]  float32, RGB, [0-1]
Output:  "output0"  [1, 37, 8400]     float32   (detections)
         "output1"  [1, 32, 160, 160] float32   (mask prototypes)
```

**Preprocessing**: Resize 2048×1536 → 640×480 → letterbox to 640×640 → RGB → ÷ 255

### PatchCore (`patchcore_model*_cropped_resnet50.onnx`)

```
Input:   "image"          [1, 3, 640, 640]  float32, RGB, [0-1]
Output:  "anomaly_score"  [1]               float32
         "anomaly_map"    [1, 1, 640, 640]  float32
```

**Preprocessing**: Crop from YOLO mask bbox → Resize directly to 640×640 → RGB → ÷ 255

---

## NuGet Packages Required

```xml
<PackageReference Include="Microsoft.ML.OnnxRuntime" Version="1.21.*" />
<!-- Optional: GPU acceleration -->
<PackageReference Include="Microsoft.ML.OnnxRuntime.Gpu" Version="1.21.*" />
```

---

## Existing C# Reference Files

| File | Description |
|------|-------------|
| `PatchCoreDetector.cs` | Low-level ONNX Runtime wrapper for PatchCore |
| `PatchCoreInspectionService.cs` | Full preprocessing + inference + heatmap overlay |
| `PatchCoreMainWindow.xaml` | WPF UI layout |
| `PatchCoreMainWindow.xaml.cs` | WPF code-behind |

> **Note**: These C# files were written for the previous model version. Update the model filenames and thresholds to match the current `*_cropped_*` ONNX files.
