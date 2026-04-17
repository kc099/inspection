# RoboViz – O-Ring Inspection System

WPF application for automated O-Ring defect detection using Mask R-CNN (ONNX) with CUDA GPU acceleration and OpenCV geometric measurement.

## Architecture

```
Load Image (2448×2048)
  ?
  ?? Geometric Measurement (OpenCvSharp)
  ?   ?? bg subtraction ? morphology ? contours ? fit circles
  ?   ?? 6 metrics: outer/inner radius, circularity, center distance, eccentricity
  ?
  ?? Threshold Evaluation
  ?   ?? REWORK? (radius, circularity out of range) ? stop
  ?   ?? REJECT? (center distance, eccentricity out of range) ? stop
  ?   ?? PASS? ? continue to Mask R-CNN
  ?
  ?? Mask R-CNN Defect Detection (ONNX Runtime + CUDA)
      ?? bin + crop to 720×720
      ?? GPU inference
      ?? REJECT if defects found
      ?? PASS if clean
```

## Prerequisites

### 1. .NET 10 SDK

Download and install from https://dotnet.microsoft.com/download/dotnet/10.0

Verify:
```
dotnet --version
```

### 2. NVIDIA GPU Driver

Install the latest **Game Ready** or **Studio** driver for your GPU.

| GPU | Min Driver | Download |
|-----|-----------|----------|
| RTX 5070 / 5080 / 5090 (Blackwell) | 572.xx+ | https://www.nvidia.com/Download/index.aspx |
| RTX 4060–4090 (Ada) | 528.xx+ | same link |
| RTX 3060–3090 (Ampere) | 472.xx+ | same link |

Verify:
```
nvidia-smi
```
You should see your GPU name, driver version, and CUDA version (12.x).

### 3. CUDA Toolkit 12.x

Download CUDA Toolkit **12.4 or later** from https://developer.nvidia.com/cuda-downloads

- Select: Windows ? x86_64 ? your Windows version ? exe (local)
- Install with default options
- The RTX 5070 (Blackwell / SM 120) requires CUDA 12.8+ for full native support, but CUDA 12.4+ works via compatibility mode

Verify:
```
nvcc --version
```
Should show `release 12.4` or later.

### 4. cuDNN 9 (via pip)

The ONNX Runtime GPU provider requires **cuDNN 9** at runtime. The easiest way to install it (no manual DLL management):

```
pip install nvidia-cudnn-cu12
```

This installs cuDNN 9 + cuBLAS 12 into your Python site-packages. The application automatically detects and adds these paths at startup (see `MaskRCNNDetector.EnsureNvidiaLibsOnPath()`).

**Alternative — system-wide install (recommended for deployment):**

1. Download cuDNN 9 from https://developer.nvidia.com/cudnn-downloads
2. Extract and copy DLLs to your CUDA toolkit bin folder:
   ```
   copy cudnn-windows-x86_64-9.*\bin\*.dll "C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.x\bin\"
   ```

### 5. Python / Anaconda (optional)

Only needed if using the `pip install nvidia-cudnn-cu12` method above. The app looks for cuDNN in:
```
%USERPROFILE%\anaconda3\Lib\site-packages\nvidia\cudnn\bin\
```

If your Anaconda/Python is installed elsewhere, update the path in `MaskRCNNDetector.EnsureNvidiaLibsOnPath()`.

## Build & Run

```bash
cd RoboViz
dotnet build
dotnet run
```

Or open `RoboViz.sln` in Visual Studio 2022 17.14+ and press F5.

## Project Files

| File | Description |
|------|-------------|
| `MaskRCNNDetector.cs` | ONNX Runtime GPU inference wrapper, CUDA/cuDNN setup |
| `OringMeasurement.cs` | OpenCvSharp geometric measurement (contours, circle fit) |
| `InspectionService.cs` | Full pipeline: measure ? evaluate ? detect ? verdict |
| `ThresholdConfig.cs` | JSON threshold loading, metric evaluation |
| `MainWindow.xaml/.cs` | WPF UI with metric table, verdict banner, model selector |
| `maskrcnn_oring.onnx` | Mask R-CNN model (copied to output on build) |
| `model1_tuned_thresholds.json` | Tuned thresholds for Model 1 |
| `model2_tuned_thresholds.json` | Tuned thresholds for Model 2 |

## NuGet Packages

| Package | Version | Purpose |
|---------|---------|---------|
| `Microsoft.ML.OnnxRuntime.Gpu` | 1.24.2 | ONNX inference with CUDA execution provider |
| `OpenCvSharp4` | 4.10.0 | OpenCV wrapper for geometric measurement |
| `OpenCvSharp4.runtime.win` | 4.10.0 | Native OpenCV binaries for Windows |

## GPU Compatibility

The ONNX Runtime CUDA EP supports all NVIDIA GPUs with **Compute Capability 5.0+**:

| GPU Family | Compute Capability | Status |
|------------|-------------------|--------|
| RTX 5070/5080/5090 (Blackwell) | SM 12.0 | ? Supported (CUDA 12.8+ recommended) |
| RTX 4060–4090 (Ada Lovelace) | SM 8.9 | ? Supported |
| RTX 3060–3090 (Ampere) | SM 8.6 | ? Supported |
| RTX 2060–2080 (Turing) | SM 7.5 | ? Supported |
| GTX 1060–1080 (Pascal) | SM 6.1 | ? Supported |

### RTX 5070 Notes

- Requires driver **572.xx+** and **CUDA 12.8+** for native Blackwell kernels
- CUDA 12.4 works via JIT recompilation (slightly slower first run)
- When CUDA 12.8+ toolkit is available, update the CUDA install accordingly
- If upgrading ONNX Runtime in the future, check the [ORT release notes](https://github.com/microsoft/onnxruntime/releases) for the matching CUDA/cuDNN versions

## Troubleshooting

### "CPU (CUDA unavailable)" shown in UI

The status bar now shows the actual error. Common causes:

1. **cuDNN not found** — run `pip install nvidia-cudnn-cu12` and restart the app
2. **CUDA toolkit not installed** — install CUDA 12.4+ from NVIDIA
3. **Driver too old** — update your GPU driver
4. **Wrong GPU** — integrated GPU selected; check `device_id` in `MaskRCNNDetector.cs`

### Slow first inference (~1s vs ~450ms steady state)

This is normal. cuDNN's `EXHAUSTIVE` algorithm search caches the optimal convolution strategy on the first run. Subsequent runs reuse the cached result. The 3-pass warmup at startup minimizes this effect.

### Typical Performance (RTX 3060 / CUDA 12.4)

| Phase | Time |
|-------|------|
| Geometric measurement | ~50 ms |
| Bin + crop to 720×720 | ~10 ms |
| Tensor preparation | ~5 ms |
| Mask R-CNN inference | ~400 ms |
| Overlay drawing | ~5 ms |
| **Total** | **~470 ms** |
