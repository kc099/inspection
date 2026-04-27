# RoboViz Performance Analysis

**Date:** 2025-01-XX  
**System:** .NET 10, WPF, ONNX Runtime, OpenCvSharp  
**Focus:** Trigger processing timing and thread safety for parallelization

---

## 📊 CURRENT PERFORMANCE BASELINE

### Conveyor 1 (Single Camera - CAM 1)
```
Total processing time: ~1 second
├─ Frame capture:     ~10-50ms
├─ Geometry (Measure): ~100-120ms
└─ MaskRCNN inference: ~800-900ms
```

### Conveyor 2 (Three Cameras - CAM 2, 3, 4)
```
Total processing time: ~3 seconds (SEQUENTIAL)
├─ CAM 4 [PatchCore]:  693ms  (geo: 0ms ✓, infer: 593ms)
├─ CAM 3 [PatchCore]:  332ms  (geo: 0ms ✓, infer: 294ms)
└─ CAM 2 [MaskRCNN]:  1962ms  (geo: 1518ms ❌, infer: 360ms)
─────────────────────────────────────────────────
Sequential sum:        2987ms
Parallel potential:    max(693, 332, 1962) = 1962ms
Potential speedup:     2987 ÷ 1962 = 1.52× (34% faster if thread-safe)
```

---

## 🔍 ROOT CAUSE ANALYSIS: Why CAM 2 Geometry Takes 1518ms

### CAM 1 Geometry (`Measure` → `BuildMask`) - **~120ms**

**Algorithm:**
```csharp
1. Convert to grayscale
2. Threshold: |pixel - bgValue| > threshold  (simple)
3. Morphology:
   ├─ Close: 7×7 ellipse kernel, 2 iterations
   └─ Open:  7×7 ellipse kernel, 1 iteration
4. Connected components: pick largest by area
5. FindContours: RETR_CCOMP (2-level hierarchy)
6. Fit circles via least-squares
7. Calculate circularity metrics
```

**Computational Cost:** LOW  
- Kernel size: 7×7 = 49 pixels per operation  
- Simple threshold (1 comparison per pixel)  
- Fast morphology (small kernels, few iterations)

---

### CAM 2 Geometry (`MeasureCam2` → `BuildMaskCam2`) - **~1518ms**

**Algorithm (washer_inspector.py methodology):**
```csharp
1. Load background image (cached)
2. Convert to grayscale
3. Ratio normalization:
   ├─ normalized = (image ÷ background) × 128
   └─ COST: Floating-point division PER PIXEL (2448×2048 = 5,013,504 fp divisions!)
4. Deviation map: |normalized - 128|
5. GaussianBlur:
   ├─ Kernel: 21×21, σ=4
   └─ COST: 441 multiply-accumulate per pixel (~2.2 billion FP ops)
6. AdaptiveThreshold: ⚠️ BOTTLENECK
   ├─ Block size: 151×151 = 22,801 pixels per output pixel
   ├─ Type: GaussianC (weighted average per block)
   └─ COST: ~114 BILLION floating-point operations (22,801 × 5,013,504)
7. Morphology (LARGE kernels):
   ├─ Close: 35×35 ellipse, 3 iterations (1,225 pixels/op × 3)
   └─ Open:  15×15 ellipse, 1 iteration (225 pixels/op × 1)
8. Connected components analysis:
   ├─ Find ALL components
   ├─ For each component:
   │   ├─ Extract blob mask (Compare operation)
   │   ├─ FindContours (separate call per component)
   │   ├─ Calculate circularity = 4π×area ÷ perimeter²
   │   └─ Score = circularity × area
   └─ Pick best component by score
9. Hole filling logic:
   ├─ FloodFill from (0,0)
   ├─ Invert to get all holes
   ├─ ConnectedComponents on holes
   ├─ Fill holes < 20% of component area
   └─ Keep central hole (o-ring inner circle)
10. FindContours: RETR_CCOMP (final outer + inner)
11. Fit circles via least-squares
12. Calculate circularity metrics
```

**Computational Cost:** VERY HIGH  
- **AdaptiveThreshold 151×151:** ~114 billion FP operations  
- **GaussianBlur 21×21:** ~2.2 billion FP operations  
- **Ratio normalization:** ~5 million FP divisions  
- **Morphology 35×35 (3×):** ~18 million pixel operations  
- **Multiple FindContours calls:** Component-by-component evaluation  

**Total complexity:** O(n²) where n = image dimension  
**Why so complex?** CAM 2 is a top-view camera with challenging lighting conditions. The background ratio normalization and adaptive thresholding compensate for uneven illumination and reflections on the metal washer surface.

---

## ⚠️ THREAD SAFETY ANALYSIS

### ❌ **CRITICAL: NOT THREAD-SAFE (Cannot Parallelize Without Changes)**

#### 1. **InspectionService Shared State** ⚠️ BLOCKER
```csharp
public class InspectionService : IDisposable
{
    private readonly MaskRCNNDetector _maskrcnn;  // ❌ Shared across threads
    // ...
}
```
**Problem:** Single `_maskrcnn` instance shared by all cameras.  
**Impact:** Parallel calls to `InspectMaskRCNN()` would race on ONNX session.  

---

#### 2. **MaskRCNNDetector ONNX Runtime** ✅ THREAD-SAFE (with ThreadLocal)
```csharp
public class MaskRCNNDetector : IDisposable
{
    private InferenceSession _session = null!;  // ✅ Thread-safe (ONNX v1.15+)
    private readonly ThreadLocal<float[]> _inputBuffer;  // ✅ ThreadLocal (safe)
}
```
**Status:** The ONNX `InferenceSession` is thread-safe for concurrent `Run()` calls (since ONNX Runtime 1.15).  
**Buffer:** `ThreadLocal<float[]>` ensures each thread gets its own input buffer.  
**Verdict:** ✅ Safe IF each thread has its own `MaskRCNNDetector` instance OR if we use a thread-safe pooling pattern.

---

#### 3. **OpenCV Operations** ⚠️ PARTIALLY THREAD-SAFE
```csharp
static OringMeasurement()
{
    Cv2.SetNumThreads(4);  // ⚠️ Global setting
}
```
**OpenCV Thread Safety:**
- ✅ Most OpenCV operations (Cv2.*) are thread-safe when operating on different `Mat` objects.  
- ❌ **Global state:** `Cv2.SetNumThreads()` sets a global OpenCV thread pool limit.  
- ⚠️ **Internal threading:** OpenCV may internally parallelize operations (e.g., GaussianBlur, AdaptiveThreshold). With `SetNumThreads(4)`, 3 parallel inspections could spawn up to 12 CPU threads → oversubscription on <12-core systems.

**Impact:**  
- Low-medium risk if system has ≥12 CPU cores.  
- High risk on 4-8 core systems: context switching overhead, cache thrashing.

---

#### 4. **CAM 2 Background Image Cache** ✅ THREAD-SAFE
```csharp
private static Mat? _cam2BgGray;  // Cached background image
private static readonly object _cam2BgLock = new();  // Lock for cache access

private static Mat? BuildMaskCam2(Mat gray)
{
    Mat? bg;
    lock (_cam2BgLock) { bg = _cam2BgGray; }  // ✅ Read-only access, safe
    // ...
}
```
**Status:** ✅ Read-only access to cached background image is thread-safe with lock.  
**Risk:** LOW (read-only, single lock, fast operation)

---

#### 5. **Bitmap Disposal Race Conditions** ⚠️ MEDIUM RISK
```csharp
// Dispose frames not used as overlay
for (int i = 0; i < frameList.Count; i++)
{
    if (!ReferenceEquals(frameList[i], results[i].OverlayImage))
        frameList[i].Dispose();  // ⚠️ Must ensure no concurrent access
}
```
**Problem:** If parallel threads modify `results[]` array concurrently, disposal logic could race.  
**Mitigation:** Use `Parallel.For` carefully, ensure each thread owns its index exclusively.  
**Risk:** MEDIUM (unlikely with proper index isolation, but needs defensive coding)

---

## 🚀 PARALLELIZATION OPTIONS

### Option A: **Parallel Inspection (Requires Refactoring)** 🔧
**Change required:** Create one `MaskRCNNDetector` instance per camera (or use `ConcurrentDictionary` pool).  

```csharp
// BEFORE (shared detector):
private readonly MaskRCNNDetector _maskrcnn;  // ❌ Single instance

// AFTER (per-camera detectors):
private readonly Dictionary<int, MaskRCNNDetector> _detectorsBySlot;  // ✅ One per camera
```

**Benefits:**  
- **Speedup:** 1.52× faster (2987ms → 1962ms for Conveyor 2)  
- **Throughput:** Higher items/hour  

**Risks:**  
- **GPU memory:** 3× ONNX sessions may exceed GPU VRAM (each session ~500MB-1GB)  
- **CPU load:** 3× parallel OpenCV + 3× ONNX inference on CPU fallback systems  
- **Development:** Refactoring `InspectionService` constructor, thread-safe error handling  

**Recommendation:** ⚠️ **High risk, moderate reward.** GPU VRAM is the likely blocker on your system. Need VRAM capacity check first.

---

### Option B: **Optimize CAM 2 Geometry (Low-Hanging Fruit)** 🎯
**Target:** Reduce CAM 2 geometry from 1518ms → ~400-600ms (still slower than CAM 1, but tolerable).

#### 1. **Reduce AdaptiveThreshold Block Size** (Biggest Win)
```csharp
// CURRENT: 151×151 block (22,801 pixels per output pixel)
Cv2.AdaptiveThreshold(devBlur, adaptive, 255,
    AdaptiveThresholdTypes.GaussianC,
    ThresholdTypes.Binary,
    blockSize: 151, c: -5);  // ❌ VERY expensive

// OPTIMIZED: 51×51 block (2,601 pixels per output pixel)
Cv2.AdaptiveThreshold(devBlur, adaptive, 255,
    AdaptiveThresholdTypes.GaussianC,
    ThresholdTypes.Binary,
    blockSize: 51, c: -5);  // ✅ ~10× faster (23K→2.6K pixels/op)
```
**Expected speedup:** 60-70% reduction (1518ms → ~450-600ms)  
**Risk:** May reduce contour detection accuracy in challenging lighting. **Requires testing with production data.**

#### 2. **Reduce GaussianBlur Kernel Size**
```csharp
// CURRENT: 21×21 kernel
Cv2.GaussianBlur(dev, devBlur, new OpenCvSharp.Size(21, 21), 4);  // ❌

// OPTIMIZED: 11×11 kernel
Cv2.GaussianBlur(dev, devBlur, new OpenCvSharp.Size(11, 11), 4);  // ✅ ~4× faster
```
**Expected speedup:** 50-100ms reduction  
**Risk:** Lower noise suppression → may increase false contours. Test required.

#### 3. **Use CUDA-Accelerated OpenCV** (If Available)
```csharp
// Requires OpenCvSharp CUDA build + GPU with CUDA compute capability
using var devBlurGpu = new GpuMat();
Cv2.Cuda.GaussianBlur(devGpu, devBlurGpu, new Size(21, 21), 4);
```
**Expected speedup:** 5-10× faster geometry (GPU offload)  
**Risk:** Requires OpenCvSharp.CUDA package, CUDA-capable GPU, complex build setup.  
**Status:** **Not implemented.** Would need build system changes.

---

### Option C: **Accept Current Performance** ✅
**Reality check:**  
- **Conveyor 2:** 3s processing time for 3 cameras  
- **Belt speed:** 143.2 mm/s × 3s = 429.6mm minimum object spacing  
- **Throughput:** ~1200 objects/hour (assuming 3s cycle time)  

**Is 3s acceptable?**  
- If belt runs at <4 items/second: ✅ Yes  
- If belt runs at >4 items/second: ❌ No (queue backlog, missed triggers)  

**User's conveyor specification:**  
- Conveyor 1: 112cm, 7.82s travel time → **~1 object every 8 seconds** ✅ **3s is fine**  

---

## 📋 RECOMMENDATIONS

### **Immediate Actions** (This Week)
1. ✅ **Add detailed timing logs** (COMPLETED - see updated `TriggerService.cs`)  
   - Per-camera frame capture time  
   - Per-camera inspection wall-clock time  
   - Sequential sum vs parallel potential calculation  
   - Example output:
     ```
     [Profiling] Frame capture done: 45.2ms | per-cam: [12ms, 18ms, 15ms]
     [Profiling] Inspection done: 2987.3ms | per-cam: [693ms, 332ms, 1962ms]
     [Profiling] Sequential sum: 2987ms | parallel potential: max(693, 332, 1962) = 1962ms (speedup: 1.5x)
     ```

2. ✅ **Document CAM 2 geometry complexity** (COMPLETED - see code comments)

3. ⚠️ **Test with production data:**  
   - Verify current 3s performance meets belt throughput requirements  
   - If acceptable: **STOP HERE** (no further optimization needed)  
   - If not acceptable: proceed to Step 4  

### **Optimization Path** (If Needed)
4. 🎯 **Try AdaptiveThreshold blockSize reduction:**  
   ```csharp
   // In OringMeasurement.cs BuildMaskCam2(), line 413:
   blockSize: 51,  // Change from 151
   ```
   - **Expected:** 1518ms → ~450-600ms  
   - **Test:** Run on 50-100 production images, verify contour detection accuracy  
   - **Risk:** LOW (easily revertible, no architecture changes)  

5. 🔧 **If still too slow:** Consider GaussianBlur kernel reduction (11×11)  

6. ⚠️ **Last resort:** Parallelization (requires GPU VRAM check + refactoring)  

---

## 🧪 TESTING CHECKLIST

Before deploying any optimization:
- [ ] Verify `AdaptiveThreshold` change detects contours on all test images  
- [ ] Check for increased "no contours found" errors in logs  
- [ ] Validate circularity/eccentricity measurements stay within thresholds  
- [ ] Confirm no regression in defect detection accuracy  
- [ ] Test on challenging images: reflective surfaces, uneven lighting, small defects  

---

## 📞 NEXT STEPS

**User, please confirm:**
1. **Is 3-second processing time acceptable for Conveyor 2?**  
   - Current belt: ~1 object/8 seconds → 3s is plenty of headroom ✅  
   - If yes: **No optimization needed**, deploy as-is.  

2. **If optimization needed:**  
   - Start with `AdaptiveThreshold` blockSize reduction (151 → 51)  
   - Test on production data first  
   - Report back on accuracy vs speed tradeoff  

3. **Parallelization:**  
   - ⚠️ **Not recommended** due to GPU VRAM constraints  
   - Only consider if geometry optimization insufficient  

---

**Updated:** 2025-01-XX  
**Status:** Timing profiling added ✅ | Geometry analysis complete ✅ | Awaiting user decision on optimization priority
