using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Threading;
using Microsoft.ML.OnnxRuntime;
using Microsoft.ML.OnnxRuntime.Tensors;

namespace RoboViz;

/// <summary>
/// Result from a single Mask R-CNN inference pass.
/// </summary>
public class Detection
{
    public float X1 { get; set; }
    public float Y1 { get; set; }
    public float X2 { get; set; }
    public float Y2 { get; set; }
    public float Score { get; set; }
    public int Label { get; set; }
    public float[,] Mask { get; set; } = null!;
}

/// <summary>
/// Mask R-CNN ONNX inference wrapper for O-Ring defect detection.
/// Two classes: 0 = background, 1 = defect.
/// Attempts CUDA GPU with full pre-flight diagnostics; falls back to CPU automatically.
/// </summary>
public class MaskRCNNDetector : IDisposable
{
    private InferenceSession _session = null!;
    private string _inputName = null!;
    private readonly ThreadLocal<float[]> _inputBuffer;
    private const int InputSize = 720;
    private const int BufferLength = 1 * 3 * InputSize * InputSize;

    private static readonly string DiagLogPath = Path.Combine(
        AppDomain.CurrentDomain.BaseDirectory, "gpu_init.log");

    public string ActiveProvider { get; private set; } = "CPU";
    public string? GpuError { get; private set; }

    // Delegates resolved at runtime so missing DLLs don't crash the process
    private delegate int CuDriverGetVersionFn(out int version);
    private delegate int CudaRuntimeGetVersionFn(out int version);
    private delegate ulong CudnnGetVersionFn();

    public MaskRCNNDetector(string modelPath, bool useGpu = true, IProgress<string>? progress = null)
    {
        _inputBuffer = new ThreadLocal<float[]>(() => new float[BufferLength]);
        bool gpuReady = false;

        // Fresh diagnostic log each startup
        try { File.WriteAllText(DiagLogPath, $"=== MaskRCNNDetector GPU Init  {DateTime.Now:O} ===\n"); } catch { }

        if (useGpu)
        {
            try
            {
                gpuReady = TryInitGpu(modelPath, progress);
            }
            catch (Exception ex)
            {
                LogDiag($"TryInitGpu outer exception: {ex}");
                GpuError ??= ex.Message;
                gpuReady = false;
            }
        }

        if (!gpuReady)
        {
            InitCpu(modelPath, progress);
        }

        LogDiag($"=== Init complete. Provider: {ActiveProvider} ===\n");
    }

    // ─── GPU Initialization Pipeline ───────────────────────────────────

    /// <summary>
    /// Full GPU init: path setup → DLL probe → version check → session → warmup.
    /// Returns true only when every stage succeeds.
    /// </summary>
    private bool TryInitGpu(string modelPath, IProgress<string>? progress)
    {
        // Step 1 – Library paths
        progress?.Report("Setting up NVIDIA library paths...");
        LogDiag("[Step 1] Setting up NVIDIA library paths");
        EnsureNvidiaLibsOnPath();

        // Step 2 – Probe critical native DLLs (prevents native crash from missing libs)
        progress?.Report("Probing CUDA / cuDNN libraries...");
        LogDiag("[Step 2] Probing native libraries");
        if (!ProbeNativeLibraries())
        {
            GpuError = "Required CUDA/cuDNN DLLs could not be loaded – see gpu_init.log. "
                     + "Likely cause: Missing CUDA or cuDNN libraries. Ensure CUDA 12.x and cuDNN 9.x are installed.";
            progress?.Report("CUDA libraries not loadable – falling back to CPU...");
            return false;
        }

        // Step 3 – Version compatibility (WARNING only, not blocking for CUDA 12.6)
        LogDiag("[Step 3] Checking CUDA / cuDNN versions");
        CheckVersionCompatibility(); // Now just logs warnings, doesn't block

        // Step 4 – Create CUDA InferenceSession
        progress?.Report("Creating CUDA inference session (may take a moment)...");
        LogDiag("[Step 4] Creating SessionOptions + CUDA EP");

        var options = new SessionOptions
        {
            GraphOptimizationLevel = GraphOptimizationLevel.ORT_ENABLE_ALL,
            InterOpNumThreads = 2,
            IntraOpNumThreads = 2,
            EnableMemoryPattern = true,
        };

        try
        {
            var cudaOpts = new OrtCUDAProviderOptions();
            cudaOpts.UpdateOptions(new Dictionary<string, string>
            {
                ["device_id"]                   = "0",
                ["cudnn_conv_algo_search"]       = "DEFAULT",
                ["cudnn_conv_use_max_workspace"] = "1",
                ["do_copy_in_default_stream"]    = "1",
                ["arena_extend_strategy"]        = "kSameAsRequested",
            });
            options.AppendExecutionProvider_CUDA(cudaOpts);
            LogDiag("  CUDA EP appended to session options.");
        }
        catch (Exception ex)
        {
            LogDiag($"  AppendExecutionProvider_CUDA failed: {ex}");
            GpuError = ex.Message;
            return false;
        }

        try
        {
            LogDiag("  Creating InferenceSession with CUDA EP...");
            progress?.Report("Loading ONNX model with CUDA (this may take 10-30 seconds)...");
            _session = new InferenceSession(modelPath, options);
            _inputName = _session.InputMetadata.Keys.First();
            ActiveProvider = "CUDA (GPU)";
            LogDiag("  InferenceSession created OK.");
        }
        catch (Exception ex)
        {
            LogDiag($"  InferenceSession creation failed: {ex}");
            LogDiag($"  Exception type: {ex.GetType().Name}");
            LogDiag($"  Stack trace: {ex.StackTrace}");
            GpuError = $"Session creation failed: {ex.Message}";
            return false;
        }

        // Step 5 – Warmup (first real GPU execution)
        LogDiag("[Step 5] GPU warmup (may take 60+ seconds on PCIe Gen2)");
        if (!RunWarmUp(progress))
        {
            LogDiag("  Warmup failed – disposing CUDA session.");
            _session?.Dispose();
            _session = null!;
            return false;
        }

        return true;
    }

    /// <summary>Create a plain CPU session (fallback).</summary>
    private void InitCpu(string modelPath, IProgress<string>? progress)
    {
        LogDiag("Creating CPU-only session...");
        progress?.Report("Loading ONNX model on CPU...");

        var opts = new SessionOptions
        {
            GraphOptimizationLevel = GraphOptimizationLevel.ORT_ENABLE_ALL,
            InterOpNumThreads = 4,
            IntraOpNumThreads = 4,
            EnableMemoryPattern = true,
        };

        _session = new InferenceSession(modelPath, opts);
        _inputName = _session.InputMetadata.Keys.First();
        ActiveProvider = GpuError != null ? "CPU (GPU failed)" : "CPU";
        LogDiag($"CPU session ready. Provider = {ActiveProvider}");
    }

    /// <summary>
    /// Run 3 warm-up inferences on GPU with extended timeout for PCIe Gen2.
    /// Returns false if any iteration fails.
    /// </summary>
    private bool RunWarmUp(IProgress<string>? progress)
    {
        var buffer = _inputBuffer.Value!;
        var tensor = new DenseTensor<float>(buffer, [1, 3, InputSize, InputSize]);
        var inputs = new List<NamedOnnxValue>
        {
            NamedOnnxValue.CreateFromTensor(_inputName, tensor)
        };

        const int iterations = 3;
        for (int i = 0; i < iterations; i++)
        {
            string msg = $"GPU warmup {i + 1}/{iterations} (first run may take 60-90s on PCIe Gen2 with older CPU)...";
            progress?.Report(msg);
            LogDiag($"  {msg}");
            
            var sw = Stopwatch.StartNew();
            try
            {
                using var _ = _session.Run(inputs);
                sw.Stop();
                LogDiag($"  Warmup {i + 1}/{iterations} completed in {sw.ElapsedMilliseconds}ms.");
            }
            catch (Exception ex)
            {
                sw.Stop();
                LogDiag($"  Warmup {i + 1}/{iterations} FAILED after {sw.ElapsedMilliseconds}ms: {ex}");
                LogDiag($"  Exception type: {ex.GetType().Name}");
                LogDiag($"  Inner exception: {ex.InnerException?.Message}");
                ActiveProvider = "CPU (GPU warmup failed)";
                GpuError = $"Warmup failed: {ex.Message}";
                progress?.Report($"GPU warmup failed – falling back to CPU. {ex.Message}");
                return false;
            }
        }

        LogDiag("  Warmup complete.");
        return true;
    }

    // ─── Native-Library Diagnostics ────────────────────────────────────

    /// <summary>
    /// Try to load every DLL that the ONNX Runtime CUDA EP needs.
    /// Returns false if any critical library cannot be loaded (avoids native crash).
    /// </summary>
    private static bool ProbeNativeLibraries()
    {
        string[] critical = ["cudart64_12.dll", "cublas64_12.dll", "cublasLt64_12.dll", "cudnn64_9.dll"];
        string[] optional = ["cufft64_11.dll", "cudnn_cnn64_9.dll", "cudnn_ops64_9.dll",
                             "cudnn_graph64_9.dll", "cudnn_adv64_9.dll",
                             "cudnn_engines_runtime64_9.dll", "cudnn_engines_precompiled64_9.dll",
                             "cudnn_heuristic64_9.dll"];

        bool allCritical = true;

        foreach (var name in critical)
        {
            bool ok = NativeLibrary.TryLoad(name, out var h);
            LogDiag($"  [CRITICAL] {name}: {(ok ? "loaded" : "NOT FOUND")}");
            if (ok) NativeLibrary.Free(h); else allCritical = false;
        }

        foreach (var name in optional)
        {
            bool ok = NativeLibrary.TryLoad(name, out var h);
            LogDiag($"  [optional] {name}: {(ok ? "loaded" : "not found")}");
            if (ok) NativeLibrary.Free(h);
        }

        if (!allCritical)
        {
            LogDiag("  ──────────────────────────────────────────────────────────────");
            LogDiag("  DIAGNOSIS: One or more critical DLLs failed to load.");
            LogDiag("  Most common cause: cuDNN DLLs are compiled for a newer CUDA version");
            LogDiag("  than the CUDA Toolkit installed on this machine.");
            LogDiag("  Your cuDNN subfolder '12.9' requires CUDA 12.9+, but you have CUDA 12.6.");
            LogDiag("  FIX: Install CUDA Toolkit 12.9+ (can coexist with 12.6) or install");
            LogDiag("       an older cuDNN (e.g. 9.5.x) that supports CUDA 12.6.");
            LogDiag("  ──────────────────────────────────────────────────────────────");
        }

        return allCritical;
    }

    /// <summary>
    /// Query CUDA driver, CUDA runtime, and cuDNN versions dynamically.
    /// Warns if the loaded cuDNN requires a newer CUDA than what is installed.
    /// Returns true always now (non-blocking) - just logs warnings.
    /// </summary>
    private static bool CheckVersionCompatibility()
    {
        int driverVer = 0, runtimeVer = 0;
        ulong cudnnVer = 0;

        // CUDA Driver (nvcuda.dll is always present when an NVIDIA GPU is installed)
        if (NativeLibrary.TryLoad("nvcuda.dll", out var nvH))
        {
            try
            {
                if (NativeLibrary.TryGetExport(nvH, "cuDriverGetVersion", out var fn))
                {
                    var getVer = Marshal.GetDelegateForFunctionPointer<CuDriverGetVersionFn>(fn);
                    if (getVer(out driverVer) == 0)
                        LogDiag($"  CUDA Driver  : {driverVer / 1000}.{(driverVer % 1000) / 10}");
                }
            }
            finally { NativeLibrary.Free(nvH); }
        }
        else
        {
            LogDiag("  nvcuda.dll not found – no NVIDIA driver installed?");
        }

        // CUDA Runtime
        if (NativeLibrary.TryLoad("cudart64_12.dll", out var cudaH))
        {
            try
            {
                if (NativeLibrary.TryGetExport(cudaH, "cudaRuntimeGetVersion", out var fn))
                {
                    var getVer = Marshal.GetDelegateForFunctionPointer<CudaRuntimeGetVersionFn>(fn);
                    if (getVer(out runtimeVer) == 0)
                        LogDiag($"  CUDA Runtime : {runtimeVer / 1000}.{(runtimeVer % 1000) / 10}");
                }
            }
            finally { NativeLibrary.Free(cudaH); }
        }
        else
        {
            LogDiag("  cudart64_12.dll not loadable.");
        }

        // cuDNN
        if (NativeLibrary.TryLoad("cudnn64_9.dll", out var cudnnH))
        {
            try
            {
                if (NativeLibrary.TryGetExport(cudnnH, "cudnnGetVersion", out var fn))
                {
                    var getVer = Marshal.GetDelegateForFunctionPointer<CudnnGetVersionFn>(fn);
                    cudnnVer = getVer();
                    LogDiag($"  cuDNN        : {cudnnVer / 10000}.{(cudnnVer % 10000) / 100}.{cudnnVer % 100}");
                }
            }
            finally { NativeLibrary.Free(cudnnH); }
        }
        else
        {
            LogDiag("  cudnn64_9.dll not loadable.");
        }

        // Version compatibility check: Now just WARNING, not blocking
        // CUDA 12.6 should work with cuDNN 9.x in most cases
        if (runtimeVer > 0 && runtimeVer < 12006)
        {
            LogDiag("  ──────────────────────────────────────────────────────────────");
            LogDiag($"  ⚠ WARNING: CUDA Runtime {runtimeVer / 1000}.{(runtimeVer % 1000) / 10} detected.");
            LogDiag("  Some cuDNN operations may fail with CUDA < 12.6.");
            LogDiag("  Recommended: Install CUDA Toolkit 12.6 or newer.");
            LogDiag("  ──────────────────────────────────────────────────────────────");
        }
        else if (runtimeVer >= 12006 && runtimeVer < 12009)
        {
            LogDiag("  ──────────────────────────────────────────────────────────────");
            LogDiag($"  ✓ CUDA Runtime {runtimeVer / 1000}.{(runtimeVer % 1000) / 10} detected (12.6+).");
            LogDiag("  This should work with cuDNN 9.x, but if issues occur,");
            LogDiag("  consider upgrading to CUDA 12.9+ for optimal compatibility.");
            LogDiag("  ──────────────────────────────────────────────────────────────");
        }

        // Return true to allow GPU initialization to proceed
        return true;
    }

    // ─── NVIDIA PATH Setup ─────────────────────────────────────────────

    /// <summary>
    /// Prepend NVIDIA library paths (CUDA Toolkit, cuDNN system install, and
    /// pip-installed packages) to PATH so the native ORT CUDA provider can
    /// find them at runtime. Prioritizes newer CUDA versions.
    /// </summary>
    public static void EnsureNvidiaLibsOnPath()
    {
        var path = Environment.GetEnvironmentVariable("PATH") ?? "";
        var newPaths = new List<string>();

        void AddIfExists(string dir)
        {
            if (Directory.Exists(dir) && !path.Contains(dir, StringComparison.OrdinalIgnoreCase))
                newPaths.Add(dir);
        }

        // 1. CUDA Toolkit system install (e.g. C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\vX.Y\bin)
        string cudaPath = Environment.GetEnvironmentVariable("CUDA_PATH") ?? "";
        if (!string.IsNullOrEmpty(cudaPath))
        {
            AddIfExists(Path.Combine(cudaPath, "bin"));
            LogDiag($"  CUDA_PATH env: {cudaPath}");
        }
        
        // Scan for installed CUDA toolkit versions (prioritize newer versions)
        string toolkitRoot = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.ProgramFiles),
            "NVIDIA GPU Computing Toolkit", "CUDA");
        if (Directory.Exists(toolkitRoot))
        {
            var versions = new List<string>();
            foreach (var versionDir in Directory.GetDirectories(toolkitRoot)
                         .OrderByDescending(d => d))
            {
                string binDir = Path.Combine(versionDir, "bin");
                AddIfExists(binDir);
                versions.Add(Path.GetFileName(versionDir));
            }
            if (versions.Count > 0)
                LogDiag($"  Found CUDA versions: {string.Join(", ", versions)}");
        }

        // 2. cuDNN system install (e.g. C:\Program Files\NVIDIA\CUDNN\v9.x\bin\*\x64)
        string cudnnRoot = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.ProgramFiles),
            "NVIDIA", "CUDNN");
        if (Directory.Exists(cudnnRoot))
        {
            foreach (var versionDir in Directory.GetDirectories(cudnnRoot)
                         .OrderByDescending(d => d))
            {
                string binDir = Path.Combine(versionDir, "bin");
                if (Directory.Exists(binDir))
                {
                    // cuDNN may nest under bin\<cuda-version>\x64
                    // Prioritize folders matching CUDA 12.6 for compatibility
                    var subDirs = Directory.GetDirectories(binDir)
                                 .OrderByDescending(d => {
                                     var name = Path.GetFileName(d);
                                     // Prioritize 12.6, then 12.x versions
                                     if (name.StartsWith("12.6")) return 1000;
                                     if (name.StartsWith("12.")) return 100;
                                     return 0;
                                 });
                    
                    foreach (var sub in subDirs)
                    {
                        AddIfExists(Path.Combine(sub, "x64"));
                        AddIfExists(sub);
                    }
                    AddIfExists(binDir);
                }
            }
        }

        // 3. Anaconda / pip-installed NVIDIA packages
        string sitePackages = Path.Combine(
            Environment.GetFolderPath(Environment.SpecialFolder.UserProfile),
            "anaconda3", "Lib", "site-packages", "nvidia");

        if (Directory.Exists(sitePackages))
        {
            string[] libs = ["cudnn", "cublas", "cufft", "curand", "cusolver", "cusparse"];
            foreach (var lib in libs)
            {
                AddIfExists(Path.Combine(sitePackages, lib, "bin"));
            }
        }

        if (newPaths.Count > 0)
        {
            string combined = string.Join(';', newPaths) + ";" + path;
            Environment.SetEnvironmentVariable("PATH", combined);
        }

        foreach (var p in newPaths)
            LogDiag($"  Added to PATH: {p}");

        if (newPaths.Count == 0)
            LogDiag("  No new paths added.");
    }

    // ─── Inference ─────────────────────────────────────────────────────

    private const int MaxDetections = 5;

    /// <summary>
    /// Run inference on a 720×720 Bitmap image. Returns at most 5 detections.
    /// </summary>
    public List<Detection> Detect(Bitmap image, float scoreThreshold, out long tensorMs, out long inferenceMs)
    {
        var sw = Stopwatch.StartNew();
        var buffer = _inputBuffer.Value!;
        FillInputBuffer(image, buffer);
        var tensor = new DenseTensor<float>(buffer, [1, 3, image.Height, image.Width]);
        tensorMs = sw.ElapsedMilliseconds;

        var inputs = new List<NamedOnnxValue>
        {
            NamedOnnxValue.CreateFromTensor(_inputName, tensor)
        };

        sw.Restart();
        using var results = _session.Run(inputs);
        inferenceMs = sw.ElapsedMilliseconds;

        var scores = results.ElementAt(2).AsTensor<float>();
        int numDetections = (int)scores.Length;

        // Early exit if no detections above threshold
        var detections = new List<Detection>(MaxDetections);
        bool anyAbove = false;
        for (int i = 0; i < numDetections && detections.Count < MaxDetections; i++)
        {
            if (scores[i] >= scoreThreshold) { anyAbove = true; break; }
        }
        if (!anyAbove) return detections;

        var boxes = results.ElementAt(0).AsTensor<float>();
        var labels = results.ElementAt(1).AsTensor<long>();
        var masks = results.ElementAt(3).AsTensor<float>();

        int maskH = masks.Dimensions.Length >= 3 ? masks.Dimensions[2] : image.Height;
        int maskW = masks.Dimensions.Length >= 4 ? masks.Dimensions[3] : image.Width;
        int maskPlane = maskH * maskW;

        for (int i = 0; i < numDetections && detections.Count < MaxDetections; i++)
        {
            float score = scores[i];
            if (score < scoreThreshold)
                continue;

            var det = new Detection
            {
                X1 = boxes[i, 0],
                Y1 = boxes[i, 1],
                X2 = boxes[i, 2],
                Y2 = boxes[i, 3],
                Score = score,
                Label = (int)labels[i],
                Mask = new float[maskH, maskW]
            };

            // Copy mask using span for contiguous access
            for (int y = 0; y < maskH; y++)
            {
                int srcOffset = y * maskW;
                for (int x = 0; x < maskW; x++)
                    det.Mask[y, x] = masks[i, 0, y, x];
            }

            detections.Add(det);
        }

        return detections;
    }

    /// <summary>
    /// Fill buffer from Bitmap. BGR → RGB planar, [0..1]. Thread-safe (uses caller's buffer).
    /// </summary>
    private static void FillInputBuffer(Bitmap bmp, float[] buffer)
    {
        int w = bmp.Width;
        int h = bmp.Height;
        int planeSize = h * w;

        var rect = new Rectangle(0, 0, w, h);
        var bmpData = bmp.LockBits(rect, ImageLockMode.ReadOnly, PixelFormat.Format24bppRgb);
        try
        {
            int stride = bmpData.Stride;
            byte[] pixels = new byte[stride * h];
            Marshal.Copy(bmpData.Scan0, pixels, 0, pixels.Length);

            for (int y = 0; y < h; y++)
            {
                int rowOffset = y * stride;
                int yOffset = y * w;
                for (int x = 0; x < w; x++)
                {
                    int idx = rowOffset + x * 3;
                    buffer[yOffset + x] = pixels[idx + 2] / 255f;             // R
                    buffer[planeSize + yOffset + x] = pixels[idx + 1] / 255f;  // G
                    buffer[planeSize * 2 + yOffset + x] = pixels[idx] / 255f;  // B
                }
            }
        }
        finally
        {
            bmp.UnlockBits(bmpData);
        }
    }

    // ─── Logging ───────────────────────────────────────────────────────

    public static void LogDiag(string message)
    {
        try
        {
            File.AppendAllText(DiagLogPath, $"[{DateTime.Now:HH:mm:ss.fff}] {message}\n");
        }
        catch { }
    }

    public void Dispose()
    {
        _session?.Dispose();
        _inputBuffer?.Dispose();
    }
}
