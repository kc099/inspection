using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Threading;
using MvCameraControl;

namespace RoboViz;

/// <summary>
/// Manages multiple HikRobot cameras for streaming acquisition.
/// Each camera runs in continuous mode on its own grab thread.
/// The latest frame from each camera is always available via <see cref="GetLatestFrame"/>.
/// </summary>
public class CameraManager : IDisposable
{
    /// <summary>
    /// Camera indices to open (0-based, in enumeration order).
    /// Change these to match your physical camera wiring.
    /// </summary>
    public static int[] CameraIndices { get; set; } = [0, 1, 2, 3];

    public int CameraCount => CameraIndices.Length;
    public bool IsStreaming { get; private set; }

    private readonly List<IDeviceInfo> _deviceInfoList = [];
    private readonly IDevice?[] _devices;
    private readonly Bitmap?[] _latestFrames;
    private readonly long[] _frameSequence; // Increments on every new frame
    private readonly object[] _frameLocks;
    private readonly Thread?[] _grabThreads;
    private readonly bool[] _grabbing;
    private bool _sdkInitialized;

    public CameraManager()
    {
        int count = CameraIndices.Length;
        _devices = new IDevice?[count];
        _latestFrames = new Bitmap?[count];
        _frameSequence = new long[count];
        _frameLocks = new object[count];
        _grabThreads = new Thread?[count];
        _grabbing = new bool[count];
        for (int i = 0; i < count; i++)
            _frameLocks[i] = new object();
    }

    /// <summary>
    /// Path to MVS native runtime DLLs.
    /// Change this if your MVS SDK is installed elsewhere.
    /// </summary>
    public static string MvsRuntimePath { get; set; } =
        @"C:\Program Files (x86)\Common Files\MVS\Runtime\Win64_x64";

    private static void EnsureMvsOnPath()
    {
        if (!Directory.Exists(MvsRuntimePath)) return;
        string path = Environment.GetEnvironmentVariable("PATH", EnvironmentVariableTarget.Process) ?? "";
        if (!path.Contains(MvsRuntimePath, StringComparison.OrdinalIgnoreCase))
        {
            Environment.SetEnvironmentVariable("PATH",
                MvsRuntimePath + ";" + path, EnvironmentVariableTarget.Process);
        }
    }

    public int Initialize()
    {
        if (!_sdkInitialized)
        {
            EnsureMvsOnPath();
            SDKSystem.Initialize();
            _sdkInitialized = true;
        }

        _deviceInfoList.Clear();
        var layerType = DeviceTLayerType.MvGigEDevice | DeviceTLayerType.MvUsbDevice
            | DeviceTLayerType.MvGenTLGigEDevice | DeviceTLayerType.MvGenTLCXPDevice
            | DeviceTLayerType.MvGenTLCameraLinkDevice | DeviceTLayerType.MvGenTLXoFDevice;

        int result = DeviceEnumerator.EnumDevices(layerType, out var deviceList);
        if (result != MvError.MV_OK)
            throw new InvalidOperationException($"EnumDevices failed: 0x{result:X}");

        _deviceInfoList.AddRange(deviceList);
        return _deviceInfoList.Count;
    }

    /// <summary>
    /// Get descriptions of all enumerated cameras.
    /// </summary>
    public List<string> GetCameraDescriptions()
    {
        var descriptions = new List<string>();
        for (int i = 0; i < _deviceInfoList.Count; i++)
        {
            var info = _deviceInfoList[i];
            string name = !string.IsNullOrEmpty(info.UserDefinedName)
                ? info.UserDefinedName
                : $"{info.ManufacturerName} {info.ModelName}";
            descriptions.Add($"[{i}] {info.TLayerType}: {name} ({info.SerialNumber})");
        }
        return descriptions;
    }

    /// <summary>
    /// Open all cameras specified in <see cref="CameraIndices"/> and start streaming.
    /// </summary>
    public void StartStreaming(IProgress<string>? progress = null)
    {
        if (IsStreaming) return;

        for (int slot = 0; slot < CameraIndices.Length; slot++)
        {
            int camIdx = CameraIndices[slot];
            if (camIdx < 0 || camIdx >= _deviceInfoList.Count)
            {
                progress?.Report($"CAM {slot + 1}: index {camIdx} not found (only {_deviceInfoList.Count} cameras)");
                continue;
            }

            progress?.Report($"Opening CAM {slot + 1} (index {camIdx})...");
            var device = DeviceFactory.CreateDevice(_deviceInfoList[camIdx]);
            int result = device.Open();
            if (result != MvError.MV_OK)
            {
                progress?.Report($"CAM {slot + 1}: Open failed 0x{result:X}");
                continue;
            }

            // Optimize GigE packet size
            if (device is IGigEDevice gigE)
            {
                if (gigE.GetOptimalPacketSize(out int packetSize) == MvError.MV_OK)
                    device.Parameters.SetIntValue("GevSCPSPacketSize", packetSize);
            }

            // Continuous acquisition, no trigger
            device.Parameters.SetEnumValueByString("AcquisitionMode", "Continuous");
            device.Parameters.SetEnumValueByString("TriggerMode", "Off");

            _devices[slot] = device;

            // Start grab thread
            _grabbing[slot] = true;
            int capturedSlot = slot;
            _grabThreads[slot] = new Thread(() => GrabThreadProc(capturedSlot))
            {
                IsBackground = true,
                Name = $"CamGrab_{slot}"
            };
            _grabThreads[slot]!.Start();

            // Start streaming on camera
            result = device.StreamGrabber.StartGrabbing();
            if (result != MvError.MV_OK)
            {
                _grabbing[slot] = false;
                progress?.Report($"CAM {slot + 1}: StartGrabbing failed 0x{result:X}");
                continue;
            }

            progress?.Report($"CAM {slot + 1}: streaming");
        }

        IsStreaming = true;
    }

    /// <summary>
    /// Stop streaming on all cameras and close devices.
    /// </summary>
    public void StopStreaming()
    {
        if (!IsStreaming) return;

        // Signal all threads to stop
        for (int i = 0; i < _grabbing.Length; i++)
            _grabbing[i] = false;

        // Wait for threads to finish
        for (int i = 0; i < _grabThreads.Length; i++)
        {
            _grabThreads[i]?.Join(2000);
            _grabThreads[i] = null;
        }

        // Stop grabbing and close devices
        for (int i = 0; i < _devices.Length; i++)
        {
            if (_devices[i] != null)
            {
                try { _devices[i]!.StreamGrabber.StopGrabbing(); } catch { }
                try { _devices[i]!.Close(); } catch { }
                try { _devices[i]!.Dispose(); } catch { }
                _devices[i] = null;
            }
        }

        IsStreaming = false;
    }

    /// <summary>
    /// Get the latest captured frame from a camera slot.
    /// Returns null if no frame is available yet.
    /// </summary>
    public Bitmap? GetLatestFrame(int slot)
    {
        if (slot < 0 || slot >= _latestFrames.Length) return null;
        lock (_frameLocks[slot])
        {
            return _latestFrames[slot] != null ? new Bitmap(_latestFrames[slot]!) : null;
        }
    }

    /// <summary>
    /// Get the latest frame with its sequence number. Returns (null, 0) if no frame available.
    /// Allows caller to detect if frame changed since last call.
    /// </summary>
    public (Bitmap? frame, long sequence) GetLatestFrameWithSequence(int slot)
    {
        if (slot < 0 || slot >= _latestFrames.Length) return (null, 0);
        lock (_frameLocks[slot])
        {
            return (_latestFrames[slot] != null ? new Bitmap(_latestFrames[slot]!) : null, _frameSequence[slot]);
        }
    }

    /// <summary>
    /// Get latest frames from all camera slots. Null entries for cameras not yet ready.
    /// </summary>
    public Bitmap?[] GetAllLatestFrames()
    {
        var frames = new Bitmap?[CameraIndices.Length];
        for (int i = 0; i < frames.Length; i++)
            frames[i] = GetLatestFrame(i);
        return frames;
    }

    private void GrabThreadProc(int slot)
    {
        var device = _devices[slot];
        if (device == null) return;

        while (_grabbing[slot])
        {
            int result = device.StreamGrabber.GetImageBuffer(1000, out IFrameOut? frameOut);
            if (result == MvError.MV_OK && frameOut != null)
            {
                try
                {
                    var bmp = frameOut.Image.ToBitmap();
                    if (bmp != null)
                    {
                        lock (_frameLocks[slot])
                        {
                            _latestFrames[slot]?.Dispose();
                            _latestFrames[slot] = bmp;
                            _frameSequence[slot]++;
                        }
                    }
                }
                finally
                {
                    device.StreamGrabber.FreeImageBuffer(frameOut);
                }
            }
        }
    }

    public void Dispose()
    {
        StopStreaming();

        for (int i = 0; i < _latestFrames.Length; i++)
        {
            lock (_frameLocks[i])
            {
                _latestFrames[i]?.Dispose();
                _latestFrames[i] = null;
            }
        }

        if (_sdkInitialized)
        {
            try { SDKSystem.Finalize(); } catch { }
        }
    }
}
