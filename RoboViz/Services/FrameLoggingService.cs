using System;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Threading;

namespace RoboViz;

public sealed class FrameLoggingService : IDisposable
{
    private readonly object _lock = new();
    private readonly FrameLoggingConfig _config;
    private readonly string _logsDirectory;
    private int _sequence;

    public FrameLoggingService(FrameLoggingConfig config)
    {
        _config = config ?? new FrameLoggingConfig();
        string configuredPath = string.IsNullOrWhiteSpace(_config.LogsDirectory) ? "logs" : _config.LogsDirectory;
        _logsDirectory = Path.IsPathRooted(configuredPath)
            ? configuredPath
            : Path.Combine(AppDomain.CurrentDomain.BaseDirectory, configuredPath);
        Directory.CreateDirectory(_logsDirectory);
    }

    public bool ShouldSave(string verdict)
    {
        if (!_config.Enabled) return false;
        if (!_config.SaveDefectsOnly) return true;
        return !string.Equals(verdict, "PASS", StringComparison.OrdinalIgnoreCase);
    }

    public string? SaveFrame(Bitmap frame, int slot, string verdict)
    {
        if (frame == null) return null;

        try
        {
            lock (_lock)
            {
                Directory.CreateDirectory(_logsDirectory);

                string ext = string.Equals(_config.ImageFormat, "png", StringComparison.OrdinalIgnoreCase)
                    ? "png"
                    : "jpg";
                string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss_fff");
                int seq = Interlocked.Increment(ref _sequence);
                string fileName = $"{timestamp}_{seq:D3}_cam{slot + 1}_{verdict}.{ext}";
                string fullPath = Path.Combine(_logsDirectory, fileName);

                if (ext == "png")
                {
                    frame.Save(fullPath, ImageFormat.Png);
                }
                else
                {
                    var jpgCodec = ImageCodecInfo.GetImageEncoders()
                        .FirstOrDefault(c => c.FormatID == ImageFormat.Jpeg.Guid);
                    if (jpgCodec == null)
                    {
                        frame.Save(fullPath, ImageFormat.Jpeg);
                    }
                    else
                    {
                        using var encParams = new EncoderParameters(1);
                        encParams.Param[0] = new EncoderParameter(Encoder.Quality, 90L);
                        frame.Save(fullPath, jpgCodec, encParams);
                    }
                }

                return fullPath;
            }
        }
        catch (Exception ex)
        {
            Debug.WriteLine($"[FrameLog] Save failed: {ex.Message}");
            return null;
        }
    }

    public void Dispose()
    {
    }
}
