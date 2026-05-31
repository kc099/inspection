using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Threading;

namespace RoboViz;

public sealed class AsyncFrameLogDispatcher : IDisposable
{
    private readonly FrameLoggingService _writer;
    private readonly BlockingCollection<FrameLogItem> _queue;
    private readonly Thread _worker;
    private volatile bool _disposed;

    public AsyncFrameLogDispatcher(FrameLoggingConfig config)
    {
        var cfg = config ?? new FrameLoggingConfig();
        _writer = new FrameLoggingService(cfg);
        int capacity = cfg.QueueCapacity <= 0 ? 256 : cfg.QueueCapacity;
        _queue = new BlockingCollection<FrameLogItem>(capacity);

        _worker = new Thread(WorkerLoop)
        {
            IsBackground = true,
            Name = "FrameLogWorker"
        };
        _worker.Start();
    }

    public bool Enqueue(
        Bitmap imageToSave,
        int slot,
        string verdict,
        IEnumerable<string>? targetDirectories = null)
    {
        if (_disposed || imageToSave == null || !_writer.ShouldSave(verdict))
            return false;

        Bitmap imageCopy;
        try
        {
            imageCopy = new Bitmap(imageToSave);
        }
        catch
        {
            return false;
        }

        string[] dirs = targetDirectories?
            .Where(d => !string.IsNullOrWhiteSpace(d))
            .Distinct(StringComparer.OrdinalIgnoreCase)
            .ToArray() ?? [];

        var item = new FrameLogItem(imageCopy, slot, verdict, dirs);
        if (_queue.TryAdd(item))
            return true;

        item.Dispose();
        Debug.WriteLine("[FrameLog] Queue full, dropping log item.");
        return false;
    }

    private void WorkerLoop()
    {
        foreach (var item in _queue.GetConsumingEnumerable())
        {
            try
            {
                if (item.TargetDirectories.Length == 0)
                {
                    _writer.SaveFrame(item.Image, item.Slot, item.Verdict);
                }
                else
                {
                    foreach (string dir in item.TargetDirectories)
                        _writer.SaveFrame(item.Image, item.Slot, item.Verdict, dir);
                }
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"[FrameLog] Worker error: {ex.Message}");
            }
            finally
            {
                item.Dispose();
            }
        }
    }

    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;

        try { _queue.CompleteAdding(); } catch { }
        try { _worker.Join(2000); } catch { }

        while (_queue.TryTake(out var item))
            item.Dispose();

        _queue.Dispose();
        _writer.Dispose();
    }

    private sealed class FrameLogItem : IDisposable
    {
        public FrameLogItem(Bitmap image, int slot, string verdict, string[] targetDirectories)
        {
            Image = image;
            Slot = slot;
            Verdict = verdict;
            TargetDirectories = targetDirectories;
        }

        public Bitmap Image { get; }
        public int Slot { get; }
        public string Verdict { get; }
        public string[] TargetDirectories { get; }

        public void Dispose()
        {
            try { Image.Dispose(); } catch { }
        }
    }
}
