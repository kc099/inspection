using System;
using System.Diagnostics;
using System.Net.Http;
using System.Net.Http.Headers;
using System.Text;

namespace RoboViz;

/// <summary>
/// Writes logical outputs either through Modbus coils or HTTP POST endpoints.
/// </summary>
public sealed class OutputCommunicationService : IDisposable
{
    private readonly TriggerConfig _config;
    private readonly ModbusService? _modbus;
    private readonly HttpClient _httpClient;
    private bool _disposed;

    /// <summary>
    /// Timeout per HTTP POST. Framework default is 100 s which is far too long
    /// for a real-time conveyor loop — 3 s keeps the pipeline responsive if the
    /// PCB hotspot is temporarily unreachable.
    /// </summary>
    private static readonly TimeSpan HttpTimeout = TimeSpan.FromSeconds(3);

    public string CommunicationMode => _config.CommunicationMode;
    public string? LastError { get; private set; }

    public OutputCommunicationService(TriggerConfig config, ModbusService? modbus = null)
    {
        _config = config;
        _modbus = modbus;
        _httpClient = new HttpClient { Timeout = HttpTimeout };
    }

    public bool Write(OutputChannel channel, bool state)
    {
        if (string.Equals(_config.CommunicationMode, "http", StringComparison.OrdinalIgnoreCase))
            return WriteHttp(channel, state);

        return WriteModbus(channel, state);
    }

    private bool WriteModbus(OutputChannel channel, bool state)
    {
        if (_modbus == null)
        {
            LastError = "Modbus output transport is not configured.";
            return false;
        }

        ushort coil = channel switch
        {
            OutputChannel.ReadyT1 => _config.OutputCoils.ReadyCoil_T1,
            OutputChannel.ReadyT2 => _config.OutputCoils.ReadyCoil_T2,
            OutputChannel.Cam1Rework => _config.OutputCoils.Cam1_ReworkCoil,
            OutputChannel.Cam1Reject => _config.OutputCoils.Cam1_RejectCoil,
            OutputChannel.Cam234Reject => _config.OutputCoils.Cam234_RejectCoil,
            OutputChannel.Cam2Rework => _config.OutputCoils.Cam2_ReworkCoil,
            _ => 0,
        };

        if (coil == 0)
        {
            LastError = $"Modbus coil not configured for {channel}.";
            return false;
        }

        bool ok = _modbus.WriteSingleCoil(coil, state);
        LastError = ok ? null : _modbus.LastError;
        return ok;
    }

    private bool WriteHttp(OutputChannel channel, bool state)
    {
        string? url = _config.HttpOutputs.GetUrl(channel)?.Trim();
        if (string.IsNullOrWhiteSpace(url))
        {
            LastError = $"HTTP URL not configured for {channel}.";
            Debug.WriteLine($"[HTTP-OUT] {channel} value={(state ? 1 : 0)} FAILED: {LastError}");
            return false;
        }

        if (!Uri.TryCreate(url, UriKind.Absolute, out var uri))
        {
            LastError = $"Invalid HTTP URL for {channel}: {url}";
            Debug.WriteLine($"[HTTP-OUT] {channel} value={(state ? 1 : 0)} FAILED: {LastError}");
            return false;
        }

        try
        {
            using var content = new ByteArrayContent(Encoding.ASCII.GetBytes(state ? "1" : "0"));
            content.Headers.ContentType = new MediaTypeHeaderValue("text/plain");

            using var request = new HttpRequestMessage(HttpMethod.Post, uri)
            {
                Content = content,
            };

            using var response = _httpClient.SendAsync(request).GetAwaiter().GetResult();
            response.EnsureSuccessStatusCode();
            LastError = null;

            if (state)
                Debug.WriteLine($"[HTTP-OUT] SENT 1 OK -> {uri} ({channel})");
            else
                Debug.WriteLine($"[HTTP-OUT] SENT 0 OK -> {uri} ({channel})");

            return true;
        }
        catch (Exception ex)
        {
            LastError = ex.Message;
            Debug.WriteLine($"[HTTP-OUT] {channel} value={(state ? 1 : 0)} FAILED -> {uri}: {LastError}");
            return false;
        }
    }

    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;
        _httpClient.Dispose();
    }
}
