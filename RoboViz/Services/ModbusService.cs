using System;
using System.Diagnostics;
using System.IO.Ports;
using System.Threading;
using NModbus;
using NModbus.Serial;

namespace RoboViz;

/// <summary>
/// Modbus RTU master over RS-485 (USB-to-RS485 converter).
/// Writes rejection coils after each inspection cycle.
///   Coil 0: CAM 1+2 rejection (1 = reject, 0 = both pass)
///   Coil 1: CAM 3+4 rejection (1 = reject, 0 = both pass)
///
/// Includes auto-reconnect: if the bus enters an error state (timeout,
/// partial frame), the serial port is flushed and the transport rebuilt
/// without releasing the COM port, keeping the port locked to this process.
/// </summary>
public class ModbusService : IDisposable
{
    private SerialPort? _port;
    private IModbusMaster? _master;
    private byte _slaveId;
    private string? _comPort;
    private int _baudRate;
    private readonly object _busLock = new();
    private bool _disposed;

    public bool IsConnected => _port?.IsOpen == true;
    public string? LastError { get; private set; }

    /// <summary>
    /// Open the serial port and create the Modbus RTU master.
    /// </summary>
    public bool Connect(string comPort, int baudRate, byte slaveId)
    {
        Disconnect();
        _slaveId = slaveId;
        _comPort = comPort;
        _baudRate = baudRate;

        try
        {
            MaskRCNNDetector.LogDiag($"[Modbus] Opening serial port {comPort}...");
            _port = new SerialPort(comPort)
            {
                BaudRate = baudRate,
                DataBits = 8,
                Parity = Parity.None,
                StopBits = StopBits.One,
                ReadTimeout = 1000,
                WriteTimeout = 1000,
            };
            _port.Open();
            MaskRCNNDetector.LogDiag($"[Modbus] Serial port opened successfully.");

            CreateMaster();

            LastError = null;
            MaskRCNNDetector.LogDiag($"[Modbus] Connected: {comPort} @ {baudRate} baud, slave {slaveId}");
            Debug.WriteLine($"[Modbus] Connected: {comPort} @ {baudRate} baud, slave {slaveId}");
            return true;
        }
        catch (Exception ex)
        {
            LastError = ex.Message;
            MaskRCNNDetector.LogDiag($"[Modbus] Connect failed: {ex}");
            Debug.WriteLine($"[Modbus] Connect FAILED: {ex.Message}");
            Disconnect();
            return false;
        }
    }

    /// <summary>
    /// Create (or recreate) the NModbus master on the existing open serial port.
    /// </summary>
    private void CreateMaster()
    {
        // Dispose old master without closing the serial port
        try { _master?.Dispose(); } catch { }

        var factory = new ModbusFactory();
        _master = factory.CreateRtuMaster(new SerialPortAdapter(_port!));
        _master.Transport.ReadTimeout = 500;
        _master.Transport.WriteTimeout = 500;
        _master.Transport.Retries = 1;
    }

    /// <summary>
    /// Flush serial buffers and rebuild the NModbus transport.
    /// Called after persistent timeouts to clear partial frames from the bus.
    /// The COM port stays open — only the transport layer is reset.
    /// </summary>
    public bool TryRecover()
    {
        lock (_busLock)
        {
            if (_port == null || !_port.IsOpen) return false;

            try
            {
                Debug.WriteLine("[Modbus] Recovering: flushing serial buffers...");
                _port.DiscardInBuffer();
                _port.DiscardOutBuffer();
                Thread.Sleep(50); // let the bus settle

                CreateMaster();

                Debug.WriteLine("[Modbus] Recovery complete: transport rebuilt.");
                MaskRCNNDetector.LogDiag("[Modbus] Bus recovered (flush + transport rebuild).");
                LastError = null;
                return true;
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"[Modbus] Recovery failed: {ex.Message}");
                MaskRCNNDetector.LogDiag($"[Modbus] Recovery failed: {ex.Message}");
                LastError = ex.Message;
                return false;
            }
        }
    }

    /// <summary>
    /// Full reconnect: close the serial port completely and reopen it.
    /// Use when <see cref="TryRecover"/> cannot fix the bus (driver-level issues).
    /// Thread-safe — acquires <c>_busLock</c>.
    /// </summary>
    public bool TryReconnect()
    {
        lock (_busLock)
        {
            if (_comPort == null) return false;

            Debug.WriteLine($"[Modbus] Full reconnect: closing {_comPort}...");
            MaskRCNNDetector.LogDiag($"[Modbus] Full reconnect on {_comPort}...");

            // Tear down everything
            try { _master?.Dispose(); } catch { }
            _master = null;

            if (_port != null)
            {
                try { _port.DiscardInBuffer(); } catch { }
                try { _port.DiscardOutBuffer(); } catch { }
                try { _port.Close(); } catch { }
                try { _port.Dispose(); } catch { }
                _port = null;
            }

            // Wait for the OS to fully release the COM port handle
            Thread.Sleep(500);

            try
            {
                _port = new SerialPort(_comPort)
                {
                    BaudRate = _baudRate,
                    DataBits = 8,
                    Parity = Parity.None,
                    StopBits = StopBits.One,
                    ReadTimeout = 1000,
                    WriteTimeout = 1000,
                };
                _port.Open();
                CreateMaster();

                LastError = null;
                Debug.WriteLine($"[Modbus] Full reconnect succeeded on {_comPort}.");
                MaskRCNNDetector.LogDiag($"[Modbus] Full reconnect OK: {_comPort} @ {_baudRate}.");
                return true;
            }
            catch (Exception ex)
            {
                LastError = ex.Message;
                Debug.WriteLine($"[Modbus] Full reconnect FAILED: {ex.Message}");
                MaskRCNNDetector.LogDiag($"[Modbus] Reconnect failed: {ex.Message}");

                // Clean up partial state
                try { _master?.Dispose(); } catch { }
                _master = null;
                try { _port?.Close(); } catch { }
                try { _port?.Dispose(); } catch { }
                _port = null;

                return false;
            }
        }
    }

    public void Disconnect()
    {
        lock (_busLock)
        {
            // Dispose master first — NModbus may try to close the port internally,
            // so we must dispose it before we touch the SerialPort.
            var master = _master;
            _master = null;
            try { master?.Dispose(); } catch { }

            if (_port != null)
            {
                var port = _port;
                _port = null;

                try { port.DiscardInBuffer(); } catch { }
                try { port.DiscardOutBuffer(); } catch { }

                try
                {
                    if (port.IsOpen)
                        port.Close();
                }
                catch (Exception ex)
                {
                    Debug.WriteLine($"[Modbus] Disconnect port close error: {ex.Message}");
                }

                try { port.Dispose(); } catch { }
            }
        }

        Debug.WriteLine("[Modbus] Disconnected.");
    }

    /// <summary>
    /// Write rejection results to Modbus coils.
    ///   coilAddress+0: CAM 1+2 — true (1) if either is not PASS
    ///   coilAddress+1: CAM 3+4 — true (1) if either is not PASS
    /// </summary>
    public bool WriteRejectionCoils(bool cam12Reject, bool cam34Reject, ushort coilAddress = 0)
    {
        if (_master == null)
        {
            LastError = "Not connected";
            return false;
        }

        try
        {
            lock (_busLock)
            {
                _master.WriteMultipleCoils(_slaveId, coilAddress, [cam12Reject, cam34Reject]);
            }
            LastError = null;
            MaskRCNNDetector.LogDiag(
                $"[Modbus] Wrote coils: CAM1+2={cam12Reject}, CAM3+4={cam34Reject}");
            return true;
        }
        catch (Exception ex)
        {
            LastError = ex.Message;
            MaskRCNNDetector.LogDiag($"[Modbus] Write failed: {ex.Message}");
            return false;
        }
    }

    /// <summary>
    /// Write a single coil value.
    /// </summary>
    public bool WriteSingleCoil(ushort coilAddress, bool value)
    {
        if (_master == null)
        {
            LastError = "Not connected";
            return false;
        }

        try
        {
            lock (_busLock)
            {
                _master.WriteSingleCoil(_slaveId, coilAddress, value);
            }
            LastError = null;
            return true;
        }
        catch (Exception ex)
        {
            LastError = ex.Message;
            return false;
        }
    }

    /// <summary>
    /// Read coils (FC 01) from the slave. Returns null on error.
    /// </summary>
    public bool[]? ReadCoils(ushort startAddress, ushort count)
    {
        if (_master == null) { LastError = "Not connected"; return null; }

        try
        {
            bool[] result;
            lock (_busLock)
            {
                result = _master.ReadCoils(_slaveId, startAddress, count);
            }
            LastError = null;
            return result;
        }
        catch (Exception ex)
        {
            LastError = ex.Message;
            return null;
        }
    }

    /// <summary>
    /// List available COM ports on this machine.
    /// </summary>
    public static string[] GetAvailablePorts() => SerialPort.GetPortNames();

    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;
        Disconnect();
    }
}
