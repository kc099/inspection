using System;
using System.Diagnostics;
using System.IO.Ports;
using NModbus;
using NModbus.Serial;

namespace RoboViz;

/// <summary>
/// Modbus RTU master over RS-485 (USB-to-RS485 converter).
/// Writes rejection coils after each inspection cycle.
///   Coil 0: CAM 1+2 rejection (1 = reject, 0 = both pass)
///   Coil 1: CAM 3+4 rejection (1 = reject, 0 = both pass)
/// </summary>
public class ModbusService : IDisposable
{
    private SerialPort? _port;
    private IModbusMaster? _master;
    private byte _slaveId;
    private readonly object _busLock = new();

    public bool IsConnected => _port?.IsOpen == true;
    public string? LastError { get; private set; }

    /// <summary>
    /// Open the serial port and create the Modbus RTU master.
    /// </summary>
    public bool Connect(string comPort, int baudRate, byte slaveId)
    {
        Disconnect();
        _slaveId = slaveId;

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

            var factory = new ModbusFactory();
            _master = factory.CreateRtuMaster(new SerialPortAdapter(_port));
            _master.Transport.ReadTimeout = 200;
            _master.Transport.WriteTimeout = 200;
            _master.Transport.Retries = 0;

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

    public void Disconnect()
    {
        _master?.Dispose();
        _master = null;

        if (_port?.IsOpen == true)
        {
            try { _port.Close(); } catch { }
        }
        _port?.Dispose();
        _port = null;
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
        Disconnect();
    }
}
