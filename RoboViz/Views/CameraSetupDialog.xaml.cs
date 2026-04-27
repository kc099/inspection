using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Controls;

namespace RoboViz;

public partial class CameraSetupDialog : Window
{
    private readonly List<string> _cameraDescriptions;
    private readonly ComboBox[] _combos;
    private readonly RadioButton[] _trig1;
    private readonly RadioButton[] _trig2;
    private readonly TextBox[] _delays;
    private readonly ushort _coilAddr1;
    private readonly ushort _coilAddr2;

    /// <summary>Result configs — populated on Start click, null if cancelled.</summary>
    public CameraSlotConfig[]? ResultConfigs { get; private set; }

    public CameraSetupDialog(List<string> cameraDescriptions, CameraSlotConfig[]? savedConfigs = null)
    {
        InitializeComponent();
        _cameraDescriptions = cameraDescriptions;

        _combos = [CmbCam0, CmbCam1, CmbCam2, CmbCam3];
        _trig1  = [Trig1_0, Trig1_1, Trig1_2, Trig1_3];
        _trig2  = [Trig2_0, Trig2_1, Trig2_2, Trig2_3];
        _delays = [TxtDelay0, TxtDelay1, TxtDelay2, TxtDelay3];

        // Load coil addresses from trigger config and label the radio buttons
        var trigCfg = TriggerConfig.Load();
        _coilAddr1 = trigCfg.TriggerCoil_Cam13;
        _coilAddr2 = trigCfg.TriggerCoil_Cam24;
        foreach (var rb in _trig1) rb.Content = _coilAddr1.ToString();
        foreach (var rb in _trig2) rb.Content = _coilAddr2.ToString();

        PopulateCameraList();
        PopulateCombos();

        if (savedConfigs != null)
            RestoreConfigs(savedConfigs);

        UpdateSummary();

        // Wire up change events for live summary updates
        foreach (var cmb in _combos) cmb.SelectionChanged += (_, _) => UpdateSummary();
        foreach (var rb in _trig1.Concat(_trig2)) rb.Checked += (_, _) => UpdateSummary();
    }

    private void PopulateCameraList()
    {
        if (_cameraDescriptions.Count == 0)
        {
            CameraListText.Text = "No cameras detected.";
            BtnStart.IsEnabled = false;
            return;
        }

        CameraListText.Text = string.Join("\n", _cameraDescriptions);
    }

    private void PopulateCombos()
    {
        for (int slot = 0; slot < 4; slot++)
        {
            _combos[slot].Items.Clear();
            _combos[slot].Items.Add("(none)");
            foreach (var desc in _cameraDescriptions)
                _combos[slot].Items.Add(desc);

            // Default: auto-assign by index if enough cameras
            _combos[slot].SelectedIndex = slot < _cameraDescriptions.Count ? slot + 1 : 0;
        }
    }

    private void RestoreConfigs(CameraSlotConfig[] configs)
    {
        foreach (var cfg in configs)
        {
            int s = cfg.Slot;
            if (s < 0 || s >= 4) continue;

            // Restore camera selection
            if (cfg.DeviceIndex >= 0 && cfg.DeviceIndex < _cameraDescriptions.Count)
                _combos[s].SelectedIndex = cfg.DeviceIndex + 1; // +1 for "(none)"
            else
                _combos[s].SelectedIndex = 0;

            // Detector is always MaskRCNN (PatchCore option removed)

            // Restore trigger
            if (cfg.TriggerGroup == 2)
                _trig2[s].IsChecked = true;
            else
                _trig1[s].IsChecked = true;

            // Restore delay (convert from µs to ms for UI display)
            int delayMs = (int)(cfg.TriggerDelayUs / 1000.0);
            _delays[s].Text = delayMs.ToString();
        }
    }

    private CameraSlotConfig[] CollectConfigs()
    {
        var configs = new CameraSlotConfig[4];
        for (int s = 0; s < 4; s++)
        {
            int comboIdx = _combos[s].SelectedIndex;
            // Parse delay from UI (in ms) and convert to µs for hardware trigger delay
            int delayMs = int.TryParse(_delays[s].Text.Trim(), out int d) ? d : 50;
            configs[s] = new CameraSlotConfig
            {
                Slot = s,
                DeviceIndex = comboIdx > 0 ? comboIdx - 1 : -1,
                Detector = "MaskRCNN",
                TriggerGroup = _trig2[s].IsChecked == true ? 2 : 1,
                TriggerDelayUs = delayMs * 1000.0,  // Convert ms to µs for camera hardware delay
                CaptureDelayMs = 0,                  // Retrieve frame immediately after hardware capture
                // CAM 3/4 (slots 2/3) are side-view cameras with no visible hole — resize only,
                // no geometric measurement. CAM 1/2 run their respective geo pipelines.
                SkipGeoMeasurement = (s >= 2),
            };
        }
        return configs;
    }

    private void UpdateSummary()
    {
        var configs = CollectConfigs();
        int assigned = configs.Count(c => c.DeviceIndex >= 0);
        int t1 = configs.Count(c => c.DeviceIndex >= 0 && c.TriggerGroup == 1);
        int t2 = configs.Count(c => c.DeviceIndex >= 0 && c.TriggerGroup == 2);

        SummaryText.Text = $"{assigned} camera(s) assigned  •  " +
            $"Coil {_coilAddr1}: {t1} cam(s)  •  Coil {_coilAddr2}: {t2} cam(s)  •  Detector: MaskRCNN";

        BtnStart.IsEnabled = assigned > 0;
    }

    private void Start_Click(object sender, RoutedEventArgs e)
    {
        // Validate: no duplicate device assignments
        var configs = CollectConfigs();
        var assignedDevices = configs.Where(c => c.DeviceIndex >= 0).Select(c => c.DeviceIndex).ToArray();
        if (assignedDevices.Length != assignedDevices.Distinct().Count())
        {
            MessageBox.Show("Each physical camera can only be assigned to one slot.",
                "Duplicate Assignment", MessageBoxButton.OK, MessageBoxImage.Warning);
            return;
        }

        ResultConfigs = configs;
        DialogResult = true;
    }

    private void Cancel_Click(object sender, RoutedEventArgs e)
    {
        DialogResult = false;
    }
}
