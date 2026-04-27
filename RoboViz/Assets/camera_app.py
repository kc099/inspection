"""
Quad Hikrobot Camera Capture with Modbus Trigger
- 4 configurable camera slots with live preview
- Modbus RTU trigger on coils 3001 / 3002 (rising-edge detection)
- Per-slot camera selection, coil assignment, and capture delay
- Saves frames to four folders (pass one/two × side/top view)
Uses Hikrobot MVS SDK (MvCameraControl) for camera control.
"""

import sys
import os
import time
import ctypes
import numpy as np
from datetime import datetime, timedelta
from threading import Thread, Event

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QGridLayout, QLabel, QPushButton, QGroupBox, QSplitter, QMessageBox,
    QSizePolicy, QComboBox, QRadioButton, QButtonGroup, QSpinBox,
    QFormLayout, QScrollArea,
)
from PySide6.QtCore import Qt, QTimer, Signal, Slot
from PySide6.QtGui import QImage, QPixmap, QFont, QPalette, QColor

import serial.tools.list_ports
from pymodbus.client import ModbusSerialClient

# Hikrobot MVS SDK
from MvCameraControl_class import MvCamera
from CameraParams_header import *
from CameraParams_const import *
from MvErrorDefine_const import *
from PixelType_header import *

# ──────────────────────────────────────────────────────────────────────
#  Output folders
# ──────────────────────────────────────────────────────────────────────
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

SLOT_FOLDERS = [
    os.path.join(BASE_DIR, "pass one", "side view"),
    os.path.join(BASE_DIR, "pass one", "topview"),
    os.path.join(BASE_DIR, "pass two", "side view"),
    os.path.join(BASE_DIR, "pass two", "topview"),
]
SLOT_LABELS = [
    "Slot 1 – Pass 1 Side View",
    "Slot 2 – Pass 1 Top View",
    "Slot 3 – Pass 2 Side View",
    "Slot 4 – Pass 2 Top View",
]
SLOT_PREFIXES = ["p1_side", "p1_top", "p2_side", "p2_top"]

for _folder in SLOT_FOLDERS:
    os.makedirs(_folder, exist_ok=True)


# ──────────────────────────────────────────────────────────────────────
#  Helpers
# ──────────────────────────────────────────────────────────────────────
def frame_to_numpy(camera, data_buf, frame_info):
    n_need_size = frame_info.nWidth * frame_info.nHeight * 3
    img_buf = (ctypes.c_ubyte * n_need_size)()

    stConvertParam = MV_CC_PIXEL_CONVERT_PARAM_EX()
    stConvertParam.nWidth = frame_info.nWidth
    stConvertParam.nHeight = frame_info.nHeight
    stConvertParam.pSrcData = data_buf
    stConvertParam.nSrcDataLen = frame_info.nFrameLen
    stConvertParam.enSrcPixelType = frame_info.enPixelType
    stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed
    stConvertParam.pDstBuffer = img_buf
    stConvertParam.nDstBufferSize = n_need_size

    ret = camera.MV_CC_ConvertPixelTypeEx(stConvertParam)
    if ret != MV_OK:
        return None
    return np.frombuffer(img_buf, dtype=np.uint8).reshape(
        frame_info.nHeight, frame_info.nWidth, 3
    ).copy()


def auto_configure_gige_ip(device_info, ip_offset=200):
    gige_info = device_info.SpecialInfo.stGigEInfo
    cam_ip = gige_info.nCurrentIp
    adapter_ip = gige_info.nNetExport
    adapter_mask = gige_info.nCurrentSubNetMask

    if (cam_ip & adapter_mask) == (adapter_ip & adapter_mask):
        return True

    new_ip = (adapter_ip & adapter_mask) | (ip_offset & (~adapter_mask & 0xFFFFFFFF))
    new_mask = adapter_mask
    new_gateway = (adapter_ip & adapter_mask) | (1 & (~adapter_mask & 0xFFFFFFFF))

    cam = MvCamera()
    ret = cam.MV_CC_CreateHandle(device_info)
    if ret != MV_OK:
        return False
    ret = cam.MV_GIGE_ForceIpEx(new_ip, new_mask, new_gateway)
    cam.MV_CC_DestroyHandle()
    return ret == MV_OK


def ip_int_to_str(ip_int):
    return f"{(ip_int >> 24) & 0xFF}.{(ip_int >> 16) & 0xFF}.{(ip_int >> 8) & 0xFF}.{ip_int & 0xFF}"


def save_bgr_bmp(bgr_img, filepath):
    h, w, _ = bgr_img.shape
    row_size = (w * 3 + 3) & ~3
    pixel_data_size = row_size * h
    file_size = 54 + pixel_data_size

    padded = np.zeros((h, row_size), dtype=np.uint8)
    for y in range(h):
        row_bytes = bgr_img[h - 1 - y].tobytes()
        padded[y, : w * 3] = np.frombuffer(row_bytes, dtype=np.uint8)

    header = bytearray(54)
    header[0:2] = b"BM"
    header[2:6] = file_size.to_bytes(4, "little")
    header[10:14] = (54).to_bytes(4, "little")
    header[14:18] = (40).to_bytes(4, "little")
    header[18:22] = w.to_bytes(4, "little")
    header[22:26] = h.to_bytes(4, "little")
    header[26:28] = (1).to_bytes(2, "little")
    header[28:30] = (24).to_bytes(2, "little")
    header[34:38] = pixel_data_size.to_bytes(4, "little")

    with open(filepath, "wb") as f:
        f.write(header)
        f.write(padded.tobytes())


# ──────────────────────────────────────────────────────────────────────
#  Camera Worker – grabs frames in a background thread
# ──────────────────────────────────────────────────────────────────────
class CameraWorker(Thread):
    def __init__(self, camera, buf_size):
        super().__init__(daemon=True)
        self.camera = camera
        self.buf_size = buf_size
        self.stop_event = Event()
        self.latest_frame = None
        self.latest_meta = None

    def run(self):
        data_buf = (ctypes.c_ubyte * self.buf_size)()
        frame_info = MV_FRAME_OUT_INFO_EX()
        while not self.stop_event.is_set():
            ret = self.camera.MV_CC_GetOneFrameTimeout(
                data_buf, self.buf_size, frame_info, 1000
            )
            if ret == MV_OK:
                img = frame_to_numpy(self.camera, data_buf, frame_info)
                if img is not None:
                    self.latest_frame = img
                    self.latest_meta = {
                        "frame_num": int(frame_info.nFrameNum),
                        "trigger_index": int(frame_info.nTriggerIndex),
                        "host_ts": int(frame_info.nHostTimeStamp),
                        "dev_ts": (
                            (int(frame_info.nDevTimeStampHigh) << 32)
                            | int(frame_info.nDevTimeStampLow)
                        ),
                        "recv_perf_ns": time.perf_counter_ns(),
                        "recv_wall": datetime.now(),
                    }

    def stop(self):
        self.stop_event.set()


# ──────────────────────────────────────────────────────────────────────
#  Modbus Poller – reads coils 3001 & 3002, fires on rising edge
# ──────────────────────────────────────────────────────────────────────
class ModbusPoller(Thread):
    def __init__(self, port, baudrate, poll_interval_ms):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.poll_interval = poll_interval_ms / 1000.0
        self.stop_event = Event()
        self.prev = {3001: False, 3002: False}
        self.on_trigger = None      # callback(coil_address: int)
        self.on_status = None       # callback(coil_3001: bool, coil_3002: bool)
        self.on_error = None        # callback(message: str)
        self.connected = False

    def run(self):
        try:
            client = ModbusSerialClient(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1,
                stopbits=1,
                bytesize=8,
                parity="N",
            )
        except Exception as exc:
            if self.on_error:
                self.on_error(f"Modbus init error: {exc}")
            return

        self.connected = client.connect()
        if not self.connected:
            if self.on_error:
                self.on_error(f"Cannot open {self.port}")
            return

        while not self.stop_event.is_set():
            try:
                result = client.read_coils(address=3001, count=2, slave=1)
                if result.isError():
                    if self.on_status:
                        self.on_status(False, False)
                    time.sleep(self.poll_interval)
                    continue

                s1, s2 = bool(result.bits[0]), bool(result.bits[1])

                if self.on_status:
                    self.on_status(s1, s2)

                # Rising-edge detection
                if s1 and not self.prev[3001] and self.on_trigger:
                    self.on_trigger(3001)
                if s2 and not self.prev[3002] and self.on_trigger:
                    self.on_trigger(3002)

                self.prev[3001] = s1
                self.prev[3002] = s2
            except Exception as exc:
                print(f"[MODBUS] Poll error: {exc}")

            time.sleep(self.poll_interval)

        client.close()

    def stop(self):
        self.stop_event.set()


# ──────────────────────────────────────────────────────────────────────
#  Main Window
# ──────────────────────────────────────────────────────────────────────
class MainWindow(QMainWindow):
    # Signals for thread-safe UI updates
    sig_frame_0 = Signal(np.ndarray)
    sig_frame_1 = Signal(np.ndarray)
    sig_frame_2 = Signal(np.ndarray)
    sig_frame_3 = Signal(np.ndarray)
    sig_coil_status = Signal(bool, bool)
    sig_trigger = Signal(int)
    sig_modbus_error = Signal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Quad Camera – Modbus Trigger Capture")
        self.setMinimumSize(1280, 720)
        self.showMaximized()

        # Camera state
        self.available_cameras = []   # [(list_type, dev_idx, display_name, transport)]
        self._gige_list = None
        self._usb_list = None
        self.slot_cams = [None] * 4
        self.slot_workers = [None] * 4
        self.streaming = False
        self.capture_counter = 0
        self.last_frame_seen = [None] * 4
        self.slot_trigger_delay_ms = [0.0] * 4

        # Modbus state
        self.modbus_poller = None
        self._modbus_connected_shown = False

        self._build_ui()
        self._connect_signals()
        self._enumerate_cameras()

        self.frame_timer = QTimer(self)
        self.frame_timer.timeout.connect(self._refresh_frames)

    # ── UI ───────────────────────────────────────────────────────────
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(6, 6, 6, 6)

        feed_style = "background-color: #1e1e1e; color: #888;"

        # ── Left: 2×2 camera feed grid ──────────────────────────────
        feeds_widget = QWidget()
        self.feeds_grid = QGridLayout(feeds_widget)
        self.feeds_grid.setContentsMargins(0, 0, 0, 0)
        self.feeds_grid.setSpacing(6)

        self.frame_labels = []
        self.frame_groups = []
        for i in range(4):
            grp = QGroupBox(SLOT_LABELS[i])
            lay = QVBoxLayout(grp)
            lbl = QLabel("No Stream")
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setStyleSheet(feed_style)
            lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            lay.addWidget(lbl)
            self.frame_labels.append(lbl)
            self.frame_groups.append(grp)

        self._arrange_feed_panels([0, 1, 2, 3])

        # ── Right: control panel ────────────────────────────────────
        ctrl_scroll = QScrollArea()
        ctrl_scroll.setWidgetResizable(True)
        ctrl_inner = QWidget()
        ctrl = QVBoxLayout(ctrl_inner)
        ctrl.setContentsMargins(10, 10, 10, 10)
        ctrl.setSpacing(10)

        title = QLabel("Controls")
        title.setFont(QFont("Segoe UI", 16, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        ctrl.addWidget(title)

        # ── Camera Configuration ────────────────────────────────────
        cam_grp = QGroupBox("Camera Configuration")
        cam_lay = QVBoxLayout(cam_grp)

        self.cam_combos = []
        self.coil_btn_groups = []
        self.coil_buttons = []
        self.delay_spins = []
        self.delay_applied_labels = []

        for i in range(4):
            row = QWidget()
            rl = QHBoxLayout(row)
            rl.setContentsMargins(0, 2, 0, 2)

            lbl = QLabel(f"Slot {i + 1}:")
            lbl.setFixedWidth(48)

            combo = QComboBox()
            combo.addItem("-- None --")

            r1 = QRadioButton("3001")
            r2 = QRadioButton("3002")
            r1.setChecked(True)
            bg = QButtonGroup(self)
            bg.addButton(r1, 3001)
            bg.addButton(r2, 3002)
            self.coil_buttons.append((r1, r2))

            delay_spin = QSpinBox()
            delay_spin.setRange(0, 5000)
            delay_spin.setValue(0)
            delay_spin.setSuffix(" ms")
            delay_spin.setFixedWidth(90)

            applied_lbl = QLabel("Applied: -")
            applied_lbl.setMinimumWidth(120)

            rl.addWidget(lbl)
            rl.addWidget(combo, 1)
            rl.addWidget(r1)
            rl.addWidget(r2)
            rl.addWidget(QLabel("Delay:"))
            rl.addWidget(delay_spin)
            rl.addWidget(applied_lbl)

            cam_lay.addWidget(row)
            self.cam_combos.append(combo)
            self.coil_btn_groups.append(bg)
            self.delay_spins.append(delay_spin)
            self.delay_applied_labels.append(applied_lbl)

        self.btn_refresh = QPushButton("Refresh Cameras")
        self.btn_refresh.setStyleSheet(
            "QPushButton { padding: 8px; font-size: 13px; border-radius: 4px; "
            "background-color: #37474f; color: white; }"
            "QPushButton:hover { background-color: #455a64; }"
        )
        cam_lay.addWidget(self.btn_refresh)
        ctrl.addWidget(cam_grp)

        # ── Trigger Source Settings ────────────────────────────────
        trig_grp = QGroupBox("Trigger Source")
        trig_lay = QFormLayout(trig_grp)

        self.combo_trigger_mode = QComboBox()
        self.combo_trigger_mode.addItem("Hardware I/O (OptoIn)", "hardware")
        self.combo_trigger_mode.addItem("Modbus Coils (3001/3002)", "modbus")

        self.combo_trigger_line = QComboBox()
        self.combo_trigger_line.addItem("Line0", "Line0")
        self.combo_trigger_line.addItem("Line1", "Line1")
        self.combo_trigger_line.addItem("Line2", "Line2")
        self.combo_trigger_line.addItem("Line3", "Line3")

        trig_lay.addRow("Mode:", self.combo_trigger_mode)
        trig_lay.addRow("Input Line:", self.combo_trigger_line)
        ctrl.addWidget(trig_grp)

        # ── Modbus Settings ─────────────────────────────────────────
        mb_grp = QGroupBox("Modbus Settings")
        self.mb_grp = mb_grp
        mb_lay = QFormLayout(self.mb_grp)

        self.combo_port = QComboBox()
        self.combo_port.setEditable(True)
        self._populate_com_ports()

        self.combo_baud = QComboBox()
        for b in ["9600", "19200", "38400", "57600", "115200", "230400"]:
            self.combo_baud.addItem(b)
        self.combo_baud.setCurrentText("115200")

        self.spin_poll = QSpinBox()
        self.spin_poll.setRange(10, 5000)
        self.spin_poll.setValue(100)
        self.spin_poll.setSuffix(" ms")

        mb_lay.addRow("COM Port:", self.combo_port)
        mb_lay.addRow("Baud Rate:", self.combo_baud)
        mb_lay.addRow("Poll Interval:", self.spin_poll)

        # Coil status indicators
        coil_row = QWidget()
        cl = QHBoxLayout(coil_row)
        cl.setContentsMargins(0, 4, 0, 0)
        self.lbl_coil_3001 = QLabel("● 3001: OFF")
        self.lbl_coil_3001.setStyleSheet("color: #888; font-weight: bold;")
        self.lbl_coil_3002 = QLabel("● 3002: OFF")
        self.lbl_coil_3002.setStyleSheet("color: #888; font-weight: bold;")
        cl.addWidget(self.lbl_coil_3001)
        cl.addWidget(self.lbl_coil_3002)
        mb_lay.addRow("Coil Status:", coil_row)

        self.lbl_modbus_status = QLabel("Disconnected")
        self.lbl_modbus_status.setStyleSheet("color: #f44336;")
        mb_lay.addRow("Connection:", self.lbl_modbus_status)

        ctrl.addWidget(self.mb_grp)

        # ── Start / Stop ────────────────────────────────────────────
        btn_css = (
            "QPushButton { padding: 14px; font-size: 15px; font-weight: bold; "
            "border-radius: 6px; }"
            "QPushButton:disabled { background-color: #555; color: #999; }"
        )

        self.btn_start = QPushButton("▶  Start")
        self.btn_start.setStyleSheet(
            btn_css
            + "QPushButton:enabled { background-color: #2e7d32; color: white; }"
            "QPushButton:enabled:hover { background-color: #388e3c; }"
        )
        ctrl.addWidget(self.btn_start)

        self.btn_stop = QPushButton("⏹  Stop")
        self.btn_stop.setEnabled(False)
        self.btn_stop.setStyleSheet(
            btn_css
            + "QPushButton:enabled { background-color: #c62828; color: white; }"
            "QPushButton:enabled:hover { background-color: #d32f2f; }"
        )
        ctrl.addWidget(self.btn_stop)

        self.lbl_count = QLabel("Captures: 0")
        self.lbl_count.setFont(QFont("Segoe UI", 13))
        self.lbl_count.setAlignment(Qt.AlignCenter)
        ctrl.addWidget(self.lbl_count)

        profile_grp = QGroupBox("Timing Profile")
        profile_lay = QVBoxLayout(profile_grp)
        self.lbl_prof_signal = QLabel("Signal: -")
        self.lbl_prof_acq = QLabel("Acquisition: -")
        self.lbl_prof_latency = QLabel("Latency: -")
        profile_lay.addWidget(self.lbl_prof_signal)
        profile_lay.addWidget(self.lbl_prof_acq)
        profile_lay.addWidget(self.lbl_prof_latency)
        ctrl.addWidget(profile_grp)

        # Save paths
        paths_grp = QGroupBox("Save Locations")
        pl = QVBoxLayout(paths_grp)
        for i in range(4):
            pl.addWidget(QLabel(f"Slot {i + 1} → {SLOT_FOLDERS[i]}"))
        ctrl.addWidget(paths_grp)

        ctrl.addStretch()

        ctrl_scroll.setWidget(ctrl_inner)

        # ── Splitter 70/30 ──────────────────────────────────────────
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(feeds_widget)
        splitter.addWidget(ctrl_scroll)
        splitter.setStretchFactor(0, 7)
        splitter.setStretchFactor(1, 3)
        splitter.setHandleWidth(4)
        main_layout.addWidget(splitter)

        self.statusBar().showMessage(
            "Ready – configure cameras and Modbus, then click Start."
        )

    # ── Populate COM ports ───────────────────────────────────────────
    def _populate_com_ports(self):
        self.combo_port.clear()
        ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
        default_idx = -1
        for idx, p in enumerate(ports):
            self.combo_port.addItem(p.device)
            if p.device == "COM7":
                default_idx = idx
        if default_idx >= 0:
            self.combo_port.setCurrentIndex(default_idx)
        elif self.combo_port.count() == 0:
            self.combo_port.addItem("COM7")

    # ── Signals ──────────────────────────────────────────────────────
    def _connect_signals(self):
        self.btn_start.clicked.connect(self._on_start)
        self.btn_stop.clicked.connect(self._on_stop)
        self.btn_refresh.clicked.connect(self._enumerate_cameras)
        self.combo_trigger_mode.currentIndexChanged.connect(self._update_trigger_ui_mode)
        for i, spin in enumerate(self.delay_spins):
            spin.valueChanged.connect(lambda _v, s=i: self._on_delay_changed(s))

        frame_sigs = [self.sig_frame_0, self.sig_frame_1,
                      self.sig_frame_2, self.sig_frame_3]
        for i, sig in enumerate(frame_sigs):
            lbl = self.frame_labels[i]
            sig.connect(lambda img, _l=lbl: self._set_pixmap(_l, img))

        self.sig_coil_status.connect(self._update_coil_display)
        self.sig_trigger.connect(self._on_modbus_trigger)
        self.sig_modbus_error.connect(self._on_modbus_error)
        self._update_trigger_ui_mode()

    def _arrange_feed_panels(self, active_slots):
        while self.feeds_grid.count():
            item = self.feeds_grid.takeAt(0)
            if item and item.widget():
                item.widget().setParent(None)

        if not active_slots:
            active_slots = [0]

        for i, grp in enumerate(self.frame_groups):
            grp.setVisible(i in active_slots)

        n = len(active_slots)
        if n == 1:
            self.feeds_grid.addWidget(self.frame_groups[active_slots[0]], 0, 0, 2, 2)
        elif n == 2:
            self.feeds_grid.addWidget(self.frame_groups[active_slots[0]], 0, 0)
            self.feeds_grid.addWidget(self.frame_groups[active_slots[1]], 0, 1)
        elif n == 3:
            self.feeds_grid.addWidget(self.frame_groups[active_slots[0]], 0, 0)
            self.feeds_grid.addWidget(self.frame_groups[active_slots[1]], 0, 1)
            self.feeds_grid.addWidget(self.frame_groups[active_slots[2]], 1, 0, 1, 2)
        else:
            self.feeds_grid.addWidget(self.frame_groups[active_slots[0]], 0, 0)
            self.feeds_grid.addWidget(self.frame_groups[active_slots[1]], 0, 1)
            self.feeds_grid.addWidget(self.frame_groups[active_slots[2]], 1, 0)
            self.feeds_grid.addWidget(self.frame_groups[active_slots[3]], 1, 1)

        self.feeds_grid.setRowStretch(0, 1)
        self.feeds_grid.setRowStretch(1, 1)
        self.feeds_grid.setColumnStretch(0, 1)
        self.feeds_grid.setColumnStretch(1, 1)

    def _is_modbus_mode(self):
        return self.combo_trigger_mode.currentData() == "modbus"

    def _apply_hardware_trigger_delay(self, slot_idx):
        cam = self.slot_cams[slot_idx]
        if cam is None or self._is_modbus_mode():
            return

        delay_ms = self.delay_spins[slot_idx].value()
        delay_us = float(delay_ms) * 1000.0

        # MVS node is typically TriggerDelay in microseconds.
        ret = cam.MV_CC_SetFloatValue("TriggerDelay", delay_us)
        if ret != MV_OK:
            # Fallback names/types on some models/firmware.
            ret = cam.MV_CC_SetFloatValue("TriggerDelayAbs", delay_us)
        if ret != MV_OK:
            ret = cam.MV_CC_SetIntValueEx("TriggerDelay", int(delay_us))

        if ret == MV_OK:
            # Read back what camera accepted when possible.
            applied_ms = delay_ms
            fv = MVCC_FLOATVALUE()
            ret_get = cam.MV_CC_GetFloatValue("TriggerDelay", fv)
            if ret_get == MV_OK:
                applied_ms = float(fv.fCurValue) / 1000.0
            else:
                ret_get = cam.MV_CC_GetFloatValue("TriggerDelayAbs", fv)
                if ret_get == MV_OK:
                    applied_ms = float(fv.fCurValue) / 1000.0

            self.slot_trigger_delay_ms[slot_idx] = applied_ms
            self.delay_applied_labels[slot_idx].setText(f"Applied: {applied_ms:.3f} ms")
            print(
                f"[INFO] Slot {slot_idx + 1} TriggerDelay requested={delay_ms} ms, "
                f"applied={applied_ms:.3f} ms"
            )
        else:
            self.slot_trigger_delay_ms[slot_idx] = 0.0
            self.delay_applied_labels[slot_idx].setText("Applied: n/a")
            print(
                f"[WARN] Slot {slot_idx + 1} TriggerDelay not applied "
                f"(camera may not support node), ret=0x{ret:08X}"
            )

    def _on_delay_changed(self, slot_idx):
        self.delay_applied_labels[slot_idx].setText("Applied: pending")
        if self.streaming and not self._is_modbus_mode() and self.slot_cams[slot_idx] is not None:
            self._apply_hardware_trigger_delay(slot_idx)

    def _update_trigger_ui_mode(self):
        use_modbus = self._is_modbus_mode()
        self.mb_grp.setEnabled(use_modbus)
        self.combo_trigger_line.setEnabled(not use_modbus)
        for i in range(4):
            r1, r2 = self.coil_buttons[i]
            r1.setEnabled(use_modbus)
            r2.setEnabled(use_modbus)
            # Delay is useful for both modes; in hardware mode it delays save after trigger frame arrival.
            self.delay_spins[i].setEnabled(True)

    def _set_pixmap(self, label, img):
        h, w, ch = img.shape
        rgb = img[:, :, ::-1].copy()
        qimg = QImage(rgb.data, w, h, w * ch, QImage.Format_RGB888)
        pix = QPixmap.fromImage(qimg)
        label.setPixmap(
            pix.scaled(label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        )

    # ── Enumerate cameras ────────────────────────────────────────────
    def _enumerate_cameras(self):
        self.available_cameras = []

        # GigE
        gige_list = MV_CC_DEVICE_INFO_LIST()
        ret = MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE, gige_list)
        if ret == MV_OK and gige_list.nDeviceNum > 0:
            # Auto-configure IPs for cameras on wrong subnet
            for i in range(gige_list.nDeviceNum):
                dev = ctypes.cast(
                    gige_list.pDeviceInfo[i],
                    ctypes.POINTER(MV_CC_DEVICE_INFO),
                ).contents
                if dev.nTLayerType == MV_GIGE_DEVICE:
                    auto_configure_gige_ip(dev, ip_offset=200 + i)
            time.sleep(0.5)
            # Re-enumerate after ForceIP
            gige_list = MV_CC_DEVICE_INFO_LIST()
            MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE, gige_list)

        self._gige_list = gige_list

        for i in range(gige_list.nDeviceNum):
            dev = ctypes.cast(
                gige_list.pDeviceInfo[i],
                ctypes.POINTER(MV_CC_DEVICE_INFO),
            ).contents
            if dev.nTLayerType == MV_GIGE_DEVICE:
                gi = dev.SpecialInfo.stGigEInfo
                model = "".join(chr(c) for c in gi.chModelName if c != 0)
                ip = ip_int_to_str(gi.nCurrentIp)
                name = f"[GigE] {model} ({ip})"
                self.available_cameras.append(("gige", i, name, "GigE"))

        # USB
        usb_list = MV_CC_DEVICE_INFO_LIST()
        ret = MvCamera.MV_CC_EnumDevices(MV_USB_DEVICE, usb_list)
        if ret != MV_OK:
            usb_list.nDeviceNum = 0
        self._usb_list = usb_list

        for i in range(usb_list.nDeviceNum):
            dev = ctypes.cast(
                usb_list.pDeviceInfo[i],
                ctypes.POINTER(MV_CC_DEVICE_INFO),
            ).contents
            if dev.nTLayerType == MV_USB_DEVICE:
                ui = dev.SpecialInfo.stUsb3VInfo
                model = "".join(chr(c) for c in ui.chModelName if c != 0)
                sn = "".join(chr(c) for c in ui.chSerialNumber if c != 0)
                tag = f" (S/N: {sn})" if sn else f" #{i + 1}"
                name = f"[USB] {model}{tag}"
                self.available_cameras.append(("usb", i, name, "USB"))

        # Update all combo boxes
        for combo in self.cam_combos:
            prev = combo.currentText()
            combo.clear()
            combo.addItem("-- None --")
            for _, _, cam_name, _ in self.available_cameras:
                combo.addItem(cam_name)
            idx = combo.findText(prev)
            if idx >= 0:
                combo.setCurrentIndex(idx)

        self.statusBar().showMessage(
            f"Found {len(self.available_cameras)} camera(s)."
        )
        shown = min(max(len(self.available_cameras), 1), 4)
        self._arrange_feed_panels(list(range(shown)))

    def _configure_trigger_for_camera(self, cam, slot_idx):
        if self._is_modbus_mode():
            cam.MV_CC_SetEnumValueByString("TriggerMode", "Off")
            return True

        line_name = self.combo_trigger_line.currentData()
        checks = [
            cam.MV_CC_SetEnumValueByString("AcquisitionMode", "Continuous"),
            cam.MV_CC_SetEnumValueByString("TriggerMode", "On"),
            cam.MV_CC_SetEnumValueByString("TriggerSource", line_name),
            cam.MV_CC_SetEnumValueByString("TriggerActivation", "RisingEdge"),
        ]
        if any(ret != MV_OK for ret in checks):
            # Fallback to numeric enum values for camera models exposing numeric entries.
            fallback = [
                cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_ON),
                cam.MV_CC_SetEnumValue(
                    "TriggerSource",
                    {
                        "Line0": MV_TRIGGER_SOURCE_LINE0,
                        "Line1": MV_TRIGGER_SOURCE_LINE1,
                        "Line2": MV_TRIGGER_SOURCE_LINE2,
                        "Line3": MV_TRIGGER_SOURCE_LINE3,
                    }.get(line_name, MV_TRIGGER_SOURCE_LINE0),
                ),
            ]
            if not all(ret == MV_OK for ret in fallback):
                return False

        self.slot_cams[slot_idx] = cam
        self._apply_hardware_trigger_delay(slot_idx)
        self.slot_cams[slot_idx] = None
        return True

    # ── Open a single camera for a slot ──────────────────────────────
    def _open_camera_for_slot(self, slot_idx):
        cam_combo_idx = self.cam_combos[slot_idx].currentIndex() - 1
        if cam_combo_idx < 0 or cam_combo_idx >= len(self.available_cameras):
            return None

        list_type, dev_idx, _, transport = self.available_cameras[cam_combo_idx]

        if list_type == "gige":
            dev = ctypes.cast(
                self._gige_list.pDeviceInfo[dev_idx],
                ctypes.POINTER(MV_CC_DEVICE_INFO),
            ).contents
        else:
            dev = ctypes.cast(
                self._usb_list.pDeviceInfo[dev_idx],
                ctypes.POINTER(MV_CC_DEVICE_INFO),
            ).contents

        cam = MvCamera()
        ret = cam.MV_CC_CreateHandle(dev)
        if ret != MV_OK:
            print(f"[ERROR] Slot {slot_idx + 1} CreateHandle: 0x{ret:08X}")
            return None

        ret = (cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive)
               if transport == "GigE" else cam.MV_CC_OpenDevice())
        if ret != MV_OK:
            cam.MV_CC_DestroyHandle()
            print(f"[ERROR] Slot {slot_idx + 1} OpenDevice: 0x{ret:08X}")
            return None

        if transport == "GigE":
            pkt = cam.MV_CC_GetOptimalPacketSize()
            if pkt > 0:
                cam.MV_CC_SetIntValueEx("GevSCPSPacketSize", pkt)

        if not self._configure_trigger_for_camera(cam, slot_idx):
            print(f"[ERROR] Slot {slot_idx + 1} trigger setup failed")
            cam.MV_CC_CloseDevice()
            cam.MV_CC_DestroyHandle()
            return None

        return cam

    # ── Start ────────────────────────────────────────────────────────
    def _on_start(self):
        self.statusBar().showMessage("Starting cameras and Modbus...")
        QApplication.processEvents()

        any_cam = False
        active_slots = []
        for i in range(4):
            cam = self._open_camera_for_slot(i)
            if cam is None:
                continue
            ret = cam.MV_CC_StartGrabbing()
            if ret != MV_OK:
                cam.MV_CC_CloseDevice()
                cam.MV_CC_DestroyHandle()
                print(f"[ERROR] Slot {i + 1} StartGrabbing: 0x{ret:08X}")
                continue

            stParam = MVCC_INTVALUE_EX()
            cam.MV_CC_GetIntValueEx("PayloadSize", stParam)
            buf_size = int(stParam.nCurValue) + 2048

            worker = CameraWorker(cam, buf_size)
            worker.start()
            self.slot_cams[i] = cam
            self.slot_workers[i] = worker
            self.last_frame_seen[i] = None
            active_slots.append(i)
            any_cam = True
            print(f"[INFO] Slot {i + 1} started – buf={buf_size}")

        if not any_cam:
            QMessageBox.warning(
                self, "No Cameras",
                "No cameras could be opened.\n"
                "Check slot selections and connections.",
            )
            return

        if self._is_modbus_mode():
            port = self.combo_port.currentText()
            baud = int(self.combo_baud.currentText())
            poll = self.spin_poll.value()

            self.modbus_poller = ModbusPoller(port, baud, poll)
            self.modbus_poller.on_trigger = lambda addr: self.sig_trigger.emit(addr)
            self.modbus_poller.on_status = lambda s1, s2: self.sig_coil_status.emit(s1, s2)
            self.modbus_poller.on_error = lambda msg: self.sig_modbus_error.emit(msg)
            self.modbus_poller.start()

        self.streaming = True
        self.frame_timer.start(33)
        self._arrange_feed_panels(active_slots)

        # Lock settings that must not change while running
        for combo in self.cam_combos:
            combo.setEnabled(False)
        self.btn_refresh.setEnabled(False)
        self.combo_trigger_mode.setEnabled(False)
        self.combo_trigger_line.setEnabled(False)
        self.combo_port.setEnabled(False)
        self.combo_baud.setEnabled(False)
        self.spin_poll.setEnabled(False)

        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        if self._is_modbus_mode():
            self.statusBar().showMessage("Running – waiting for Modbus triggers...")
        else:
            self.statusBar().showMessage("Running – waiting for hardware trigger on selected line...")

    # ── Refresh live feeds ───────────────────────────────────────────
    def _refresh_frames(self):
        sigs = [self.sig_frame_0, self.sig_frame_1,
                self.sig_frame_2, self.sig_frame_3]
        for i, sig in enumerate(sigs):
            w = self.slot_workers[i]
            if w and w.latest_frame is not None:
                sig.emit(w.latest_frame)
                if not self._is_modbus_mode() and w.latest_meta is not None:
                    frame_num = w.latest_meta.get("frame_num")
                    if frame_num != self.last_frame_seen[i]:
                        self.last_frame_seen[i] = frame_num
                        self._capture_slot(i, auto_frame=w.latest_frame, auto_meta=w.latest_meta)

    # ── Coil status display ──────────────────────────────────────────
    @Slot(bool, bool)
    def _update_coil_display(self, s1, s2):
        if s1:
            self.lbl_coil_3001.setText("● 3001: HIGH")
            self.lbl_coil_3001.setStyleSheet("color: #4caf50; font-weight: bold;")
        else:
            self.lbl_coil_3001.setText("● 3001: LOW")
            self.lbl_coil_3001.setStyleSheet("color: #888; font-weight: bold;")

        if s2:
            self.lbl_coil_3002.setText("● 3002: HIGH")
            self.lbl_coil_3002.setStyleSheet("color: #4caf50; font-weight: bold;")
        else:
            self.lbl_coil_3002.setText("● 3002: LOW")
            self.lbl_coil_3002.setStyleSheet("color: #888; font-weight: bold;")

        if not self._modbus_connected_shown:
            self.lbl_modbus_status.setText("Connected")
            self.lbl_modbus_status.setStyleSheet("color: #4caf50;")
            self._modbus_connected_shown = True

    # ── Modbus trigger handler ───────────────────────────────────────
    @Slot(int)
    def _on_modbus_trigger(self, coil_addr):
        signal_perf_ns = time.perf_counter_ns()
        signal_wall = datetime.now()
        print(f"[TRIGGER] Coil {coil_addr} rising edge detected")
        for i in range(4):
            assigned = self.coil_btn_groups[i].checkedId()
            if assigned == coil_addr and self.slot_workers[i] is not None:
                delay = self.delay_spins[i].value()
                if delay > 0:
                    QTimer.singleShot(
                        delay,
                        lambda s=i, p=signal_perf_ns, w=signal_wall: self._capture_slot(
                            s, signal_perf_ns=p, signal_wall=w
                        ),
                    )
                else:
                    self._capture_slot(i, signal_perf_ns=signal_perf_ns, signal_wall=signal_wall)

    def _update_profile(self, slot, signal_wall=None, acq_wall=None,
                        latency_ms=None, trigger_index=None, signal_estimated=False):
        signal_text = signal_wall.strftime("%H:%M:%S.%f")[:-3] if signal_wall else "-"
        acq_text = acq_wall.strftime("%H:%M:%S.%f")[:-3] if acq_wall else "-"
        trig_text = f" | TriggerIndex={trigger_index}" if trigger_index is not None else ""
        est_text = " (est)" if signal_estimated else ""
        self.lbl_prof_signal.setText(f"Signal: {signal_text}{est_text}{trig_text}")
        self.lbl_prof_acq.setText(f"Acquisition: {acq_text} (Slot {slot + 1})")
        if latency_ms is None:
            self.lbl_prof_latency.setText("Latency: N/A")
        else:
            self.lbl_prof_latency.setText(f"Latency: {latency_ms:.3f} ms")

    def _capture_slot(self, slot, signal_perf_ns=None, signal_wall=None,
                      auto_frame=None, auto_meta=None):
        worker = self.slot_workers[slot]
        if worker is None:
            return

        frame = auto_frame.copy() if auto_frame is not None else None
        meta = auto_meta if auto_meta is not None else worker.latest_meta
        if frame is None:
            if worker.latest_frame is None:
                return
            frame = worker.latest_frame.copy()

        if meta is None:
            acq_perf_ns = None
            acq_wall = datetime.now()
            trigger_index = None
        else:
            acq_perf_ns = meta.get("recv_perf_ns")
            acq_wall = meta.get("recv_wall", datetime.now())
            trigger_index = meta.get("trigger_index")

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        filename = f"{SLOT_PREFIXES[slot]}_{timestamp}.bmp"
        path = os.path.join(SLOT_FOLDERS[slot], filename)
        save_bgr_bmp(frame, path)
        self.capture_counter += 1
        self.lbl_count.setText(f"Captures: {self.capture_counter}")

        latency_ms = None
        if signal_perf_ns is not None and acq_perf_ns is not None:
            latency_ms = (acq_perf_ns - signal_perf_ns) / 1_000_000.0

        signal_estimated = False
        if signal_wall is None and not self._is_modbus_mode():
            # In hardware mode we don't have direct input-edge timestamp from SDK here,
            # so estimate signal time using camera-applied TriggerDelay.
            applied_delay_ms = self.slot_trigger_delay_ms[slot]
            signal_wall = acq_wall - timedelta(milliseconds=applied_delay_ms)
            signal_estimated = True
            if latency_ms is None:
                latency_ms = applied_delay_ms

        self._update_profile(
            slot,
            signal_wall=signal_wall,
            acq_wall=acq_wall,
            latency_ms=latency_ms,
            trigger_index=trigger_index,
            signal_estimated=signal_estimated,
        )
        self.statusBar().showMessage(
            f"Captured slot {slot + 1} ({SLOT_PREFIXES[slot]}) at {timestamp}"
        )
        print(f"[CAPTURE] Slot {slot + 1} → {path}")

    # ── Modbus error ─────────────────────────────────────────────────
    @Slot(str)
    def _on_modbus_error(self, msg):
        self.lbl_modbus_status.setText(f"Error: {msg}")
        self.lbl_modbus_status.setStyleSheet("color: #f44336;")
        self.statusBar().showMessage(f"Modbus error: {msg}")

    # ── Stop ─────────────────────────────────────────────────────────
    def _on_stop(self):
        self.frame_timer.stop()

        # Modbus
        if self.modbus_poller:
            self.modbus_poller.stop()
            self.modbus_poller.join(timeout=3)
            self.modbus_poller = None
            self._modbus_connected_shown = False

        # Camera workers
        for i in range(4):
            w = self.slot_workers[i]
            if w:
                w.stop()
                w.join(timeout=3)
                self.slot_workers[i] = None

        # Camera handles
        for i in range(4):
            cam = self.slot_cams[i]
            if cam:
                cam.MV_CC_StopGrabbing()
                cam.MV_CC_CloseDevice()
                cam.MV_CC_DestroyHandle()
                self.slot_cams[i] = None

        self.streaming = False

        # Re-enable settings
        for combo in self.cam_combos:
            combo.setEnabled(True)
        self.btn_refresh.setEnabled(True)
        self.combo_trigger_mode.setEnabled(True)
        self.combo_trigger_line.setEnabled(True)
        self.combo_port.setEnabled(True)
        self.combo_baud.setEnabled(True)
        self.spin_poll.setEnabled(True)

        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)

        for lbl in self.frame_labels:
            lbl.setPixmap(QPixmap())
            lbl.setText("No Stream")

        shown = min(max(len(self.available_cameras), 1), 4)
        self._arrange_feed_panels(list(range(shown)))

        self.lbl_coil_3001.setText("● 3001: OFF")
        self.lbl_coil_3001.setStyleSheet("color: #888; font-weight: bold;")
        self.lbl_coil_3002.setText("● 3002: OFF")
        self.lbl_coil_3002.setStyleSheet("color: #888; font-weight: bold;")
        self.lbl_modbus_status.setText("Disconnected")
        self.lbl_modbus_status.setStyleSheet("color: #f44336;")
        self.lbl_prof_signal.setText("Signal: -")
        self.lbl_prof_acq.setText("Acquisition: -")
        self.lbl_prof_latency.setText("Latency: -")
        for lbl in self.delay_applied_labels:
            lbl.setText("Applied: -")
        self._update_trigger_ui_mode()

        self.statusBar().showMessage("Stopped.")

    def closeEvent(self, event):
        if self.streaming:
            self._on_stop()
        event.accept()


# ──────────────────────────────────────────────────────────────────────
def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(45, 45, 45))
    palette.setColor(QPalette.WindowText, QColor(220, 220, 220))
    palette.setColor(QPalette.Base, QColor(30, 30, 30))
    palette.setColor(QPalette.AlternateBase, QColor(45, 45, 45))
    palette.setColor(QPalette.ToolTipBase, QColor(220, 220, 220))
    palette.setColor(QPalette.ToolTipText, QColor(220, 220, 220))
    palette.setColor(QPalette.Text, QColor(220, 220, 220))
    palette.setColor(QPalette.Button, QColor(55, 55, 55))
    palette.setColor(QPalette.ButtonText, QColor(220, 220, 220))
    palette.setColor(QPalette.BrightText, QColor(255, 50, 50))
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, QColor(0, 0, 0))
    app.setPalette(palette)

    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
