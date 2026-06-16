import sys
import asyncio
import re
import time
from collections import deque

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QPushButton, QLabel, QTextEdit, QListWidget, QListWidgetItem,
    QMessageBox, QTabWidget, QFileDialog, QCheckBox, QComboBox, QSplitter
)
from PySide6.QtCore import Qt, QTimer

from qasync import QEventLoop, asyncSlot
from bleak import BleakScanner, BleakClient

# Real-time plotting
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


DEVICE_NAME = "SMARTWATCH"
LOG_NOTIFY_UUID = "9f7b0001-6c35-4d2c-9c85-4a8c1a2b3c4d"

# Number of samples kept in each waveform plot.
MAX_SAMPLES = 500

# Plot refresh interval in ms. BLE can send faster than this; plots are throttled
# so the GUI remains responsive.
PLOT_REFRESH_MS = 250


class WaveformPanel(QWidget):
    """
    Large single-graph real-time signal visualization panel.

    Use the dropdown to choose exactly which validation graph to view.
    This is easier for PPG validation than showing many small subplots.
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        self.series = {}
        self.start_time = time.monotonic()
        self.first_fw_t_ms = None

        self.signal_views = [
            ("Final PPG waveform", ["PPG_FILT"], "Final filtered PPG"),
            ("Final PPG filtered + threshold", ["PPG_FILT", "PPG_TH"], "Filtered PPG / threshold"),
            ("PPG raw selected", ["PPG_RAW_SELECTED"], "Selected raw PPG ADC"),
            ("PPG raw green", ["PPG_GREEN"], "MAX30101 green raw ADC"),
            ("PPG raw all channels", ["PPG_RED", "PPG_IR", "PPG_GREEN"], "MAX30101 raw ADC"),
            ("Peak detection", ["PPG_PEAK"], "Peak flag"),
            ("IBI / pulse interval", ["IBI_MS"], "IBI (ms)"),
            ("Heart rate", ["HR"], "Heart rate (BPM)"),
            ("HRV / RMSSD", ["HRV_RMSSD"], "RMSSD (ms)"),
            ("HR + HRV summary", ["HR", "HRV_RMSSD"], "BPM / RMSSD (ms)"),
            ("PPG SQI", ["PPG_SQI"], "Signal quality index"),
            ("Clip fraction", ["CLIP_FRAC"], "Clipped sample fraction"),
            ("Saturation flag", ["SATURATION"], "Saturation 0/1"),
            ("Artifact flag", ["ARTIFACT"], "Artifact 0/1"),
            ("Quality OK gate", ["QUALITY_OK"], "Quality OK 0/1"),
            ("Motion SMA", ["SMA"], "SMA / motion level"),
            ("Acceleration variance", ["ACC_VAR"], "Acceleration variance"),
            ("Sampling frequency", ["FS_HZ"], "Sampling frequency (Hz)"),
            ("Accelerometer XYZ", ["ACC_X", "ACC_Y", "ACC_Z"], "Acceleration"),
            ("Gyroscope XYZ", ["GYRO_X", "GYRO_Y", "GYRO_Z"], "Gyroscope"),
            ("Temperature", ["TEMP", "THERM"], "Temperature (°C)"),
            ("EDA", ["EDA"], "EDA"),
            ("Battery", ["BATTERY", "VBAT"], "Battery / voltage"),
            ("SpO2", ["SPO2"], "SpO2 (%)"),
        ]

        self.signal_combo = QComboBox()
        for title, _, _ in self.signal_views:
            self.signal_combo.addItem(title)
        self.signal_combo.currentIndexChanged.connect(self.refresh)

        self.enabled_cb = QCheckBox("Enable live plotting")
        self.enabled_cb.setChecked(True)

        self.pause_cb = QCheckBox("Pause plot")
        self.pause_cb.setChecked(False)

        self.autoscale_cb = QCheckBox("Auto-scale Y")
        self.autoscale_cb.setChecked(True)

        self.window_combo = QComboBox()
        self.window_combo.addItems(["1000 samples", "500 samples", "300 samples", "150 samples", "75 samples"])
        self.window_combo.setCurrentText("500 samples")
        self.window_combo.currentIndexChanged.connect(self.refresh)

        self.clear_btn = QPushButton("Clear Waveforms")
        self.clear_btn.clicked.connect(self.clear)

        self.save_csv_btn = QPushButton("Save Selected CSV")
        self.save_csv_btn.clicked.connect(self.save_selected_csv)

        control_row_1 = QHBoxLayout()
        control_row_1.addWidget(QLabel("Graph:"))
        control_row_1.addWidget(self.signal_combo, 2)
        control_row_1.addWidget(QLabel("Display window:"))
        control_row_1.addWidget(self.window_combo)

        control_row_2 = QHBoxLayout()
        control_row_2.addWidget(self.enabled_cb)
        control_row_2.addWidget(self.pause_cb)
        control_row_2.addWidget(self.autoscale_cb)
        control_row_2.addStretch(1)
        control_row_2.addWidget(self.save_csv_btn)
        control_row_2.addWidget(self.clear_btn)

        self.stats_lbl = QLabel("Waiting for data...")
        self.stats_lbl.setWordWrap(True)

        self.figure = Figure(figsize=(12, 7), tight_layout=True)
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(1, 1, 1)
        self.ax.grid(True, linewidth=0.3)

        for _, names, _ in self.signal_views:
            for name in names:
                self.series.setdefault(name, deque(maxlen=MAX_SAMPLES * 4))

        layout = QVBoxLayout(self)
        layout.addLayout(control_row_1)
        layout.addLayout(control_row_2)
        layout.addWidget(self.stats_lbl)
        layout.addWidget(self.canvas, 1)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(PLOT_REFRESH_MS)

    def _current_window_len(self) -> int:
        text = self.window_combo.currentText()
        m = re.search(r"(\d+)", text)
        return int(m.group(1)) if m else 500

    def _selected_view(self):
        idx = self.signal_combo.currentIndex()
        if idx < 0 or idx >= len(self.signal_views):
            idx = 0
        return self.signal_views[idx]

    def clear(self):
        self.series.clear()
        for _, names, _ in self.signal_views:
            for name in names:
                self.series[name] = deque(maxlen=MAX_SAMPLES * 4)
        self.start_time = time.monotonic()
        self.first_fw_t_ms = None
        self.stats_lbl.setText("Waiting for data...")
        self.refresh()

    def add_value(self, name: str, value: float, t_seconds: float | None = None):
        if not self.enabled_cb.isChecked():
            return
        if name.startswith("__"):
            return
        if name not in self.series:
            self.series[name] = deque(maxlen=MAX_SAMPLES * 4)
        if t_seconds is None:
            t_seconds = time.monotonic() - self.start_time
        self.series[name].append((float(t_seconds), float(value)))

    def add_values(self, values: dict):
        if not values:
            return

        fw_t_ms = values.get("__t_ms__")
        t_seconds = None
        if fw_t_ms is not None:
            try:
                fw_t_ms = float(fw_t_ms)
                if self.first_fw_t_ms is None:
                    self.first_fw_t_ms = fw_t_ms
                t_seconds = (fw_t_ms - self.first_fw_t_ms) / 1000.0
            except Exception:
                t_seconds = None

        for name, value in values.items():
            if name.startswith("__"):
                continue
            try:
                self.add_value(name, float(value), t_seconds)
            except Exception:
                pass

    def refresh(self):
        if self.pause_cb.isChecked():
            return

        title, names, y_label = self._selected_view()
        window_len = self._current_window_len()

        self.ax.clear()
        self.ax.grid(True, linewidth=0.3)
        self.ax.set_title(title, fontsize=13)
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel(y_label)

        any_data = False
        stats_parts = []

        for name in names:
            data = list(self.series.get(name, []))[-window_len:]
            if not data:
                continue

            x, y = zip(*data)
            self.ax.plot(x, y, linewidth=1.2, label=name)
            any_data = True

            latest = y[-1]
            ymin = min(y)
            ymax = max(y)
            mean = sum(y) / len(y)
            stats_parts.append(
                f"{name}: latest={latest:.3f}, min={ymin:.3f}, max={ymax:.3f}, mean={mean:.3f}, n={len(y)}"
            )

        if any_data:
            self.ax.legend(loc="upper right", fontsize=9)
            if not self.autoscale_cb.isChecked():
                # Keep binary flag plots readable when autoscale is disabled.
                if all(name in ("PPG_PEAK", "ARTIFACT", "SATURATION") for name in names):
                    self.ax.set_ylim(-0.1, 1.1)
            self.stats_lbl.setText(" | ".join(stats_parts))
        else:
            self.ax.text(0.5, 0.5, "Waiting for selected signal...", ha="center", va="center", transform=self.ax.transAxes)
            self.stats_lbl.setText(f"Waiting for data for: {', '.join(names)}")

        self.canvas.draw_idle()

    def save_selected_csv(self):
        title, names, _ = self._selected_view()
        safe_title = re.sub(r'[\\/*?:"<>|]+', "_", title).strip().replace(" ", "_") or "selected_signal"
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            f'Save "{title}" Data',
            f"{safe_title}.csv",
            "CSV Files (*.csv);;All Files (*)"
        )
        if not file_path:
            return

        try:
            with open(file_path, "w", encoding="utf-8") as f:
                f.write("channel,t_seconds,value\n")
                for name in names:
                    for t, value in self.series.get(name, []):
                        f.write(f"{name},{t:.6f},{value:.9g}\n")
            self.stats_lbl.setText(f"Saved selected CSV to: {file_path}")
        except Exception as e:
            QMessageBox.critical(self, "Save CSV failed", str(e))


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BLE Log Viewer + Real-Time Signals")
        self.resize(1280, 760)

        self.client: BleakClient | None = None
        self.devices = {}  # addr -> device (from latest scan)
        self._rx_buf = ""

        self.device_list = QListWidget()
        self.scan_btn = QPushButton("Scan")
        self.connect_btn = QPushButton("Connect")
        self.disconnect_btn = QPushButton("Disconnect")
        self.clear_btn = QPushButton("Clear Current Tab")
        self.save_btn = QPushButton("Save Current Tab")
        self.status_lbl = QLabel("Status: Idle")

        self.tabs = QTabWidget()
        self.logs = {}
        self._ensure_log_tab("All")

        self.waveform_panel = WaveformPanel()
        self.tabs.addTab(self.waveform_panel, "Real-Time Signals")

        root = QWidget()
        main = QHBoxLayout(root)

        left = QVBoxLayout()
        left.addWidget(QLabel("Discovered BLE Devices"))
        left.addWidget(self.device_list)

        btn_row = QHBoxLayout()
        btn_row.addWidget(self.scan_btn)
        btn_row.addWidget(self.connect_btn)
        btn_row.addWidget(self.disconnect_btn)

        left.addLayout(btn_row)
        left.addWidget(self.clear_btn)
        left.addWidget(self.save_btn)
        left.addWidget(self.status_lbl)
        main.addLayout(left, 3)

        right = QVBoxLayout()
        right.addWidget(QLabel("Logs and real-time waveforms"))
        right.addWidget(self.tabs)
        main.addLayout(right, 8)

        self.setCentralWidget(root)

        self.scan_btn.clicked.connect(self.on_scan)
        self.connect_btn.clicked.connect(self.on_connect)
        self.disconnect_btn.clicked.connect(self.on_disconnect)
        self.clear_btn.clicked.connect(self.on_clear_current_tab)
        self.save_btn.clicked.connect(self.on_save_current_tab)

    def _ensure_log_tab(self, name: str) -> QTextEdit:
        if name in self.logs:
            return self.logs[name]
        w = QTextEdit()
        w.setReadOnly(True)
        self.logs[name] = w
        self.tabs.addTab(w, name)
        return w

    def _append(self, tab_name: str, text: str):
        w = self._ensure_log_tab(tab_name)
        w.append(text)
        w.ensureCursorVisible()

    def set_status(self, text: str):
        self.status_lbl.setText(f"Status: {text}")

    def _safe_filename(self, name: str) -> str:
        return re.sub(r'[\\/*?:"<>|]+', "_", name).strip() or "logs"

    def on_clear_current_tab(self):
        idx = self.tabs.currentIndex()
        if idx < 0:
            return

        w = self.tabs.widget(idx)
        if isinstance(w, QTextEdit):
            w.clear()
        elif isinstance(w, WaveformPanel):
            w.clear()

    def on_save_current_tab(self):
        idx = self.tabs.currentIndex()
        if idx < 0:
            QMessageBox.warning(self, "No tab selected", "There is no tab selected.")
            return

        tab_name = self.tabs.tabText(idx).strip()
        w = self.tabs.widget(idx)

        if isinstance(w, WaveformPanel):
            file_path, _ = QFileDialog.getSaveFileName(
                self,
                'Save "Real-Time Signals" Image',
                "real_time_signals.png",
                "PNG Image (*.png);;All Files (*)"
            )
            if not file_path:
                return
            try:
                w.figure.savefig(file_path, dpi=160)
                self.set_status('Saved waveform image')
                self._append("All", f'Saved real-time signals image to: {file_path}')
            except Exception as e:
                QMessageBox.critical(self, "Save failed", str(e))
            return

        if not isinstance(w, QTextEdit):
            QMessageBox.warning(self, "Save failed", "Current tab is not a text log tab.")
            return

        content = w.toPlainText()
        if not content.strip():
            QMessageBox.information(self, "No logs", f'The "{tab_name}" tab is empty.')
            return

        default_name = f"{self._safe_filename(tab_name)}.txt"
        file_path, _ = QFileDialog.getSaveFileName(
            self,
            f'Save "{tab_name}" Logs',
            default_name,
            "Text Files (*.txt);;Log Files (*.log);;All Files (*)"
        )

        if not file_path:
            return

        try:
            with open(file_path, "w", encoding="utf-8") as f:
                f.write(content)
            self.set_status(f'Saved "{tab_name}"')
            self._append("All", f'Saved current tab "{tab_name}" to: {file_path}')
        except Exception as e:
            QMessageBox.critical(self, "Save failed", str(e))

    @asyncSlot()
    async def on_scan(self):
        self.set_status("Scanning...")
        self._append("All", "Scanning for BLE devices...")
        self.device_list.clear()
        self.devices.clear()

        try:
            devices = await BleakScanner.discover(timeout=6.0)
        except Exception as e:
            self.set_status("Scan failed")
            QMessageBox.critical(self, "Scan failed", str(e))
            return

        if not devices:
            self.set_status("No devices found")
            self._append("All", "No devices found.")
            return

        for d in devices:
            name = d.name or "(Unknown)"
            addr = d.address
            self.devices[addr] = d

            item = QListWidgetItem(f"{name} | {addr}")
            item.setData(Qt.UserRole, addr)
            self.device_list.addItem(item)

        self.set_status(f"Found {len(devices)} device(s)")
        self._append("All", f"Found {len(devices)} device(s).")

    def on_notify(self, sender: int, data: bytearray):
        chunk = bytes(data).decode("utf-8", errors="replace")
        self._rx_buf += chunk
        self._rx_buf = self._rx_buf.replace("\r\n", "\n").replace("\r", "\n")

        while "\n" in self._rx_buf:
            line, self._rx_buf = self._rx_buf.split("\n", 1)
            line = line.strip()
            if not line:
                continue

            module = line.split(":", 1)[0].strip() if ":" in line else "Unknown"
            self._append("All", line)
            self._append(module, line)

            # New feature: parse existing log lines and plot real-time signals.
            values = self.parse_signal_values(line)
            if values:
                self.waveform_panel.add_values(values)

    def parse_signal_values(self, line: str) -> dict:
        """
        Converts firmware BLE text logs into plot channels.

        Supports final PPG stream lines from complete FW:
            PPG_STREAM,t=...,seq=...,raw=...,green=...,filt=...,th=...,peak=...,ibi=...,hr=...,fs=...
        Also supports PPG validation lines:
            PV,t=...,seq=...,green=...,filt=...,th=...,peak=...,ibi=...,hr=...,fs=...
            PV_WIN,t=...,ppg_n=...,sqi=...,clip=...,sat=...,art=...,sma=...,acc_var=...
        Also keeps support for older raw PPG/IMU/temp/EDA/HR log formats.
        """
        values = {}
        lower = line.lower()

        # key=value parser; supports key=123, key=-12.3, key:123
        pairs = {}
        for key, val in re.findall(r"([A-Za-z0-9_/%]+)\s*[:=]\s*(-?\d+(?:\.\d+)?)", line):
            try:
                pairs[key.lower()] = float(val)
            except ValueError:
                pass

        def get(*keys):
            for k in keys:
                if k.lower() in pairs:
                    return pairs[k.lower()]
            return None

        fw_t = get("t", "t_ms", "time_ms")
        if fw_t is not None:
            values["__t_ms__"] = fw_t

        is_pv_line = (
            lower.startswith("pv,") or lower.startswith("pv_win,") or
            lower.startswith("ppg_stream,") or "pv," in lower or
            "pv_win," in lower or "ppg_stream," in lower
        )

        # PPG final/validation stream fields
        raw_selected = get("raw", "ppg_raw", "ppg_selected", "stored_sig")
        green = get("green", "ppg_green", "ppg:grn", "grn")
        filt = get("filt", "filtered", "ppg_filt", "ppg_filtered")
        th = get("th", "threshold", "ppg_th")
        peak = get("peak", "peak_flag")
        ibi = get("ibi", "ibi_ms")
        fs = get("fs", "fs_hz", "ppg_fs")
        sqi = get("sqi", "ppg_sqi")
        clip = get("clip", "clip_frac", "clip_fraction")
        sat = get("sat", "saturation")
        art = get("art", "artifact")
        qok = get("qok", "quality_ok")
        sma = get("sma")
        acc_var = get("acc_var", "accel_var", "acceleration_variance")

        if raw_selected is not None and ("ppg_stream" in lower or "pv," in lower or "ppg out" in lower):
            values["PPG_RAW_SELECTED"] = raw_selected
        if green is not None:
            values["PPG_GREEN"] = green
        if filt is not None:
            values["PPG_FILT"] = filt
        if th is not None:
            values["PPG_TH"] = th
        if peak is not None:
            values["PPG_PEAK"] = peak
        if ibi is not None and ibi > 0:
            values["IBI_MS"] = ibi
        if fs is not None:
            values["FS_HZ"] = fs
        if sqi is not None:
            values["PPG_SQI"] = sqi
        if clip is not None:
            values["CLIP_FRAC"] = clip
        if sat is not None:
            values["SATURATION"] = sat
        if art is not None:
            values["ARTIFACT"] = art
        if qok is not None:
            values["QUALITY_OK"] = qok
        if sma is not None:
            values["SMA"] = sma
        if acc_var is not None:
            values["ACC_VAR"] = acc_var

        # Red/IR are kept for compatibility with non-validation logs.
        ir = get("ir", "ppg_ir", "ppg:ir")
        red = get("red", "ppg_red", "ppg:red")
        if ir is not None:
            values["PPG_IR"] = ir
        if red is not None:
            values["PPG_RED"] = red

        # IMU accelerometer
        ax = get("ax_mg", "accx", "acc_x", "ax")
        ay = get("ay_mg", "accy", "acc_y", "ay")
        az = get("az_mg", "accz", "acc_z", "az")
        if ax is not None:
            values["ACC_X"] = ax
        if ay is not None:
            values["ACC_Y"] = ay
        if az is not None:
            values["ACC_Z"] = az

        # IMU gyroscope
        gx = get("gx_mdps", "gyrox", "gyro_x", "gx")
        gy = get("gy_mdps", "gyroy", "gyro_y", "gy")
        gz = get("gz_mdps", "gyroz", "gyro_z", "gz")
        if gx is not None:
            values["GYRO_X"] = gx
        if gy is not None:
            values["GYRO_Y"] = gy
        if gz is not None:
            values["GYRO_Z"] = gz

        # Temperature
        temp_x100 = get("temp_c_x100", "temperature_x100")
        temp = get("temp", "temp_c", "temperature", "therm")
        if temp_x100 is not None:
            values["TEMP"] = temp_x100 / 100.0
        elif temp is not None:
            values["TEMP"] = temp

        # EDA / SCR
        eda_mv = get("eda_mv", "mv", "eda")
        eda_raw = get("eda_raw", "raw")
        scr_amp = get("scr_amp", "scr:amp")
        scr_freq = get("scr_freq", "scr:freq")
        scr_rise = get("scr_rise", "scr:rise")

        if eda_mv is not None and ("eda" in lower or "scr" in lower or "raw_eda" in lower):
            values["EDA"] = eda_mv
        elif eda_raw is not None and ("eda" in lower or "scr" in lower or "raw_eda" in lower):
            values["EDA"] = eda_raw

        if scr_amp is not None:
            values["SCR_AMP"] = scr_amp
        if scr_freq is not None:
            values["SCR_FREQ"] = scr_freq
        if scr_rise is not None:
            values["SCR_RISE"] = scr_rise

        # HR / HRV / SpO2. In PPG stream lines, hr=-1 means no new valid beat yet, so skip it.
        hr = get("hr", "heart_rate")
        hrv_rmssd = get("hrv_rmssd", "rmssd", "hrv")
        spo2 = get("spo2", "spo2%")
        if hr is not None:
            if not is_pv_line or hr > 0:
                values["HR"] = hr
        if hrv_rmssd is not None and hrv_rmssd > 0:
            values["HRV_RMSSD"] = hrv_rmssd
        if spo2 is not None:
            values["SPO2"] = spo2

        # Battery / voltage
        batt = get("battery", "batt", "bat", "battery_percent")
        vbat = get("vbat", "battery_mv", "vbatt")
        if batt is not None:
            values["BATTERY"] = batt
        if vbat is not None:
            values["VBAT"] = vbat

        # Fallback support for compact labels such as:
        # ACC:X=..., GYRO:Y=..., PPG:IR=...
        compact_map = {
            "ACC:X": "ACC_X", "ACC:Y": "ACC_Y", "ACC:Z": "ACC_Z",
            "GYRO:X": "GYRO_X", "GYRO:Y": "GYRO_Y", "GYRO:Z": "GYRO_Z",
            "PPG:IR": "PPG_IR", "PPG:RED": "PPG_RED", "PPG:GRN": "PPG_GREEN",
            "PPG:GREEN": "PPG_GREEN", "HR": "HR", "EDA": "EDA",
            "SCR:AMP": "SCR_AMP", "SCR:FREQ": "SCR_FREQ", "SCR:RISE": "SCR_RISE",
            "THERM": "THERM", "TEMP": "TEMP",
        }
        for label, channel in compact_map.items():
            m = re.search(re.escape(label) + r"\s*[:=]\s*(-?\d+(?:\.\d+)?)", line, flags=re.I)
            if m:
                try:
                    values[channel] = float(m.group(1))
                except ValueError:
                    pass

        return values

    @asyncSlot()
    async def on_connect(self):
        if self.client:
            await self.on_disconnect()

        # ALWAYS do a fresh scan and match by NAME
        self.set_status("Fresh scan for connect...")
        self._append("All", f"Fresh scan: looking for {DEVICE_NAME}...")

        try:
            devices = await BleakScanner.discover(timeout=6.0)
        except Exception as e:
            self.set_status("Scan failed")
            QMessageBox.critical(self, "Scan failed", str(e))
            return

        target = None
        for d in devices:
            if (d.name or "").strip() == DEVICE_NAME:
                target = d
                break

        if target is None:
            self.set_status("Device not advertising")
            QMessageBox.critical(
                self,
                "Connection failed",
                f"Could not find {DEVICE_NAME} in a fresh scan.\n\n"
                "Do this:\n"
                "1) Close nRF Connect (mobile/desktop) completely.\n"
                "2) Turn Bluetooth OFF/ON on PC.\n"
                "3) Press Scan again.\n"
            )
            return

        self._rx_buf = ""
        self.set_status(f"Connecting to {target.address} ...")
        self._append("All", f"Connecting to {DEVICE_NAME} @ {target.address} ...")

        self.client = BleakClient(target, timeout=15.0)

        try:
            await self.client.connect()
        except Exception as e:
            self.set_status("Connection failed")
            QMessageBox.critical(self, "Connection failed", str(e))
            self.client = None
            return

        self.set_status("Connected")
        self._append("All", "Connected.")

        self.set_status("Enabling notifications...")
        self._append("All", f"Starting notify on: {LOG_NOTIFY_UUID}")

        try:
            await self.client.start_notify(LOG_NOTIFY_UUID, self.on_notify)
        except Exception as e:
            self.set_status("Notify failed")
            QMessageBox.critical(self, "Notify failed", str(e))
            await self.on_disconnect()
            return

        self.set_status("Streaming logs")
        self._append("All", "✅ Notifications enabled. Waiting for logs...")

    @asyncSlot()
    async def on_disconnect(self):
        if self.client:
            try:
                try:
                    await self.client.stop_notify(LOG_NOTIFY_UUID)
                except Exception:
                    pass
                await self.client.disconnect()
            except Exception:
                pass
            self.client = None

        self._rx_buf = ""
        self.set_status("Disconnected")
        self._append("All", "Disconnected.")


def main():
    if sys.platform.startswith("win"):
        try:
            asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
        except Exception:
            pass

    app = QApplication(sys.argv)
    loop = QEventLoop(app)
    asyncio.set_event_loop(loop)

    win = MainWindow()
    win.show()

    with loop:
        loop.run_forever()


if __name__ == "__main__":
    main()
