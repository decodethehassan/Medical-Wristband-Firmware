import asyncio
import re
import sys
import time
from collections import deque
from pathlib import Path

from bleak import BleakClient, BleakScanner
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from qasync import QEventLoop, asyncSlot


DEVICE_NAME = "SMARTWATCH"
LOG_NOTIFY_UUID = "9f7b0001-6c35-4d2c-9c85-4a8c1a2b3c4d"
MAX_POINTS = 2000
PLOT_REFRESH_MS = 120


KV_RE = re.compile(r"([A-Za-z0-9_/%]+)\s*[:=]\s*(-?\d+(?:\.\d+)?)")


def parse_ppg_line(line: str) -> dict | None:
    """Parse only PV/PPG_STREAM lines used for wrist PPG validation."""
    lower = line.lower()
    is_ppg = lower.startswith("ppg_stream,") or lower.startswith("pv,")
    if not is_ppg:
        return None

    pairs = {}
    for key, value in KV_RE.findall(line):
        try:
            pairs[key.lower()] = float(value)
        except ValueError:
            pass

    def get(*names, default=None):
        for name in names:
            if name.lower() in pairs:
                return pairs[name.lower()]
        return default

    fw_t_ms = get("t", "t_ms", "time_ms")
    green = get("green", "ppg_green", "grn")
    raw = get("raw", "ppg_raw", "stored_sig", default=green)
    filt = get("filt", "filtered", "ppg_filt")
    th = get("th", "threshold", "ppg_th")
    peak = get("peak", "peak_flag", default=0.0)
    qok = get("qok", "quality_ok", default=-1.0)
    sqi = get("sqi", "ppg_sqi", default=-1.0)
    art = get("art", "artifact", default=-1.0)
    settle = get("settle", "settling", default=-1.0)
    fs = get("fs", "fs_hz", default=-1.0)

    if fw_t_ms is None or filt is None:
        return None

    return {
        "t_ms": fw_t_ms,
        "raw": raw,
        "green": green,
        "filt": filt,
        "th": th,
        "peak": peak,
        "qok": qok,
        "sqi": sqi,
        "art": art,
        "settle": settle,
        "fs": fs,
    }


class PpgPlot(QWidget):
    def __init__(self):
        super().__init__()
        self.first_t_ms: float | None = None
        self.samples = deque(maxlen=MAX_POINTS)

        self.figure = Figure(figsize=(10, 6), tight_layout=True)
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(1, 1, 1)
        self.stats = QLabel("Waiting for PPG_STREAM/PV data...")
        self.stats.setWordWrap(True)

        self.clear_btn = QPushButton("Clear PPG Plot")
        self.clear_btn.clicked.connect(self.clear)
        self.save_csv_btn = QPushButton("Save PPG CSV")
        self.save_csv_btn.clicked.connect(self.save_csv)

        top = QHBoxLayout()
        top.addWidget(self.clear_btn)
        top.addWidget(self.save_csv_btn)
        top.addStretch(1)

        layout = QVBoxLayout(self)
        layout.addLayout(top)
        layout.addWidget(self.stats)
        layout.addWidget(self.canvas, 1)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(PLOT_REFRESH_MS)

    def clear(self):
        self.first_t_ms = None
        self.samples.clear()
        self.stats.setText("Waiting for PPG_STREAM/PV data...")
        self.refresh()

    def add_sample(self, sample: dict):
        t_ms = float(sample["t_ms"])
        if self.first_t_ms is None:
            self.first_t_ms = t_ms
        sample = dict(sample)
        sample["t_s"] = (t_ms - self.first_t_ms) / 1000.0
        self.samples.append(sample)

    def refresh(self):
        self.ax.clear()
        self.ax.grid(True, linewidth=0.3)
        self.ax.set_title("Wrist PPG validation: green raw + filtered PPG")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("ADC / filtered units")

        if not self.samples:
            self.ax.text(0.5, 0.5, "Waiting for PPG_STREAM or PV lines...", ha="center", va="center", transform=self.ax.transAxes)
            self.canvas.draw_idle()
            return

        data = list(self.samples)
        t = [s["t_s"] for s in data]
        raw = [s.get("green") if s.get("green") is not None else s.get("raw") for s in data]
        filt = [s.get("filt") for s in data]
        th = [s.get("th") for s in data]

        # Raw green is offset-normalized so it can be seen together with filtered PPG.
        raw_valid = [(ti, yi) for ti, yi in zip(t, raw) if yi is not None]
        if raw_valid:
            rt, ry = zip(*raw_valid)
            raw_mean = sum(ry) / len(ry)
            self.ax.plot(rt, [v - raw_mean for v in ry], linewidth=0.8, label="green raw - DC")

        filt_valid = [(ti, yi) for ti, yi in zip(t, filt) if yi is not None]
        if filt_valid:
            ft, fy = zip(*filt_valid)
            self.ax.plot(ft, fy, linewidth=1.2, label="filtered PPG")

        th_valid = [(ti, yi) for ti, yi in zip(t, th) if yi is not None]
        if th_valid:
            tt, ty = zip(*th_valid)
            self.ax.plot(tt, ty, linewidth=1.0, label="threshold")

        peak_t = []
        peak_y = []
        for s in data:
            if s.get("peak", 0.0) >= 0.5 and s.get("filt") is not None:
                peak_t.append(s["t_s"])
                peak_y.append(s["filt"])
        if peak_t:
            self.ax.scatter(peak_t, peak_y, s=28, label="accepted peak")

        self.ax.legend(loc="upper right", fontsize=9)

        latest = data[-1]
        self.stats.setText(
            f"samples={len(data)} | fs={latest.get('fs', -1):.1f} Hz | "
            f"qok={int(latest.get('qok', -1))} | sqi={latest.get('sqi', -1):.3f} | "
            f"art={int(latest.get('art', -1))} | settle={int(latest.get('settle', -1))} | "
            f"green={latest.get('green', latest.get('raw')):.0f} | filt={latest.get('filt', 0):.2f}"
        )
        self.canvas.draw_idle()

    def save_csv(self):
        if not self.samples:
            QMessageBox.information(self, "No PPG data", "No PPG data has been received yet.")
            return

        path, _ = QFileDialog.getSaveFileName(self, "Save PPG CSV", "wrist_ppg_log.csv", "CSV Files (*.csv);;All Files (*)")
        if not path:
            return

        keys = ["t_ms", "t_s", "raw", "green", "filt", "th", "peak", "qok", "sqi", "art", "settle", "fs"]
        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write(",".join(keys) + "\n")
                for s in self.samples:
                    f.write(",".join(str(s.get(k, "")) for k in keys) + "\n")
        except Exception as exc:
            QMessageBox.critical(self, "CSV save failed", str(exc))


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Simple Wrist PPG GUI")
        self.resize(1200, 720)

        self.client: BleakClient | None = None
        self.devices = {}
        self.rx_buf = ""

        self.device_list = QListWidget()
        self.scan_btn = QPushButton("Scan")
        self.connect_btn = QPushButton("Connect")
        self.disconnect_btn = QPushButton("Disconnect")
        self.clear_logs_btn = QPushButton("Clear Logs")
        self.save_logs_btn = QPushButton("Save Logs")
        self.status = QLabel("Status: Idle")
        self.logs = QTextEdit()
        self.logs.setReadOnly(True)
        self.plot = PpgPlot()

        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.addWidget(QLabel("BLE devices"))
        left_layout.addWidget(self.device_list, 2)
        left_layout.addWidget(self.scan_btn)
        left_layout.addWidget(self.connect_btn)
        left_layout.addWidget(self.disconnect_btn)
        left_layout.addWidget(self.clear_logs_btn)
        left_layout.addWidget(self.save_logs_btn)
        left_layout.addWidget(self.status)
        left_layout.addWidget(QLabel("Logs used by this GUI"))
        left_layout.addWidget(self.logs, 5)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left)
        splitter.addWidget(self.plot)
        splitter.setSizes([380, 820])
        self.setCentralWidget(splitter)

        self.scan_btn.clicked.connect(self.scan)
        self.connect_btn.clicked.connect(self.connect_selected)
        self.disconnect_btn.clicked.connect(self.disconnect)
        self.clear_logs_btn.clicked.connect(self.logs.clear)
        self.save_logs_btn.clicked.connect(self.save_logs)

    def append_log(self, text: str):
        self.logs.append(text)
        self.logs.ensureCursorVisible()

    def set_status(self, text: str):
        self.status.setText(f"Status: {text}")

    @asyncSlot()
    async def scan(self):
        self.set_status("Scanning...")
        self.device_list.clear()
        self.devices.clear()
        self.append_log("Scanning for BLE devices...")

        try:
            devices = await BleakScanner.discover(timeout=6.0)
        except Exception as exc:
            self.set_status("Scan failed")
            QMessageBox.critical(self, "Scan failed", str(exc))
            return

        for dev in devices:
            name = dev.name or "(Unknown)"
            addr = dev.address
            self.devices[addr] = dev
            item = QListWidgetItem(f"{name} | {addr}")
            item.setData(Qt.UserRole, addr)
            self.device_list.addItem(item)
            if name == DEVICE_NAME:
                self.device_list.setCurrentItem(item)

        self.set_status(f"Found {len(devices)} device(s)")
        self.append_log(f"Found {len(devices)} device(s).")

    @asyncSlot()
    async def connect_selected(self):
        item = self.device_list.currentItem()
        if item is None:
            QMessageBox.warning(self, "No device selected", "Scan and select your wristband first.")
            return

        addr = item.data(Qt.UserRole)
        dev = self.devices.get(addr)
        if dev is None:
            QMessageBox.warning(self, "Device missing", "Please scan again.")
            return

        await self.disconnect()
        self.set_status(f"Connecting to {dev.name or addr}...")
        self.append_log(f"Connecting to {dev.name or addr}...")

        try:
            self.client = BleakClient(dev)
            await self.client.connect()
            await self.client.start_notify(LOG_NOTIFY_UUID, self.on_notify)
        except Exception as exc:
            self.client = None
            self.set_status("Connection failed")
            QMessageBox.critical(self, "Connection failed", str(exc))
            return

        self.set_status("Connected and listening for PPG_STREAM/PV")
        self.append_log("Connected. Waiting for PPG_STREAM/PV lines...")

    @asyncSlot()
    async def disconnect(self):
        if self.client is not None:
            try:
                if self.client.is_connected:
                    await self.client.disconnect()
            except Exception:
                pass
            self.client = None
            self.set_status("Disconnected")

    def on_notify(self, sender, data: bytearray):
        chunk = bytes(data).decode("utf-8", errors="replace")
        self.rx_buf += chunk.replace("\r\n", "\n").replace("\r", "\n")

        while "\n" in self.rx_buf:
            line, self.rx_buf = self.rx_buf.split("\n", 1)
            line = line.strip()
            if not line:
                continue
            self.append_log(line)
            sample = parse_ppg_line(line)
            if sample is not None:
                self.plot.add_sample(sample)

    def save_logs(self):
        text = self.logs.toPlainText()
        if not text.strip():
            QMessageBox.information(self, "No logs", "No logs to save yet.")
            return

        path, _ = QFileDialog.getSaveFileName(self, "Save logs", "wrist_ppg_ble_logs.txt", "Text Files (*.txt);;All Files (*)")
        if not path:
            return
        try:
            Path(path).write_text(text, encoding="utf-8")
        except Exception as exc:
            QMessageBox.critical(self, "Save logs failed", str(exc))

    def closeEvent(self, event):
        # qasync will let the event loop finish after window closes.
        event.accept()


async def main_async():
    app = QApplication.instance()
    win = MainWindow()
    win.show()
    while win.isVisible():
        await asyncio.sleep(0.1)
    await win.disconnect()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    loop = QEventLoop(app)
    asyncio.set_event_loop(loop)
    with loop:
        loop.run_until_complete(main_async())
