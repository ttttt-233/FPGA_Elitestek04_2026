"""Qt target monitor. All serial work is owned by SerialWorker/FPGAClient."""
from queue import Queue, Empty
from threading import Event, Lock
from pathlib import Path
import os
import sys
import time
from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtGui import QFont, QFontDatabase
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QComboBox, QSpinBox, QPushButton, QCheckBox, QGroupBox, QScrollArea,
    QSplitter, QTableWidget, QTableWidgetItem, QHeaderView, QPlainTextEdit, QFileDialog)
try:
    from .client import FPGAClient, CommandError, list_ports
    from .protocol import Algorithm, MODE_NAMES, OSD_MASK_NAMES, Status, digit_label, BAUD
except ImportError:
    from client import FPGAClient, CommandError, list_ports
    from protocol import Algorithm, MODE_NAMES, OSD_MASK_NAMES, Status, digit_label, BAUD


class SerialWorker(QThread):
    connection = Signal(bool, object)
    state = Signal(object, object)
    message = Signal(str)
    snapshot = Signal()
    photo_progress = Signal(int, int)
    photo_saved = Signal(object)
    photo_failed = Signal(str)

    def __init__(self, port, baud=BAUD, parent=None, client_factory=FPGAClient):
        super().__init__(parent)
        self.port, self.baud, self.factory = port, baud, client_factory
        self.tasks = Queue()
        self.stop_event = Event()
        self.client = None
        self._snapshot_lock = Lock()
        self._latest_snapshot = None

    def take_snapshot(self):
        # One queued notification, newest frame wins if the UI is busy.
        with self._snapshot_lock:
            latest, self._latest_snapshot = self._latest_snapshot, None
        return latest

    def on_record(self, record):
        if record["type"] != "frame": return
        with self._snapshot_lock:
            notify = self._latest_snapshot is None
            self._latest_snapshot = record
        if notify: self.snapshot.emit()

    def submit(self, method, *args):
        self.tasks.put((method, args))

    def stop(self):
        self.stop_event.set()
        if self.client is not None:
            self.client.cancel()

    def run(self):
        try:
            if self.stop_event.is_set(): return
            self.client = self.factory(self.port, self.baud)
            if self.stop_event.is_set(): return
            self.client.listeners.append(self.on_record)
            if hasattr(self.client, 'photo_progress'):
                self.client.photo_progress.append(self.photo_progress.emit)
            self.client.connect()
            self.connection.emit(True, self.client.version)
            last_poll = last_targets = 0.0
            streaming = self.client.version.get("build_id", 0) >= 0x0303
            while not self.stop_event.is_set():
                try:
                    method, args = self.tasks.get(timeout=0.002)
                except Empty:
                    method = None
                if method:
                    try:
                        result = getattr(self.client, method)(*args)
                        if method == 'save_playback_photo': self.photo_saved.emit(result)
                        self.message.emit(f"{method}: {result}")
                    except Exception as exc:
                        if method == 'save_playback_photo': self.photo_failed.emit(str(exc))
                        self.message.emit(f"{method}: {exc}")
                    last_poll = 0
                if self.stop_event.is_set(): break
                self.client.poll()
                now = time.monotonic()
                if now - last_poll >= 0.75:
                    previous_version = dict(self.client.version)
                    if hasattr(self.client, 'get_version'):
                        version = self.client.get_version()
                        if version != previous_version:
                            self.connection.emit(True, version)
                            streaming = version.get('build_id', 0) >= 0x0303
                    status = self.client.get_status()
                    self.state.emit(status, None)
                    last_poll = time.monotonic()
                if not streaming and now - last_targets >= 0.1:
                    try:
                        targets = self.client.get_targets()
                    except CommandError as exc:
                        if exc.status != Status.BUSY: raise
                        targets = []
                    self.on_record(dict(type="frame", targets=targets))
                    last_targets = time.monotonic()
        except Exception as exc:
            if not self.stop_event.is_set(): self.message.emit(str(exc))
        finally:
            if self.client is not None:
                self.client.disconnect()
            self.connection.emit(False, {})


try:
    from .target_view import TargetCanvas
except ImportError:
    from target_view import TargetCanvas


def initialize_fonts(app):
    # Offscreen Windows Qt can start without a system font database. Load a
    # local system font explicitly; no bundled/downloaded font is required.
    if not QFontDatabase.families():
        path=Path(os.environ.get('SystemRoot','C:/Windows'))/'Fonts/segoeui.ttf'
        if path.exists():
            font_id=QFontDatabase.addApplicationFont(str(path))
            families=QFontDatabase.applicationFontFamilies(font_id)
            if families: app.setFont(QFont(families[0],9))


class MainWindow(QMainWindow):
    def __init__(self):
        initialize_fonts(QApplication.instance())
        super().__init__()
        self.worker = None
        self.setWindowTitle("Ti60 UART — runtime controls and persistent targets")
        self.resize(1120, 820)
        root = QWidget();self.setCentralWidget(root);layout = QVBoxLayout(root)
        row = QHBoxLayout();layout.addLayout(row)
        self.port = QComboBox();self.port.setEditable(True)
        self.baud = QSpinBox();self.baud.setRange(1200, 2000000);self.baud.setValue(BAUD)
        refresh = QPushButton("Refresh ports");refresh.clicked.connect(self.refresh_ports)
        self.connect_button = QPushButton("Connect");self.connect_button.clicked.connect(self.toggle_connection)
        self.version_label = QLabel("Disconnected")
        for widget in (QLabel("Port"), self.port, refresh, QLabel("Baud"), self.baud, self.connect_button, self.version_label): row.addWidget(widget)
        self.summary = QLabel("No FPGA connection. Target map uses telemetry; SNAP RAW is downloaded when saving.")
        self.summary.setWordWrap(True);layout.addWidget(self.summary)
        splitter = QSplitter();layout.addWidget(splitter, 1)
        left = QWidget();column = QVBoxLayout(left);column.setContentsMargins(0, 0, 0, 0)
        self.canvas = TargetCanvas();column.addWidget(self.canvas, 1)
        column.addWidget(QLabel("Standard target view | PC trail: last 8 received centers"))
        self.table = QTableWidget(6, 11)
        self.table.setHorizontalHeaderLabels(("Slot", "ID hex", "Kind", "Digit", "Quality", "Age", "Lost", "BBox L,T,R,B", "vx", "vy", "Alive"))
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(7, QHeaderView.Stretch)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers);self.table.setMaximumHeight(235)
        column.addWidget(self.table);splitter.addWidget(left)
        scroll = QScrollArea();scroll.setWidgetResizable(True);scroll.setMinimumWidth(300)
        controls = QWidget();self.controls_layout = QVBoxLayout(controls);scroll.setWidget(controls);splitter.addWidget(scroll)
        self.controls = controls
        self.build_controls();self.controls.setEnabled(False)
        splitter.setSizes([790, 310])
        self.log = QPlainTextEdit();self.log.setReadOnly(True);self.log.setMaximumHeight(100)
        self.log.document().setMaximumBlockCount(150);layout.addWidget(self.log)
        self.refresh_ports()

    def refresh_ports(self):
        old = self.port.currentText()
        try: ports = list_ports()
        except Exception as exc: ports = [];self.version_label.setText(str(exc))
        self.port.clear();self.port.addItems(ports)
        if old: self.port.setCurrentText(old)

    def command(self, method, *args):
        if self.worker and self.worker.isRunning(): self.worker.submit(method, *args)

    def box(self, title):
        box = QGroupBox(title);grid = QGridLayout(box);self.controls_layout.addWidget(box);return grid

    def button(self, grid, row, col, label, method, *args):
        b = QPushButton(label);b.clicked.connect(lambda: self.command(method, *args));grid.addWidget(b, row, col);return b

    def build_controls(self):
        self.checks = {}
        self.controls_layout.addWidget(QLabel("All levels are available directly"))
        grid = self.box("Display")
        self.mode = QComboBox();self.mode.addItems(MODE_NAMES)
        self.mode.activated.connect(lambda i: self.command("set_mode", i))
        grid.addWidget(QLabel("Display"), 0, 0);grid.addWidget(self.mode, 0, 1)
        grid = self.box("Threshold")
        self.threshold = QSpinBox();self.threshold.setRange(16, 1024);self.threshold.setValue(96)
        grid.addWidget(self.threshold, 0, 0)
        b = QPushButton("Set threshold");b.clicked.connect(lambda: self.command("set_threshold", self.threshold.value()));grid.addWidget(b, 0, 1)
        c = QCheckBox("AUTO threshold");c.clicked.connect(lambda state: self.command("set_auto", state))
        grid.addWidget(c, 1, 0, 1, 2);self.checks["auto"] = c
        grid = self.box("Algorithm")
        self.algorithm = QComboBox();self.algorithm.addItems([a.name for a in Algorithm])
        self.algorithm.activated.connect(lambda i: self.command("set_algorithm", i))
        grid.addWidget(QLabel("Edge"), 0, 0);grid.addWidget(self.algorithm, 0, 1)
        self.filter = QComboBox();self.filter.addItems(("Median", "Gaussian"))
        self.filter.activated.connect(lambda i: self.command("set_filter", i))
        grid.addWidget(QLabel("Filter"), 1, 0);grid.addWidget(self.filter, 1, 1)
        self.output_profile = QComboBox();self.output_profile.addItems(("720P", "480P60"))
        self.output_profile.activated.connect(lambda i: self.command("profile_request", i))
        self.output_profile.setToolTip("Switches the FPGA image; requires both profiles in flash.")
        grid.addWidget(QLabel("Output"), 2, 0);grid.addWidget(self.output_profile, 2, 1)
        c = QCheckBox("Shape recognition");c.clicked.connect(lambda state: self.command("set_shape_enable", state))
        grid.addWidget(c, 3, 0, 1, 2);self.checks["shape"] = c
        self.filter_profile = QLabel("Waiting for applied filter / output")
        self.filter_profile.setWordWrap(True)
        grid.addWidget(self.filter_profile, 4, 0, 1, 2)
        grid = self.box("OSD")
        for row, (name, title, method) in enumerate((("osd", "OSD ALL", "set_osd_enable"), ("quick_hide", "Quick hide", "set_quick_hide"))):
            c = QCheckBox(title);c.clicked.connect(lambda state, method=method: self.command(method, state));grid.addWidget(c, row, 0, 1, 2);self.checks[name] = c
        self.mask_checks = []
        for bit, title in enumerate(OSD_MASK_NAMES):
            c = QCheckBox(title);c.setChecked(bit < 2)
            c.clicked.connect(self.set_osd_regions)
            grid.addWidget(c, bit + 2, 0, 1, 2);self.mask_checks.append(c)
        grid = self.box("Capture")
        for row, pair in enumerate((("Snapshot", "snapshot", "PRE", "pre_capture"), ("VIDEO IN", "video_start", "VIDEO OUT", "video_stop"))):
            self.button(grid, row, 0, pair[0], pair[1]);self.button(grid, row, 1, pair[2], pair[3])
        grid = self.box("Playback")
        for row, pair in enumerate((("Media previous", "media_prev", "Media next", "media_next"), ("Play", "play", "Pause", "pause"))):
            self.button(grid, row, 0, pair[0], pair[1]);self.button(grid, row, 1, pair[2], pair[3])
        self.save_photo_button = QPushButton('Save playback SNAP: DNG + 2 JPEG')
        self.save_photo_button.clicked.connect(self.save_playback_photo)
        grid.addWidget(self.save_photo_button, 2, 0, 1, 2)
        self.photo_progress_label = QLabel('Select a SNAP photo in PLAYBACK')
        self.photo_progress_label.setWordWrap(True);grid.addWidget(self.photo_progress_label, 3, 0, 1, 2)
        self.controls_layout.addStretch()

    def set_osd_regions(self):
        self.command("set_osd_mask", sum(1 << bit for bit, check in enumerate(self.mask_checks) if check.isChecked()))

    def save_playback_photo(self):
        path, _ = QFileDialog.getSaveFileName(self, 'Save playback SNAP', 'photo.dng', 'Digital Negative (*.dng)')
        if path:
            self.save_photo_button.setEnabled(False)
            self.photo_progress_label.setText('Downloading selected SNAP…')
            self.command('save_playback_photo', path)

    def toggle_connection(self):
        if self.worker and self.worker.isRunning():
            self.connect_button.setEnabled(False);self.worker.stop();return
        if not self.port.currentText().strip():
            self.log.appendPlainText("Choose a serial port first.");return
        self.worker = SerialWorker(self.port.currentText().strip(), self.baud.value(), self)
        self.worker.connection.connect(self.connection_changed)
        self.worker.state.connect(self.update_state)
        self.worker.snapshot.connect(self.update_snapshot)
        self.worker.message.connect(self.log.appendPlainText)
        self.worker.photo_progress.connect(lambda done,total: self.photo_progress_label.setText(
            f'RAW {done*100//total}% ({done:,}/{total:,} bytes); integer replay follows'))
        self.worker.photo_saved.connect(lambda paths: self.photo_finished('Saved DNG + 2 JPEG: '+str(paths[0])))
        self.worker.photo_failed.connect(lambda error: self.photo_finished('Save failed: '+error))
        self.worker.finished.connect(lambda: self.connect_button.setEnabled(True))
        self.connect_button.setEnabled(False);self.worker.start()

    def photo_finished(self, message):
        self.photo_progress_label.setText(message)
        self.save_photo_button.setEnabled(self.controls.isEnabled())

    def connection_changed(self, connected, version):
        self.controls.setEnabled(connected);self.port.setEnabled(not connected);self.baud.setEnabled(not connected)
        self.connect_button.setText("Disconnect" if connected else "Connect");self.connect_button.setEnabled(True)
        if connected:
            self.version_label.setText(f"Protocol {version['protocol']} · build {version['build_id']:04X}")
            self.canvas.width_px=version["width"];self.canvas.height_px=version["height"];self.canvas.fps=version["fps"]
            self.output_profile.setCurrentIndex(1 if version["height"] == 480 else 0)
            for widget in (self.filter, self.output_profile):
                widget.setEnabled(version["build_id"] >= 0x0307)
            self.filter.setToolTip("" if version["build_id"] >= 0x0307 else "Requires firmware build 0307 or newer.")
            self.output_profile.setToolTip("Switches the FPGA image; requires both profiles in flash."
                if version["build_id"] >= 0x0307 else "Requires firmware build 0307 or newer.")
        else:
            self.version_label.setText("Disconnected");self.summary.setText("Disconnected — no live target data")
            self.canvas.trails.clear();self.canvas.previous_lost.clear()
            self.canvas.set_targets([]);self.table.clearContents()

    def update_state(self, status, targets):
        c, media = status["control"], status["media"]
        filter_name = {0: "MEDIAN", 1: "GAUSSIAN"}.get(status.get("actual_filter"), "pending")
        self.filter_profile.setText(f"Applied: {filter_name} · {self.canvas.width_px}×{self.canvas.height_px}")
        self.output_profile.setCurrentIndex(1 if self.canvas.height_px == 480 else 0)
        if status.get("actual_filter") in (0, 1): self.filter.setCurrentIndex(status["actual_filter"])
        regions = ", ".join(title for bit, title in enumerate(OSD_MASK_NAMES) if c["osd_mask"] & (1 << bit)) or "None"
        self.summary.setText(f"FPS {status['fps']:.1f} · ERR {status['failed_frames']} · targets {status['target_count']} · "
            f"{Algorithm(c['algorithm']).name} · threshold manual {c['threshold']} / applied {status['actual_high']} "
            f"(low {status['actual_low']}) · AUTO {c['auto']} · shape {c['shape']} · "
            f"profile {self.canvas.width_px}×{self.canvas.height_px}@{self.canvas.fps} · "
            f"media {media['index']}/{media['count']} REC {media['recording']} PLAY {media['playing']} · "
            f"OSD ALL {c['osd']} HIDE {c['quick_hide']} · {regions} · "
            f"UART rx/check/overdrive {status['uart_rx_errors']}/{status['uart_checksum_errors']}/{status['uart_tx_overruns']}")
        self.algorithm.setCurrentIndex(c["algorithm"]);self.mode.setCurrentIndex(c["mode"])
        if not self.threshold.hasFocus(): self.threshold.setValue(c["threshold"])
        for name in ("auto", "shape", "osd", "quick_hide"): self.checks[name].setChecked(c[name])
        for bit, check in enumerate(self.mask_checks): check.setChecked(bool(c["osd_mask"] & (1 << bit)))
        if targets is not None: self.update_targets(targets)

    def update_snapshot(self):
        if self.worker:
            snapshot = self.worker.take_snapshot()
            if snapshot is not None: self.update_targets(snapshot["targets"])

    def update_targets(self, targets):
        self.table.clearContents()
        for t in targets:
            b=t["bbox"]
            values=(t["slot"],f"{t['id']:02X}",t["kind_name"],digit_label(t["digit"]) if t["kind"]==4 else "",t["quality"],
                    t["age"],t["lost_age"],",".join(str(b[k]) for k in ("left","top","right","bottom")),t["vx"],t["vy"],t["alive"])
            for col,value in enumerate(values): self.table.setItem(t["slot"],col,QTableWidgetItem(str(value)))
        self.canvas.set_targets(targets)

    def closeEvent(self, event):
        if self.worker and self.worker.isRunning():
            self.worker.stop()
            if not self.worker.wait(2000):
                self.worker.finished.connect(self.close);event.ignore();return
        event.accept()


def main():
    app = QApplication(sys.argv);window = MainWindow();window.show();return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
