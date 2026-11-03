"""Standard target geometry and bounded interpolation, independent of serial I/O."""
from collections import deque
import math
import time
from PySide6.QtCore import Qt, QTimer, QRectF, QPointF
from PySide6.QtGui import QColor, QPainter, QPainterPath, QPen, QFont, QPolygonF, QTransform
from PySide6.QtWidgets import QWidget
try:
    from .protocol import digit_label
except ImportError:
    from protocol import digit_label


class TargetCanvas(QWidget):
    STALE_SECONDS = 0.5

    def __init__(self, parent=None):
        super().__init__(parent)
        self.width_px, self.height_px, self.fps = 1280, 720, 30
        self.targets, self.trails, self.previous_lost = [], {}, {}
        self.transitions = {}
        self.received_at = 0.0
        self.duration = 1 / 30
        self.show_bounds = False
        self.setMinimumSize(440, 245)
        self.timer = QTimer(self)
        self.timer.setTimerType(Qt.PreciseTimer)
        self.timer.setInterval(16)
        self.timer.timeout.connect(self.update)
        self.timer.start()

    def display_box(self, target, now=None):
        now = time.monotonic() if now is None else now
        previous = self.transitions.get(target["id"], target["bbox"])
        alpha = min(1.0, max(0.0, (now - self.received_at) / self.duration))
        return {k: previous[k] + (v - previous[k]) * alpha for k, v in target["bbox"].items()}

    def set_targets(self, targets):
        now = time.monotonic()
        previous = {t["id"]: self.display_box(t, now) for t in self.targets
                    if t["alive"] and not t["lost_age"]}
        gap = now - self.received_at
        self.transitions = {t["id"]: previous[t["id"]] for t in targets
                            if t["id"] in previous and t["age"] > 1 and gap < self.STALE_SECONDS}
        self.duration = min(0.06, max(0.016, gap))
        self.received_at = now
        self.targets = targets
        active = {t["id"] for t in targets if t["alive"]}
        self.trails = {k: v for k, v in self.trails.items() if k in active}
        for target in targets:
            identity = target["id"]
            if target["alive"] and target["lost_age"] == 0:
                if target["age"] == 1 or self.previous_lost.get(identity, 0) >= (self.fps + 1) // 2:
                    self.trails[identity] = deque(maxlen=8)
                points = self.trails.setdefault(identity, deque(maxlen=8))
                b = target["bbox"]
                point = ((b["left"] + b["right"]) / 2, (b["top"] + b["bottom"]) / 2)
                if not points or points[-1] != point: points.append(point)
        self.previous_lost = {t["id"]: t["lost_age"] for t in targets}
        self.update()

    @staticmethod
    def target_path(target, box):
        """Canonical geometry fitted inside the measured extent, in image pixels."""
        path = QPainterPath()
        cx, cy = (box["left"] + box["right"]) / 2, (box["top"] + box["bottom"]) / 2
        w, h = max(1, box["right"] - box["left"]), max(1, box["bottom"] - box["top"])
        kind = target["kind"]
        if kind in (1, 2):
            size = min(w, h)
            square = QRectF(cx - size / 2, cy - size / 2, size, size)
            if kind == 1: path.addRect(square)
            else: path.addEllipse(square)
        elif kind == 3:
            side = min(w, h * 2 / math.sqrt(3))
            height = side * math.sqrt(3) / 2
            path.addPolygon(QPolygonF([QPointF(cx, cy - height / 2),
                QPointF(cx + side / 2, cy + height / 2), QPointF(cx - side / 2, cy + height / 2)]))
            path.closeSubpath()
        elif kind == 4:
            glyph = QPainterPath()
            glyph.addText(QPointF(0, 0), QFont("Arial", 100, QFont.Bold),
                          str(target["digit"]) if 0 <= target["digit"] <= 9 else "?")
            bounds = glyph.boundingRect()
            if bounds.width() and bounds.height():
                scale = min(w / bounds.width(), h / bounds.height())
                transform = QTransform()
                transform.translate(cx - bounds.width() * scale / 2, cy - bounds.height() * scale / 2)
                transform.scale(scale, scale)
                transform.translate(-bounds.left(), -bounds.top())
                path = transform.map(glyph)
        return path

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#101a24"))
        scale = min((self.width() - 20) / self.width_px, (self.height() - 20) / self.height_px)
        ox, oy = (self.width() - self.width_px * scale) / 2, (self.height() - self.height_px * scale) / 2
        p.translate(ox, oy);p.scale(scale, scale)
        font = p.font();font.setPointSizeF(9 / scale);p.setFont(font)
        p.setPen(QPen(QColor("#38505e"), 1 / scale))
        p.drawRect(QRectF(0, 0, self.width_px, self.height_px))
        now = time.monotonic()
        if now - self.received_at < self.STALE_SECONDS:
            for target in self.targets:
                if not target["alive"] or target["lost_age"] or not target["quality"]: continue
                b = self.display_box(target, now)
                path = self.target_path(target, b)
                color = QColor({1: "#70cedb", 2: "#86db9e", 3: "#f4cf75", 4: "#c9adff"}.get(target["kind"], "#70cedb"))
                p.setPen(QPen(color, 2 / scale))
                p.fillPath(path, color if target["kind"] == 4 else QColor(color.red(), color.green(), color.blue(), 35))
                p.drawPath(path)
                if self.show_bounds:
                    p.setPen(QPen(QColor("#526578"), 1 / scale, Qt.DashLine))
                    p.drawRect(QRectF(b["left"], b["top"], b["right"] - b["left"], b["bottom"] - b["top"]))
                p.setPen(QPen(color, 1 / scale))
                label = digit_label(target["digit"]) if target["kind"] == 4 else target["kind_name"]
                p.drawText(QPointF(b["left"], max(18, b["top"] - 8)), f"#{target['id']:02X} {label}")
                for age, point in enumerate(reversed(self.trails.get(target["id"], ()))):
                    r = max(1.5, 4 - age * 0.4) / scale
                    p.drawEllipse(QPointF(*point), r, r)
        p.end()
