import math
from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import Qt, QPointF, pyqtSignal, QRectF
from PyQt5.QtGui import QPainter, QColor, QPen, QBrush, QFont


class JoystickWidget(QWidget):
    x_changed = pyqtSignal(float)
    y_changed = pyqtSignal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._knob_x = 0.0
        self._knob_y = 0.0
        self._dragging = False
        self.setMinimumSize(150, 150)
        self.setMaximumSize(250, 250)

    def value(self):
        return self._knob_x, self._knob_y

    def reset(self):
        self._knob_x = 0.0
        self._knob_y = 0.0
        self._dragging = False
        self.x_changed.emit(0.0)
        self.y_changed.emit(0.0)
        self.update()

    def _knob_radius(self):
        return min(self.width(), self.height()) * 0.12

    def _base_radius(self):
        return min(self.width(), self.height()) * 0.38

    def _center(self):
        return QPointF(self.width() / 2, self.height() / 2)

    def _clamp_to_circle(self, dx, dy, r):
        dist = math.sqrt(dx * dx + dy * dy)
        if dist > r:
            dx = dx / dist * r
            dy = dy / dist * r
        return dx, dy

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        center = self._center()
        base_r = self._base_radius()
        knob_r = self._knob_radius()

        painter.fillRect(self.rect(), QColor(30, 30, 30))

        painter.setPen(QPen(QColor(80, 80, 80), 2))
        painter.setBrush(QBrush(QColor(50, 50, 50)))
        painter.drawEllipse(center, base_r, base_r)

        painter.setPen(QPen(QColor(70, 70, 70), 1))
        painter.drawLine(int(center.x() - base_r), int(center.y()),
                         int(center.x() + base_r), int(center.y()))
        painter.drawLine(int(center.x()), int(center.y() - base_r),
                         int(center.x()), int(center.y() + base_r))

        knob_pos = QPointF(
            center.x() + self._knob_x * base_r,
            center.y() + self._knob_y * base_r
        )
        painter.setPen(QPen(QColor(100, 180, 255), 2))
        painter.setBrush(QBrush(QColor(70, 130, 200)))
        painter.drawEllipse(knob_pos, knob_r, knob_r)

        painter.setPen(QColor(150, 150, 150))
        painter.setFont(QFont("Monospace", 8))
        painter.drawText(self.rect(), Qt.AlignTop | Qt.AlignHCenter,
                         f"x={self._knob_x:.2f}  y={self._knob_y:.2f}")
        painter.end()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._dragging = True
            self._update_from_mouse(event.pos())

    def mouseMoveEvent(self, event):
        if self._dragging:
            self._update_from_mouse(event.pos())

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._dragging = False
            self.reset()

    def _update_from_mouse(self, pos):
        center = self._center()
        base_r = self._base_radius()
        dx = pos.x() - center.x()
        dy = pos.y() - center.y()
        dx, dy = self._clamp_to_circle(dx, dy, base_r)
        old_x, old_y = self._knob_x, self._knob_y
        self._knob_x = round(dx / base_r, 3) if base_r else 0.0
        self._knob_y = round(dy / base_r, 3) if base_r else 0.0
        if abs(self._knob_x - old_x) > 0.001:
            self.x_changed.emit(self._knob_x)
        if abs(self._knob_y - old_y) > 0.001:
            self.y_changed.emit(self._knob_y)
        self.update()
