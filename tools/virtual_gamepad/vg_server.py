#!/usr/bin/env python3
import sys
import json
import socket
import threading
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QHBoxLayout,
                              QVBoxLayout, QLabel, QSlider, QPushButton,
                              QStatusBar, QGroupBox)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
from joystick import JoystickWidget

HOST = "0.0.0.0"
PORT = 8888


class TcpServer:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.clients = []
        self.lock = threading.Lock()
        self.running = False
        self.sock = None

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.sock.listen(5)
        self.sock.settimeout(1.0)
        self.running = True
        threading.Thread(target=self._accept_loop, daemon=True).start()

    def stop(self):
        self.running = False
        if self.sock:
            self.sock.close()
        with self.lock:
            for c in self.clients:
                try:
                    c.close()
                except OSError:
                    pass
            self.clients.clear()

    def _accept_loop(self):
        while self.running:
            try:
                conn, addr = self.sock.accept()
                with self.lock:
                    self.clients.append(conn)
                threading.Thread(target=self._handle_client, args=(conn, addr), daemon=True).start()
            except socket.timeout:
                continue
            except OSError:
                break

    def _handle_client(self, conn, addr):
        while self.running:
            try:
                data = conn.recv(1)
                if not data:
                    break
            except OSError:
                break
        with self.lock:
            if conn in self.clients:
                self.clients.remove(conn)
        try:
            conn.close()
        except OSError:
            pass

    def broadcast(self, data: bytes):
        with self.lock:
            dead = []
            for c in self.clients:
                try:
                    c.sendall(data)
                except OSError:
                    dead.append(c)
            for c in dead:
                self.clients.remove(c)

    def client_count(self):
        with self.lock:
            return len(self.clients)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ARES Virtual Gamepad")
        self.setMinimumSize(500, 400)

        self.server = TcpServer(HOST, PORT)
        self.server.start()

        self._build_ui()

        self.send_timer = QTimer()
        self.send_timer.timeout.connect(self._send_state)
        self.send_timer.start(20)  # 50Hz

        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self._update_status)
        self.status_timer.start(500)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)

        left_group = QGroupBox("Left Stick (X / Y)")
        left_layout = QVBoxLayout(left_group)
        self.left_stick = JoystickWidget()
        left_layout.addWidget(self.left_stick)
        main_layout.addWidget(left_group)

        center_layout = QVBoxLayout()

        height_group = QGroupBox("Height")
        height_layout = QVBoxLayout(height_group)
        self.height_label = QLabel("0.000")
        self.height_label.setAlignment(Qt.AlignCenter)
        self.height_label.setFont(QFont("Monospace", 14))
        height_layout.addWidget(self.height_label)

        self.height_slider = QSlider(Qt.Vertical)
        self.height_slider.setMinimum(-500)
        self.height_slider.setMaximum(500)
        self.height_slider.setValue(0)
        self.height_slider.setTickPosition(QSlider.TicksBothSides)
        self.height_slider.setTickInterval(100)
        self.height_slider.valueChanged.connect(self._on_height_changed)
        height_layout.addWidget(self.height_slider)
        center_layout.addWidget(height_group)

        self.reset_btn = QPushButton("Reset All")
        self.reset_btn.setFixedHeight(40)
        self.reset_btn.clicked.connect(self._reset_all)
        center_layout.addWidget(self.reset_btn)

        center_layout.addStretch()
        main_layout.addLayout(center_layout)

        right_group = QGroupBox("Right Stick (Yaw)")
        right_layout = QVBoxLayout(right_group)
        self.right_stick = JoystickWidget()
        right_layout.addWidget(self.right_stick)
        main_layout.addWidget(right_group)

        self.statusBar().showMessage("No connections")

    def _on_height_changed(self, val):
        self.height_label.setText(f"{val / 10000:.3f}")

    def _reset_all(self):
        self.left_stick.reset()
        self.right_stick.reset()
        self.height_slider.setValue(0)

    def _send_state(self):
        lx, ly = self.left_stick.value()
        rx, _ = self.right_stick.value()
        height = self.height_slider.value() / 10000.0
        msg = json.dumps({"lx": round(lx, 3), "ly": round(ly, 3),
                          "rx": round(rx, 3), "height": round(height, 4)}) + "\n"
        self.server.broadcast(msg.encode("utf-8"))

    def _update_status(self):
        n = self.server.client_count()
        if n == 0:
            self.statusBar().showMessage(f"No connections | Listening on {HOST}:{PORT}")
        else:
            self.statusBar().showMessage(f"{n} client(s) connected | Listening on {HOST}:{PORT}")

    def closeEvent(self, event):
        self.send_timer.stop()
        self.status_timer.stop()
        self.server.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
