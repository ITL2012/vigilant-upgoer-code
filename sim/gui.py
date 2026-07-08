import sys
import os
import json
import time
import math
import numpy as np
from pathlib import Path
from collections import deque

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QSplitter, QLabel, QComboBox, QPushButton, QSlider, QGroupBox,
    QGridLayout, QFrame, QStatusBar, QToolBar, QAction, QFileDialog,
    QSpinBox, QDoubleSpinBox, QProgressBar, QMessageBox,
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont, QColor, QPalette

import matplotlib
matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from simulate import CoSimulation, FLIGHT_PHASE, PHASE_NAMES
from model_loader import RocketModel, SURFACE_CHANNELS, SURFACE_COLORS
from rocket_gl import RocketGLWidget
from surface_dialog import SurfaceAssignmentDialog
from vehicle_spec_dialog import VehicleSpecDialog

PHASE_COLORS = {
    "TRANSPORT": "#607D8B",
    "PAD": "#9E9E9E",
    "READY": "#FFC107",
    "BOOST": "#F44336",
    "COAST": "#2196F3",
    "DESCENT": "#4CAF50",
}


class TelemetryPlot(FigureCanvas):
    def __init__(self, parent=None, width=8, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi, facecolor="#1a1a2e")
        super().__init__(self.fig)
        self.setParent(parent)
        self.axes = {}
        self._setup_axes()
        self.max_points = 500

        self.time_data = deque(maxlen=self.max_points)
        self.alt_data = deque(maxlen=self.max_points)
        self.vel_data = deque(maxlen=self.max_points)
        self.roll_data = deque(maxlen=self.max_points)
        self.pitch_data = deque(maxlen=self.max_points)
        self.yaw_data = deque(maxlen=self.max_points)
        self.g_load_data = deque(maxlen=self.max_points)
        self.airspeed_data = deque(maxlen=self.max_points)
        self.servo_data = [deque(maxlen=self.max_points) for _ in range(8)]
        self.phase_data = deque(maxlen=self.max_points)
        self.kp_data = deque(maxlen=self.max_points)
        self.ki_data = deque(maxlen=self.max_points)
        self.kd_data = deque(maxlen=self.max_points)

    def _setup_axes(self):
        gs = self.fig.add_gridspec(3, 3, hspace=0.35, wspace=0.3)

        self.axes["alt"] = self.fig.add_subplot(gs[0, 0])
        self.axes["vel"] = self.fig.add_subplot(gs[0, 1])
        self.axes["att"] = self.fig.add_subplot(gs[0, 2])
        self.axes["gload"] = self.fig.add_subplot(gs[1, 0])
        self.axes["airspeed"] = self.fig.add_subplot(gs[1, 1])
        self.axes["servos"] = self.fig.add_subplot(gs[1, 2])
        self.axes["gains"] = self.fig.add_subplot(gs[2, 0])
        self.axes["phase"] = self.fig.add_subplot(gs[2, 1:])

        for key, ax in self.axes.items():
            ax.set_facecolor("#16213e")
            ax.tick_params(colors="#aaa", labelsize=7)
            for spine in ax.spines.values():
                spine.set_color("#333")
            ax.title.set_color("#ddd")
            ax.xaxis.label.set_color("#aaa")
            ax.yaxis.label.set_color("#aaa")

    def update_data(self, entry: dict):
        t = entry.get("time", 0)
        self.time_data.append(t)
        self.alt_data.append(entry.get("altitude", 0))
        self.vel_data.append(entry.get("velocity_z", 0))
        self.roll_data.append(entry.get("roll", 0))
        self.pitch_data.append(entry.get("pitch", 0))
        self.yaw_data.append(entry.get("yaw", 0))
        self.g_load_data.append(entry.get("g_load", 0))
        self.airspeed_data.append(entry.get("controller_airspeed", entry.get("airspeed", 0)))

        servos = entry.get("servo_angles", [90] * 8)
        for i in range(8):
            self.servo_data[i].append(servos[i] if i < len(servos) else 90)

        gains = entry.get("active_gains", [0, 0, 0])
        self.kp_data.append(gains[0] if len(gains) > 0 else 0)
        self.ki_data.append(gains[1] if len(gains) > 1 else 0)
        self.kd_data.append(gains[2] if len(gains) > 2 else 0)

        phase_name = entry.get("controller_phase", "PAD")
        phase_id = FLIGHT_PHASE.get(phase_name, 1)
        self.phase_data.append(phase_id)

    def refresh_plots(self):
        if len(self.time_data) < 2:
            return
        t = list(self.time_data)

        ax = self.axes["alt"]
        ax.cla()
        ax.plot(t, list(self.alt_data), color="#00e676", linewidth=1)
        ax.set_ylabel("Alt (m)", fontsize=7)
        ax.set_title("Altitude", fontsize=8, color="#ddd")
        ax.set_facecolor("#16213e")
        ax.tick_params(colors="#aaa", labelsize=7)

        ax = self.axes["vel"]
        ax.cla()
        ax.plot(t, list(self.vel_data), color="#29b6f6", linewidth=1)
        ax.set_ylabel("Vz (m/s)", fontsize=7)
        ax.set_title("Vertical Velocity", fontsize=8, color="#ddd")
        ax.set_facecolor("#16213e")
        ax.tick_params(colors="#aaa", labelsize=7)

        ax = self.axes["att"]
        ax.cla()
        ax.plot(t, list(self.roll_data), color="#ef5350", linewidth=0.8, label="Roll")
        ax.plot(t, list(self.pitch_data), color="#66bb6a", linewidth=0.8, label="Pitch")
        ax.plot(t, list(self.yaw_data), color="#42a5f5", linewidth=0.8, label="Yaw")
        ax.set_ylabel("Deg", fontsize=7)
        ax.set_title("Attitude", fontsize=8, color="#ddd")
        ax.legend(fontsize=5, loc="upper right", facecolor="#16213e", edgecolor="#333", labelcolor="#aaa")
        ax.set_facecolor("#16213e")
        ax.tick_params(colors="#aaa", labelsize=7)

        ax = self.axes["gload"]
        ax.cla()
        ax.plot(t, list(self.g_load_data), color="#ffa726", linewidth=1)
        ax.set_ylabel("G", fontsize=7)
        ax.set_title("G-Load", fontsize=8, color="#ddd")
        ax.set_facecolor("#16213e")
        ax.tick_params(colors="#aaa", labelsize=7)

        ax = self.axes["airspeed"]
        ax.cla()
        ax.plot(t, list(self.airspeed_data), color="#ab47bc", linewidth=1)
        ax.set_ylabel("m/s", fontsize=7)
        ax.set_title("Kalman Airspeed", fontsize=8, color="#ddd")
        ax.set_facecolor("#16213e")
        ax.tick_params(colors="#aaa", labelsize=7)

        ax = self.axes["servos"]
        ax.cla()
        colors = ["#ef5350", "#f44336", "#ab47bc", "#9c27b0",
                   "#2196f3", "#1e88e5", "#00bcd4", "#009688"]
        labels = ["C0", "C1", "C2", "C3", "F4", "F5", "F6", "F7"]
        for i in range(8):
            ax.plot(t, list(self.servo_data[i]), color=colors[i], linewidth=0.6, label=labels[i])
        ax.set_ylabel("Deg", fontsize=7)
        ax.set_title("Servo Angles", fontsize=8, color="#ddd")
        ax.legend(fontsize=4, loc="upper right", ncol=2, facecolor="#16213e", edgecolor="#333", labelcolor="#aaa")
        ax.set_facecolor("#16213e")
        ax.tick_params(colors="#aaa", labelsize=7)

        ax = self.axes["gains"]
        ax.cla()
        ax.plot(t, list(self.kp_data), color="#ef5350", linewidth=0.8, label="Kp")
        ax.plot(t, list(self.ki_data), color="#66bb6a", linewidth=0.8, label="Ki")
        ax.plot(t, list(self.kd_data), color="#42a5f5", linewidth=0.8, label="Kd")
        ax.set_ylabel("Gain", fontsize=7)
        ax.set_title("Active PID Gains", fontsize=8, color="#ddd")
        ax.legend(fontsize=5, loc="upper right", facecolor="#16213e", edgecolor="#333", labelcolor="#aaa")
        ax.set_facecolor("#16213e")
        ax.tick_params(colors="#aaa", labelsize=7)

        ax = self.axes["phase"]
        ax.cla()
        ax.plot(t, list(self.phase_data), color="#ffa726", linewidth=1.5, drawstyle="steps-post")
        ax.set_ylabel("Phase", fontsize=7)
        ax.set_yticks([0, 1, 2, 3, 4, 5])
        ax.set_yticklabels(["TRN", "PAD", "RDY", "BST", "CST", "DSC"], fontsize=6)
        ax.set_title("Flight Phase", fontsize=8, color="#ddd")
        ax.set_facecolor("#16213e")
        ax.tick_params(colors="#aaa", labelsize=7)

        self.draw_idle()


class SimWindow(QMainWindow):
    def __init__(self, scenario_dir: str = None):
        super().__init__()
        self.setWindowTitle("Rocket Flight Co-Simulator")
        self.setGeometry(50, 50, 1600, 950)
        self.setStyleSheet("""
            QMainWindow { background-color: #1a1a2e; }
            QWidget { background-color: #1a1a2e; color: #ddd; }
            QGroupBox { border: 1px solid #333; border-radius: 4px; margin-top: 8px; padding-top: 12px; color: #ddd; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; color: #ddd; }
            QPushButton { background-color: #16213e; border: 1px solid #333; border-radius: 4px; padding: 6px 14px; color: #ddd; }
            QPushButton:hover { background-color: #1a2744; }
            QPushButton:pressed { background-color: #0f3460; }
            QComboBox { background-color: #16213e; border: 1px solid #333; border-radius: 4px; padding: 4px; color: #ddd; }
            QComboBox QAbstractItemView { background-color: #16213e; color: #ddd; selection-background-color: #0f3460; }
            QLabel { color: #ddd; }
            QProgressBar { background-color: #16213e; border: 1px solid #333; border-radius: 4px; text-align: center; color: #ddd; }
            QProgressBar::chunk { background-color: #0f3460; border-radius: 3px; }
            QSlider::groove:horizontal { background: #16213e; height: 6px; border-radius: 3px; }
            QSlider::handle:horizontal { background: #0f3460; width: 14px; margin: -4px 0; border-radius: 7px; }
        """)

        if scenario_dir is None:
            scenario_dir = str(Path(__file__).parent / "scenarios")
        self.scenario_dir = scenario_dir
        self.sim = None
        self.running = False
        self.sim_speed = 1.0

        self.rocket_model = RocketModel()

        self._build_ui()
        self._load_scenarios()

        self.sim_timer = QTimer()
        self.sim_timer.timeout.connect(self._sim_tick)
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self._update_plots)
        self.plot_counter = 0

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(6, 6, 6, 6)

        toolbar = QHBoxLayout()
        self.scenario_combo = QComboBox()
        self.scenario_combo.setMinimumWidth(200)
        toolbar.addWidget(QLabel("Scenario:"))
        toolbar.addWidget(self.scenario_combo)

        self.btn_load = QPushButton("Load")
        self.btn_load.clicked.connect(self._load_scenario)
        toolbar.addWidget(self.btn_load)

        toolbar.addSpacing(20)

        self.btn_start = QPushButton("Start")
        self.btn_start.clicked.connect(self._start_sim)
        toolbar.addWidget(self.btn_start)

        self.btn_pause = QPushButton("Pause")
        self.btn_pause.clicked.connect(self._pause_sim)
        self.btn_pause.setEnabled(False)
        toolbar.addWidget(self.btn_pause)

        self.btn_reset = QPushButton("Reset")
        self.btn_reset.clicked.connect(self._reset_sim)
        toolbar.addWidget(self.btn_reset)

        toolbar.addSpacing(20)

        self.btn_load_model = QPushButton("Load Model")
        self.btn_load_model.clicked.connect(self._on_load_model)
        toolbar.addWidget(self.btn_load_model)

        self.btn_assign_surfaces = QPushButton("Assign Surfaces")
        self.btn_assign_surfaces.clicked.connect(self._on_assign_surfaces)
        self.btn_assign_surfaces.setEnabled(False)
        toolbar.addWidget(self.btn_assign_surfaces)

        self.btn_extract_props = QPushButton("Extract Props")
        self.btn_extract_props.clicked.connect(self._on_extract_props)
        self.btn_extract_props.setEnabled(False)
        toolbar.addWidget(self.btn_extract_props)

        self.model_label = QLabel("(no model)")
        self.model_label.setStyleSheet("font-size: 10px; color: #888;")
        toolbar.addWidget(self.model_label)

        toolbar.addSpacing(20)

        toolbar.addWidget(QLabel("Speed:"))
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(1, 20)
        self.speed_slider.setValue(10)
        self.speed_slider.setFixedWidth(120)
        self.speed_slider.valueChanged.connect(self._speed_changed)
        toolbar.addWidget(self.speed_slider)
        self.speed_label = QLabel("1.0x")
        toolbar.addWidget(self.speed_label)

        toolbar.addSpacing(20)

        self.phase_label = QLabel("Phase: ---")
        self.phase_label.setStyleSheet("font-weight: bold; font-size: 14px; padding: 4px 8px; border-radius: 4px; background-color: #333;")
        toolbar.addWidget(self.phase_label)

        self.time_label = QLabel("T+0.00s")
        self.time_label.setStyleSheet("font-size: 13px;")
        toolbar.addWidget(self.time_label)

        toolbar.addStretch()
        main_layout.addLayout(toolbar)

        splitter = QSplitter(Qt.Horizontal)

        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)

        self.telemetry_plot = TelemetryPlot(width=9, height=6, dpi=90)
        left_layout.addWidget(self.telemetry_plot)

        status_grid = QGridLayout()
        status_grid.setSpacing(4)

        self.val_labels = {}
        label_defs = [
            ("Altitude", 0, 0), ("Velocity Z", 0, 1),
            ("Airspeed", 0, 2), ("G-Load", 0, 3),
            ("Roll", 1, 0), ("Pitch", 1, 1),
            ("Yaw", 1, 2), ("Thrust", 1, 3),
            ("Kp", 2, 0), ("Ki", 2, 1),
            ("Kd", 2, 2), ("Log Drops", 2, 3),
        ]

        for name, row, col in label_defs:
            lbl = QLabel(f"{name}: ---")
            lbl.setStyleSheet("font-size: 11px; padding: 2px 6px; background-color: #16213e; border-radius: 3px;")
            status_grid.addWidget(lbl, row, col)
            self.val_labels[name] = (lbl, name)

        servo_grid = QGridLayout()
        servo_grid.setSpacing(2)
        servo_names = ["C0-P+", "C1-P-", "C2-Y+", "C3-Y-", "F4-R+", "F5-R-", "F6-R+", "F7-R-"]
        self.servo_labels = []
        for i, name in enumerate(servo_names):
            lbl = QLabel(f"{name}: 90.0")
            lbl.setStyleSheet("font-size: 10px; padding: 1px 4px; background-color: #16213e; border-radius: 2px;")
            r, c = i // 4, i % 4
            servo_grid.addWidget(lbl, r, c)
            self.servo_labels.append(lbl)

        left_layout.addLayout(status_grid)
        left_layout.addLayout(servo_grid)

        splitter.addWidget(left_panel)

        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)

        self.rocket_3d = RocketGLWidget()
        right_layout.addWidget(self.rocket_3d)

        splitter.addWidget(right_panel)

        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)
        main_layout.addWidget(splitter, stretch=1)

        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 3000)
        self.progress_bar.setValue(0)
        self.progress_bar.setFixedHeight(16)
        main_layout.addWidget(self.progress_bar)

        self.statusBar().showMessage("Ready - load a scenario to begin")
        self.statusBar().setStyleSheet("color: #aaa; background-color: #1a1a2e;")

    def _load_scenarios(self):
        self.scenario_combo.clear()
        if os.path.isdir(self.scenario_dir):
            for f in sorted(os.listdir(self.scenario_dir)):
                if f.endswith(".json"):
                    self.scenario_combo.addItem(f)

    def _load_scenario(self):
        name = self.scenario_combo.currentText()
        if not name:
            return
        path = os.path.join(self.scenario_dir, name)
        try:
            self.sim = CoSimulation(path)
            self.statusBar().showMessage(f"Loaded: {name}")
        except Exception as e:
            QMessageBox.warning(self, "Load Error", str(e))

    def _start_sim(self):
        if self.sim is None:
            self._load_scenario()
        if self.sim is None:
            return

        self.running = True
        self.btn_start.setEnabled(False)
        self.btn_pause.setEnabled(True)

        self.sim_timer.start(10)
        self.plot_timer.start(200)

    def _pause_sim(self):
        self.running = False
        self.sim_timer.stop()
        self.plot_timer.stop()
        self.btn_start.setEnabled(True)
        self.btn_pause.setEnabled(False)

    def _reset_sim(self):
        self._pause_sim()
        if self.sim:
            self.sim.reset()
        self.telemetry_plot = TelemetryPlot(width=9, height=6, dpi=90)
        self.rocket_3d.trail_positions.clear()
        self.progress_bar.setValue(0)
        self.time_label.setText("T+0.00s")
        self.phase_label.setText("Phase: ---")
        self.statusBar().showMessage("Reset")

    def _speed_changed(self, val):
        self.sim_speed = val / 10.0
        self.speed_label.setText(f"{self.sim_speed:.1f}x")

    def _on_load_model(self):
        file_filter = "3D Models (*.obj *.stl *.ply *.glb *.gltf *.step *.stp);;All Files (*)"
        path, _ = QFileDialog.getOpenFileName(self, "Load Rocket Model", "", file_filter)
        if not path:
            return

        self.rocket_model = RocketModel()
        if self.rocket_model.load_model(path):
            n_parts = len(self.rocket_model.get_part_names())
            self.btn_assign_surfaces.setEnabled(True)
            self.btn_extract_props.setEnabled(True)
            self.model_label.setText(f"{Path(path).name} ({n_parts} parts)")
            self.rocket_3d.set_rocket_model(self.rocket_model)
            self.statusBar().showMessage(f"Model loaded: {Path(path).name} — {n_parts} mesh parts")
        else:
            QMessageBox.warning(self, "Load Error", f"Failed to load model: {path}")
            self.model_label.setText("(no model)")

    def _on_assign_surfaces(self):
        if not self.rocket_model.valid:
            QMessageBox.information(self, "No Model", "Load a 3D model first.")
            return
        dlg = SurfaceAssignmentDialog(self.rocket_model, self)
        if dlg.exec_() == dlg.Accepted:
            self.rocket_3d.set_rocket_model(self.rocket_model)
            n_assigned = len(self.rocket_model.surface_assignments)
            self.statusBar().showMessage(f"Surfaces assigned: {n_assigned} channels configured")
        else:
            self.rocket_3d.set_rocket_model(self.rocket_model)

    def _on_extract_props(self):
        if not self.rocket_model.valid:
            QMessageBox.information(self, "No Model", "Load a 3D model first.")
            return
        dlg = VehicleSpecDialog(self.rocket_model, self)
        dlg.exec_()

    def _sim_tick(self):
        if not self.running or self.sim is None:
            return

        steps_per_tick = max(1, int(self.sim_speed))
        for _ in range(steps_per_tick):
            entry = self.sim.step()
            self.telemetry_plot.update_data(entry)
            self.plot_counter += 1

        phase = entry.get("controller_phase", "PAD")
        color = PHASE_COLORS.get(phase, "#607D8B")
        self.phase_label.setText(f"Phase: {phase}")
        self.phase_label.setStyleSheet(f"font-weight: bold; font-size: 14px; padding: 4px 8px; border-radius: 4px; background-color: {color}; color: #fff;")

        self.time_label.setText(f"T+{entry['time']:.2f}s")

        alt = entry.get("altitude", 0)
        vel = entry.get("velocity_z", 0)
        aspd = entry.get("controller_airspeed", 0)
        gload = entry.get("g_load", 0)
        roll = entry.get("roll", 0)
        pitch = entry.get("pitch", 0)
        yaw = entry.get("yaw", 0)
        thrust = entry.get("thrust", 0)
        gains = entry.get("active_gains", [0, 0, 0])
        log_drops = entry.get("log_drops", 0)

        updates = {
            "Altitude": f"Alt: {alt:.1f} m",
            "Velocity Z": f"Vz: {vel:.1f} m/s",
            "Airspeed": f"AS: {aspd:.1f} m/s",
            "G-Load": f"G: {gload:.1f}",
            "Roll": f"R: {roll:.2f} d",
            "Pitch": f"P: {pitch:.2f} d",
            "Yaw": f"Y: {yaw:.2f} d",
            "Thrust": f"F: {thrust:.0f} N",
            "Kp": f"Kp: {gains[0]:.2f}",
            "Ki": f"Ki: {gains[1]:.2f}",
            "Kd": f"Kd: {gains[2]:.3f}",
            "Log Drops": f"Drops: {log_drops}",
        }
        for key, text in updates.items():
            if key in self.val_labels:
                self.val_labels[key][0].setText(text)

        servos = entry.get("servo_angles", [90] * 8)
        servo_names_short = ["C0+", "C1-", "C2+", "C3-", "F4+", "F5-", "F6+", "F7-"]
        for i, lbl in enumerate(self.servo_labels):
            angle = servos[i] if i < len(servos) else 90
            lbl.setText(f"{servo_names_short[i]}: {angle:.1f}")

        progress = min(int(entry["time"] / 30.0 * 3000), 3000)
        self.progress_bar.setValue(progress)

        self.rocket_3d.update_rocket(
            qx=entry.get("qx", 0),
            qy=entry.get("qy", 0),
            qz=entry.get("qz", 0),
            qw=entry.get("qw", 1),
            altitude=entry.get("altitude", 0),
            servo_angles=entry.get("servo_angles"),
        )

        if alt < -5.0 and entry["time"] > 3.0:
            self._pause_sim()
            self.statusBar().showMessage("Simulation ended - vehicle below ground")

    def _update_plots(self):
        if self.sim is None:
            return
        self.telemetry_plot.refresh_plots()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    palette = QPalette()
    palette.setColor(QPalette.Window, QColor("#1a1a2e"))
    palette.setColor(QPalette.WindowText, QColor("#ddd"))
    palette.setColor(QPalette.Base, QColor("#16213e"))
    palette.setColor(QPalette.AlternateBase, QColor("#1a1a2e"))
    palette.setColor(QPalette.ToolTipBase, QColor("#16213e"))
    palette.setColor(QPalette.ToolTipText, QColor("#ddd"))
    palette.setColor(QPalette.Text, QColor("#ddd"))
    palette.setColor(QPalette.Button, QColor("#16213e"))
    palette.setColor(QPalette.ButtonText, QColor("#ddd"))
    palette.setColor(QPalette.Highlight, QColor("#0f3460"))
    palette.setColor(QPalette.HighlightedText, QColor("#fff"))
    app.setPalette(palette)

    window = SimWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
