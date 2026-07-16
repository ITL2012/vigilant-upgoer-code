#!/usr/bin/env python3
"""
ISAAC-L Flight Profile 3D Visualizer & Editor
==============================================
View, edit, and export flight profiles (attitude trajectories) in 3D.

The trajectory is rendered as a 3D rocket orientation path: starting upright
at the origin, each profile step rotates the rocket's body frame by the
step's (roll, pitch, yaw) target. The 3D path shows the rocket's nose vector
over time, so you can literally see "rotate 180, pitch 20, then flatten".

Features:
  - Load/save the SAME JSON schema the firmware uses
  - Drag-step editing (roll/pitch/yaw/duration/trigger/rate)
  - 3D animated trajectory preview
  - Export directly to the SD profile folder or POST to the flight controller

Usage:
  python3 profile_visualizer.py [profile.json]
  python3 profile_visualizer.py --upload http://192.168.4.1 MY_PROFILE

JSON schema (matches firmware /src/flight_profile.h):
{
  "name": "MY_PROFILE",
  "loop": false,
  "steps": [
    {"roll":180,"pitch":0,"yaw":0,"duration":5.0,"trigger":"time","value":0,"rate":40},
    {"roll":0,"pitch":0,"yaw":0,"trigger":"apogee","rate":25}
  ]
}
"""

import sys
import json
import math
import argparse

import numpy as np
import matplotlib

matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers 3d proj)

from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QLineEdit,
    QLabel,
    QDoubleSpinBox,
    QComboBox,
    QCheckBox,
    QFileDialog,
    QMessageBox,
    QGroupBox,
)
from PyQt5.QtCore import Qt

# ---- Trigger choices (must match firmware triggerFromName) ----
TRIGGERS = ["time", "apogee", "altitude", "velocity", "manual"]


# ============================================================================
# Quaternion / attitude math (matches firmware conventions, degrees in/out)
# ============================================================================
def deg2rad(d):
    return d * math.pi / 180.0


def quat_from_euler(roll_deg, pitch_deg, yaw_deg):
    """Z-Y-X (yaw-pitch-roll) intrinsic -> quaternion [w,x,y,z]."""
    r, p, y = deg2rad(roll_deg), deg2rad(pitch_deg), deg2rad(yaw_deg)
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y_ = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return np.array([w, x, y_, z])


def quat_mult(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def rotate_vector(q, v):
    """Rotate vector v by quaternion q."""
    vq = np.array([0.0, v[0], v[1], v[2]])
    qc = np.array([q[0], -q[1], -q[2], -q[3]])
    res = quat_mult(quat_mult(q, vq), qc)
    return res[1:]


# ============================================================================
# Profile model
# ============================================================================
def default_profile():
    return {
        "name": "NEW_PROFILE",
        "loop": False,
        "steps": [
            {"roll": 0, "pitch": 0, "yaw": 0, "duration": 3.0,
             "trigger": "time", "value": 0, "rate": 40},
            {"roll": 180, "pitch": 0, "yaw": 0, "duration": 5.0,
             "trigger": "time", "value": 0, "rate": 40},
            {"roll": 0, "pitch": 0, "yaw": 0, "duration": 3.0,
             "trigger": "time", "value": 0, "rate": 40},
            {"roll": 0, "pitch": 20, "yaw": 0, "duration": 4.0,
             "trigger": "time", "value": 0, "rate": 30},
            {"roll": 0, "pitch": 0, "yaw": 0, "trigger": "apogee", "rate": 25},
        ],
    }


def load_profile(path):
    with open(path, "r") as f:
        data = json.load(f)
    # normalize step keys
    for s in data.get("steps", []):
        s.setdefault("duration", 0.0)
        s.setdefault("trigger", "time")
        s.setdefault("value", 0.0)
        s.setdefault("rate", 0.0)
        s.setdefault("roll", 0.0)
        s.setdefault("pitch", 0.0)
        s.setdefault("yaw", 0.0)
    return data


def save_profile(data, path):
    with open(path, "w") as f:
        json.dump(data, f, indent=2)


# ============================================================================
# Trajectory generation (matches firmware FlightProfileEngine S-curve)
# ============================================================================
def quintic(t):
    if t <= 0:
        return 0.0
    if t >= 1:
        return 1.0
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t))


def build_trajectory(data, samples_per_step=30):
    """Return list of (time, roll, pitch, yaw, nose_vec) along the path."""
    steps = data.get("steps", [])
    traj = []
    t_total = 0.0
    prev_euler = (0.0, 0.0, 0.0)
    for i, step in enumerate(steps):
        dur = step.get("duration", 0.0)
        trig = step.get("trigger", "time")
        # For non-time triggers, use a nominal display duration so we can draw
        nominal = dur if dur > 0 else (3.0 if trig != "manual" else 1.0)
        target = (step.get("roll", 0), step.get("pitch", 0), step.get("yaw", 0))
        n = max(2, int(samples_per_step * nominal)) if trig == "time" else samples_per_step
        for k in range(n):
            frac = k / (n - 1) if n > 1 else 1.0
            s = quintic(frac)
            roll = prev_euler[0] + (target[0] - prev_euler[0]) * s
            pitch = prev_euler[1] + (target[1] - prev_euler[1]) * s
            yaw = prev_euler[2] + (target[2] - prev_euler[2]) * s
            q = quat_from_euler(roll, pitch, yaw)
            nose = rotate_vector(q, np.array([0.0, 0.0, 1.0]))
            traj.append((t_total + frac * nominal, roll, pitch, yaw, nose))
            if k == n - 1:
                prev_euler = target
        t_total += nominal
    return traj


# ============================================================================
# 3D Canvas
# ============================================================================
class TrajectoryCanvas(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = Figure(figsize=(7, 6))
        super().__init__(self.fig)
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.parent = parent
        self.traj = []
        self.anim = None

    def draw_trajectory(self, data):
        self.traj = build_trajectory(data)
        self.ax.clear()
        self.ax.set_xlabel("X (body right)")
        self.ax.set_ylabel("Y (body fwd)")
        self.ax.set_zlabel("Z (up / nose)")
        self.ax.set_title("Expected Attitude Trajectory\n(nose vector over time)")

        if not self.traj:
            self.draw()
            return

        xs = [p[4][0] for p in self.traj]
        ys = [p[4][1] for p in self.traj]
        zs = [p[4][2] for p in self.traj]

        # Color by progress
        from matplotlib.cm import viridis

        self.ax.plot(xs, ys, zs, color="#00ffaa", linewidth=2, zorder=1)
        # Scatter with time-coloring
        ts = np.linspace(0, 1, len(self.traj))
        self.ax.scatter(xs, ys, zs, c=ts, cmap="viridis", s=12, zorder=2)

        # Draw rocket body glyphs at each step boundary
        steps = data.get("steps", [])
        t_acc = 0.0
        for step in steps:
            dur = step.get("duration", 0.0)
            trig = step.get("trigger", "time")
            nominal = dur if dur > 0 else (3.0 if trig != "manual" else 1.0)
            # find nearest traj sample
            idx = min(
                range(len(self.traj)),
                key=lambda i: abs(self.traj[i][0] - t_acc),
            )
            _, r, p, yw, nose = self.traj[idx]
            self._draw_rocket_glyph(r, p, yw, nose, t_acc / max(1e-6, self.traj[-1][0]))
            t_acc += nominal

        # Axes limits fixed for stable view
        self.ax.set_xlim(-1.2, 1.2)
        self.ax.set_ylim(-1.2, 1.2)
        self.ax.set_zlim(-1.2, 1.2)
        self.ax.view_init(elev=20, azim=45)
        self.draw()

    def _draw_rocket_glyph(self, roll, pitch, yaw, nose, color_t):
        from matplotlib.cm import viridis

        q = quat_from_euler(roll, pitch, yaw)
        # body axis (tail -> nose)
        tail = -nose
        up = rotate_vector(q, np.array([0.0, 1.0, 0.0]))
        # draw a short stick: tail (scaled) to nose
        self.ax.plot(
            [tail[0] * 0.6, nose[0] * 1.0],
            [tail[1] * 0.6, nose[1] * 1.0],
            [tail[2] * 0.6, nose[2] * 1.0],
            color=viridis(color_t),
            linewidth=3,
        )
        # fins hint
        for ang in [0, 120, 240]:
            a = deg2rad(ang)
            fin = tail * 0.6 + 0.25 * (
                math.cos(a) * np.array([1.0, 0.0, 0.0])
                + math.sin(a) * np.array([0.0, 1.0, 0.0])
            )
            self.ax.plot(
                [tail[0] * 0.6, fin[0]],
                [tail[1] * 0.6, fin[1]],
                [tail[2] * 0.6, fin[2]],
                color=viridis(color_t),
                alpha=0.5,
                linewidth=1,
            )


# ============================================================================
# Main window
# ============================================================================
class MainWindow(QMainWindow):
    def __init__(self, initial=None):
        super().__init__()
        self.setWindowTitle("ISAAC-L Flight Profile Visualizer")
        self.resize(1100, 720)
        self.data = default_profile() if initial is None else load_profile(initial)

        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)

        # ---- Left: 3D view ----
        left = QVBoxLayout()
        self.canvas = TrajectoryCanvas(self)
        left.addWidget(self.canvas)

        # Animation controls
        anim_row = QHBoxLayout()
        self.btn_play = QPushButton("Animate")
        self.btn_play.clicked.connect(self.toggle_animation)
        self.chk_loop = QCheckBox("Loop profile")
        self.chk_loop.stateChanged.connect(lambda: self._set_loop())
        anim_row.addWidget(self.btn_play)
        anim_row.addWidget(self.chk_loop)
        left.addLayout(anim_row)
        root.addLayout(left, stretch=3)

        # ---- Right: editor ----
        right = QVBoxLayout()

        # Name + loop
        name_row = QHBoxLayout()
        name_row.addWidget(QLabel("Name:"))
        self.edit_name = QLineEdit(self.data.get("name", "NEW_PROFILE"))
        self.edit_name.textChanged.connect(lambda t: self._set_name(t))
        name_row.addWidget(self.edit_name)
        right.addLayout(name_row)

        # Step list
        right.addWidget(QLabel("Steps:"))
        self.list_steps = QListWidget()
        self.list_steps.currentRowChanged.connect(self._on_step_selected)
        right.addWidget(self.list_steps)

        btns = QHBoxLayout()
        self.btn_add = QPushButton("+ Step")
        self.btn_add.clicked.connect(self._add_step)
        self.btn_del = QPushButton("- Step")
        self.btn_del.clicked.connect(self._del_step)
        self.btn_up = QPushButton("Up")
        self.btn_up.clicked.connect(lambda: self._move_step(-1))
        self.btn_down = QPushButton("Down")
        self.btn_down.clicked.connect(lambda: self._move_step(1))
        btns.addWidget(self.btn_add)
        btns.addWidget(self.btn_del)
        btns.addWidget(self.btn_up)
        btns.addWidget(self.btn_down)
        right.addLayout(btns)

        # Step editor group
        self.grp_step = QGroupBox("Step Editor")
        sg = QVBoxLayout()

        def make_spin(label, minv, maxv, dec, key):
            row = QHBoxLayout()
            row.addWidget(QLabel(label))
            sp = QDoubleSpinBox()
            sp.setRange(minv, maxv)
            sp.setDecimals(dec)
            sp.setSingleStep(1.0 if dec == 0 else 0.5)
            sp.valueChanged.connect(lambda v, k=key: self._set_step_field(k, v))
            row.addWidget(sp)
            sg.addLayout(row)
            return sp

        self.sp_roll = make_spin("Roll (deg)", -180, 180, 1, "roll")
        self.sp_pitch = make_spin("Pitch (deg)", -90, 90, 1, "pitch")
        self.sp_yaw = make_spin("Yaw (deg)", -360, 360, 1, "yaw")
        self.sp_dur = make_spin("Duration (s)", 0, 120, 1, "duration")
        self.sp_rate = make_spin("Max rate (deg/s)", 0, 200, 1, "rate")
        self.sp_value = make_spin("Trigger value", -1000, 1000, 2, "value")

        trig_row = QHBoxLayout()
        trig_row.addWidget(QLabel("Trigger:"))
        self.cmb_trig = QComboBox()
        self.cmb_trig.addItems(TRIGGERS)
        self.cmb_trig.currentTextChanged.connect(lambda t: self._set_step_field("trigger", t))
        trig_row.addWidget(self.cmb_trig)
        sg.addLayout(trig_row)

        self.grp_step.setLayout(sg)
        right.addWidget(self.grp_step)

        # File / upload actions
        file_row = QHBoxLayout()
        self.btn_load = QPushButton("Open JSON")
        self.btn_load.clicked.connect(self._load)
        self.btn_save = QPushButton("Save JSON")
        self.btn_save.clicked.connect(self._save)
        self.btn_export = QPushButton("Export to SD")
        self.btn_export.clicked.connect(self._export_sd)
        file_row.addWidget(self.btn_load)
        file_row.addWidget(self.btn_save)
        file_row.addWidget(self.btn_export)
        right.addLayout(file_row)

        self.edit_upload = QLineEdit("http://192.168.4.1")
        right.addWidget(QLabel("Controller URL (for upload):"))
        right.addWidget(self.edit_upload)
        self.btn_upload = QPushButton("Upload to Controller")
        self.btn_upload.clicked.connect(self._upload)
        right.addWidget(self.btn_upload)

        root.addLayout(right, stretch=2)

        self._refresh_list()
        self._refresh_canvas()

    # ---- data helpers ----
    def _refresh_list(self):
        self.list_steps.blockSignals(True)
        self.list_steps.clear()
        for i, s in enumerate(self.data.get("steps", [])):
            txt = f"#{i} R{s.get('roll',0):.0f} P{s.get('pitch',0):.0f} Y{s.get('yaw',0):.0f} [{s.get('trigger','time')}]"
            self.list_steps.addItem(QListWidgetItem(txt))
        self.list_steps.blockSignals(False)
        if self.list_steps.count() > 0:
            self.list_steps.setCurrentRow(0)

    def _refresh_canvas(self):
        self.canvas.draw_trajectory(self.data)

    def _refresh_step_editor(self):
        row = self.list_steps.currentRow()
        steps = self.data.get("steps", [])
        if row < 0 or row >= len(steps):
            self.grp_step.setEnabled(False)
            return
        self.grp_step.setEnabled(True)
        s = steps[row]
        self.sp_roll.blockSignals(True)
        self.sp_pitch.blockSignals(True)
        self.sp_yaw.blockSignals(True)
        self.sp_dur.blockSignals(True)
        self.sp_rate.blockSignals(True)
        self.sp_value.blockSignals(True)
        self.cmb_trig.blockSignals(True)
        self.sp_roll.setValue(s.get("roll", 0))
        self.sp_pitch.setValue(s.get("pitch", 0))
        self.sp_yaw.setValue(s.get("yaw", 0))
        self.sp_dur.setValue(s.get("duration", 0))
        self.sp_rate.setValue(s.get("rate", 0))
        self.sp_value.setValue(s.get("value", 0))
        self.cmb_trig.setCurrentText(s.get("trigger", "time"))
        self.sp_roll.blockSignals(False)
        self.sp_pitch.blockSignals(False)
        self.sp_yaw.blockSignals(False)
        self.sp_dur.blockSignals(False)
        self.sp_rate.blockSignals(False)
        self.sp_value.blockSignals(False)
        self.cmb_trig.blockSignals(False)

    # ---- callbacks ----
    def _on_step_selected(self, row):
        self._refresh_step_editor()

    def _set_step_field(self, key, val):
        row = self.list_steps.currentRow()
        steps = self.data.get("steps", [])
        if row < 0 or row >= len(steps):
            return
        steps[row][key] = val
        self._refresh_list()
        self.list_steps.setCurrentRow(row)
        self._refresh_canvas()

    def _set_name(self, txt):
        self.data["name"] = txt

    def _set_loop(self):
        self.data["loop"] = self.chk_loop.isChecked()

    def _add_step(self):
        self.data.setdefault("steps", []).append(
            {"roll": 0, "pitch": 0, "yaw": 0, "duration": 3.0,
             "trigger": "time", "value": 0, "rate": 40}
        )
        self._refresh_list()
        self.list_steps.setCurrentRow(self.list_steps.count() - 1)
        self._refresh_canvas()

    def _del_step(self):
        row = self.list_steps.currentRow()
        steps = self.data.get("steps", [])
        if 0 <= row < len(steps):
            steps.pop(row)
            self._refresh_list()
            self._refresh_canvas()

    def _move_step(self, d):
        row = self.list_steps.currentRow()
        steps = self.data.get("steps", [])
        j = row + d
        if 0 <= row < len(steps) and 0 <= j < len(steps):
            steps[row], steps[j] = steps[j], steps[row]
            self._refresh_list()
            self.list_steps.setCurrentRow(j)
            self._refresh_canvas()

    def _load(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Open profile", "", "JSON (*.json)"
        )
        if path:
            self.data = load_profile(path)
            self.edit_name.setText(self.data.get("name", ""))
            self.chk_loop.setChecked(self.data.get("loop", False))
            self._refresh_list()
            self._refresh_canvas()

    def _save(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "Save profile", self.data.get("name", "profile") + ".json", "JSON (*.json)"
        )
        if path:
            save_profile(self.data, path)
            QMessageBox.information(self, "Saved", f"Wrote {path}")

    def _export_sd(self):
        """Open a file dialog to write into the mounted SD /profiles folder."""
        path, _ = QFileDialog.getSaveFileName(
            self, "Export to SD profiles folder",
            "/media/ISAAC/profiles/" + self.data.get("name", "profile") + ".json",
            "JSON (*.json)",
        )
        if path:
            save_profile(self.data, path)
            QMessageBox.information(self, "Exported", f"Copied to SD: {path}")

    def _upload(self):
        try:
            import urllib.request
            import urllib.parse

            url = self.edit_upload.text().rstrip("/") + "/profile_upload"
            data = urllib.parse.urlencode(
                {"name": self.data.get("name", "profile"), "json": json.dumps(self.data)}
            ).encode()
            req = urllib.request.Request(url, data=data, method="POST")
            with urllib.request.urlopen(req, timeout=5) as resp:
                body = resp.read().decode()
            QMessageBox.information(self, "Upload", body)
        except Exception as e:
            QMessageBox.warning(self, "Upload failed", str(e))

    def toggle_animation(self):
        if self.canvas.anim and self.canvas.anim.event_source:
            self.canvas.anim.event_source.stop()
            self.canvas.anim = None
            self.btn_play.setText("Animate")
            return
        self.btn_play.setText("Stop")
        self._animate()

    def _animate(self):
        traj = self.canvas.traj
        if not traj:
            return
        ax = self.canvas.ax
        (line,) = ax.plot([], [], [], color="red", linewidth=3)
        n = len(traj)

        def update(frame):
            end = int((frame + 1) / 30.0 * n) + 1
            end = min(end, n)
            xs = [p[4][0] for p in traj[:end]]
            ys = [p[4][1] for p in traj[:end]]
            zs = [p[4][2] for p in traj[:end]]
            line.set_data(xs, ys)
            line.set_3d_properties(zs)
            return (line,)

        self.canvas.anim = animation.FuncAnimation(
            self.canvas.fig, update, frames=30, interval=50, blit=False, repeat=self.chk_loop.isChecked()
        )
        self.canvas.draw()


def main():
    ap = argparse.ArgumentParser(description="ISAAC-L Flight Profile 3D Visualizer")
    ap.add_argument("profile", nargs="?", help="profile JSON to open")
    ap.add_argument("--upload", metavar="URL", help="upload to controller URL and exit")
    ap.add_argument("--name", help="profile name when uploading")
    args = ap.parse_args()

    if args.upload:
        # headless upload mode
        path = args.profile
        if not path:
            print("ERROR: need a profile JSON file to upload")
            sys.exit(1)
        data = load_profile(path)
        if args.name:
            data["name"] = args.name
        import urllib.request
        import urllib.parse

        url = args.upload.rstrip("/") + "/profile_upload"
        payload = urllib.parse.urlencode(
            {"name": data.get("name", "profile"), "json": json.dumps(data)}
        ).encode()
        req = urllib.request.Request(url, data=payload, method="POST")
        with urllib.request.urlopen(req, timeout=5) as resp:
            print(resp.read().decode())
        return

    app = QApplication(sys.argv)
    win = MainWindow(initial=args.profile)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
