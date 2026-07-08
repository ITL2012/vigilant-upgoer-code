import json
import numpy as np
from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QLabel, QComboBox, QDoubleSpinBox, QPushButton, QScrollArea,
    QWidget, QFileDialog, QMessageBox, QCheckBox,
)
from PyQt5.QtCore import Qt
from model_loader import RocketModel, SurfaceAssignment, SURFACE_CHANNELS, SURFACE_COLORS


class SurfaceRow(QGroupBox):
    def __init__(self, channel_info: dict, color: tuple, part_names: list, parent=None):
        super().__init__(parent)
        self.channel_info = channel_info
        self.color = color
        self._enabled = False

        ch_id = channel_info["id"]
        label = channel_info["label"]
        axis = channel_info["axis"]
        sign = channel_info["sign"]

        self.setTitle(f"Ch{ch_id}: {label} ({axis}, {'+'if sign > 0 else '-'})")
        self.setStyleSheet(
            f"SurfaceRow {{ border: 2px solid rgb({int(color[0]*255)},{int(color[1]*255)},{int(color[2]*255)}); "
            f"border-radius: 4px; margin-top: 8px; padding-top: 12px; color: #ddd; }}"
            f"SurfaceRow::title {{ color: rgb({int(color[0]*255)},{int(color[1]*255)},{int(color[2]*255)}); }}"
        )

        layout = QGridLayout(self)
        layout.setSpacing(4)

        self.enable_cb = QCheckBox("Enabled")
        self.enable_cb.stateChanged.connect(self._on_enable)
        layout.addWidget(self.enable_cb, 0, 0, 1, 2)

        layout.addWidget(QLabel("Part:"), 1, 0)
        self.part_combo = QComboBox()
        self.part_combo.addItems(["(none)"] + part_names)
        self.part_combo.setEnabled(False)
        layout.addWidget(self.part_combo, 1, 1)

        self.pivot_spins = []
        for i, axis_label in enumerate(["Pivot X:", "Pivot Y:", "Pivot Z:"]):
            layout.addWidget(QLabel(axis_label), 2 + i // 2, (i % 2) * 2)
            spin = QDoubleSpinBox()
            spin.setRange(-1000, 1000)
            spin.setDecimals(4)
            spin.setSingleStep(0.001)
            spin.setEnabled(False)
            layout.addWidget(spin, 2 + i // 2, (i % 2) * 2 + 1)
            self.pivot_spins.append(spin)

        row_offset = 4
        self.hinge_spins = []
        for i, axis_label in enumerate(["Hinge X:", "Hinge Y:", "Hinge Z:"]):
            layout.addWidget(QLabel(axis_label), row_offset + i // 2, (i % 2) * 2)
            spin = QDoubleSpinBox()
            spin.setRange(-1, 1)
            spin.setDecimals(4)
            spin.setSingleStep(0.01)
            spin.setEnabled(False)
            layout.addWidget(spin, row_offset + i // 2, (i % 2) * 2 + 1)
            self.hinge_spins.append(spin)

        self.hinge_spins[2].setValue(1.0)

        row_offset = 6
        layout.addWidget(QLabel("Neutral (deg):"), row_offset, 0)
        self.neutral_spin = QDoubleSpinBox()
        self.neutral_spin.setRange(-90, 90)
        self.neutral_spin.setDecimals(1)
        self.neutral_spin.setSingleStep(1.0)
        self.neutral_spin.setValue(0.0)
        self.neutral_spin.setEnabled(False)
        layout.addWidget(self.neutral_spin, row_offset, 1)

        layout.addWidget(QLabel("Max Defl (deg):"), row_offset, 2)
        self.max_defl_spin = QDoubleSpinBox()
        self.max_defl_spin.setRange(1, 90)
        self.max_defl_spin.setDecimals(1)
        self.max_defl_spin.setSingleStep(1.0)
        self.max_defl_spin.setValue(30.0)
        self.max_defl_spin.setEnabled(False)
        layout.addWidget(self.max_defl_spin, row_offset, 3)

    def _on_enable(self, state):
        self._enabled = state == Qt.Checked
        self.part_combo.setEnabled(self._enabled)
        for s in self.pivot_spins:
            s.setEnabled(self._enabled)
        for s in self.hinge_spins:
            s.setEnabled(self._enabled)
        self.neutral_spin.setEnabled(self._enabled)
        self.max_defl_spin.setEnabled(self._enabled)

    def get_assignment(self) -> SurfaceAssignment | None:
        if not self._enabled:
            return None
        sa = SurfaceAssignment()
        idx = self.part_combo.currentIndex()
        if idx <= 0:
            return None
        sa.part_name = self.part_combo.currentText()
        sa.pivot_point = np.array([s.value() for s in self.pivot_spins])
        hinge = np.array([s.value() for s in self.hinge_spins])
        norm = np.linalg.norm(hinge)
        sa.hinge_axis = hinge / norm if norm > 1e-10 else np.array([0, 0, 1.0])
        sa.neutral_angle_deg = self.neutral_spin.value()
        sa.max_deflection_deg = self.max_defl_spin.value()
        sa.channel_index = self.channel_info["id"]
        return sa

    def set_from_assignment(self, sa: SurfaceAssignment):
        if sa is None:
            self.enable_cb.setChecked(False)
            return
        self.enable_cb.setChecked(True)
        idx = self.part_combo.findText(sa.part_name)
        if idx >= 0:
            self.part_combo.setCurrentIndex(idx)
        for i, v in enumerate(sa.pivot_point):
            self.pivot_spins[i].setValue(v)
        for i, v in enumerate(sa.hinge_axis):
            self.hinge_spins[i].setValue(v)
        self.neutral_spin.setValue(sa.neutral_angle_deg)
        self.max_defl_spin.setValue(sa.max_deflection_deg)


class SurfaceAssignmentDialog(QDialog):
    def __init__(self, rocket_model: RocketModel, parent=None):
        super().__init__(parent)
        self.rocket_model = rocket_model
        self.setWindowTitle("Control Surface Assignment")
        self.setMinimumSize(700, 600)
        self.setStyleSheet(
            "QDialog { background-color: #1a1a2e; color: #ddd; }"
            "QLabel { color: #ddd; }"
            "QComboBox { background-color: #16213e; border: 1px solid #333; "
            "  border-radius: 4px; padding: 4px; color: #ddd; }"
            "QComboBox QAbstractItemView { background-color: #16213e; color: #ddd; "
            "  selection-background-color: #0f3460; }"
            "QDoubleSpinBox { background-color: #16213e; border: 1px solid #333; "
            "  border-radius: 3px; padding: 2px; color: #ddd; }"
            "QPushButton { background-color: #16213e; border: 1px solid #333; "
            "  border-radius: 4px; padding: 6px 14px; color: #ddd; }"
            "QPushButton:hover { background-color: #1a2744; }"
            "QCheckBox { color: #ddd; }"
            "QCheckBox::indicator { border: 1px solid #555; border-radius: 3px; "
            "  background-color: #16213e; width: 16px; height: 16px; }"
            "QCheckBox::indicator:checked { background-color: #0f3460; }"
            "QScrollArea { border: none; }"
        )

        layout = QVBoxLayout(self)

        info = QLabel(
            f"Model: {rocket_model.model_path}\n"
            f"Parts: {', '.join(rocket_model.get_part_names())}\n\n"
            f"Assign each servo channel to a mesh part. Set the pivot point (hinge location) "
            f"and hinge axis (rotation axis) for each control surface."
        )
        info.setWordWrap(True)
        info.setStyleSheet("padding: 8px; background-color: #16213e; border-radius: 4px;")
        layout.addWidget(info)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setSpacing(8)

        part_names = rocket_model.get_part_names()
        self.surface_rows = []
        for i, ch in enumerate(SURFACE_CHANNELS):
            row = SurfaceRow(ch, SURFACE_COLORS[i], part_names)
            if i in rocket_model.surface_assignments:
                row.set_from_assignment(rocket_model.surface_assignments[i])
            self.surface_rows.append(row)
            scroll_layout.addWidget(row)

        scroll_layout.addStretch()
        scroll.setWidget(scroll_content)
        layout.addWidget(scroll)

        btn_layout = QHBoxLayout()

        btn_load_config = QPushButton("Load Config")
        btn_load_config.clicked.connect(self._load_config)
        btn_layout.addWidget(btn_load_config)

        btn_save_config = QPushButton("Save Config")
        btn_save_config.clicked.connect(self._save_config)
        btn_layout.addWidget(btn_save_config)

        btn_layout.addStretch()

        preview_cb = QCheckBox("Preview")
        preview_cb.setStyleSheet("color: #ddd;")
        self.preview_cb = preview_cb
        btn_layout.addWidget(preview_cb)

        btn_cancel = QPushButton("Cancel")
        btn_cancel.clicked.connect(self.reject)
        btn_layout.addWidget(btn_cancel)

        btn_apply = QPushButton("Apply")
        btn_apply.setDefault(True)
        btn_apply.clicked.connect(self._apply)
        btn_apply.setStyleSheet(
            "QPushButton { background-color: #0f3460; font-weight: bold; }"
            "QPushButton:hover { background-color: #1a5276; }"
        )
        btn_layout.addWidget(btn_apply)

        layout.addLayout(btn_layout)

    def _apply(self):
        for row in self.surface_rows:
            sa = row.get_assignment()
            if sa is not None:
                self.rocket_model.set_surface_assignment(row.channel_info["id"], sa)
            else:
                self.rocket_model.remove_surface_assignment(row.channel_info["id"])
        self.accept()

    def _save_config(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Surface Config", "", "JSON Files (*.json)"
        )
        if path:
            for row in self.surface_rows:
                sa = row.get_assignment()
                if sa is not None:
                    self.rocket_model.set_surface_assignment(row.channel_info["id"], sa)
                else:
                    self.rocket_model.remove_surface_assignment(row.channel_info["id"])
            self.rocket_model.save_config(path)

    def _load_config(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Load Surface Config", "", "JSON Files (*.json)"
        )
        if path:
            if self.rocket_model.load_config(path):
                for row in self.surface_rows:
                    ch = row.channel_info["id"]
                    if ch in self.rocket_model.surface_assignments:
                        row.set_from_assignment(self.rocket_model.surface_assignments[ch])
                    else:
                        row.set_from_assignment(None)
