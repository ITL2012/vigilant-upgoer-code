import json
import os
import numpy as np
from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QLabel, QComboBox, QDoubleSpinBox, QPushButton, QScrollArea,
    QWidget, QFileDialog, QMessageBox, QTabWidget, QTableWidget,
    QTableWidgetItem, QHeaderView, QCheckBox, QTextEdit,
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QColor

from vehicle_extractor import (
    VehicleExtractor, VehicleProperties, MATERIAL_DENSITIES, PartProperties,
)
from model_loader import RocketModel, SURFACE_CHANNELS


class VehicleSpecDialog(QDialog):
    def __init__(self, rocket_model: RocketModel, parent=None):
        super().__init__(parent)
        self.rocket_model = rocket_model
        self.extractor = VehicleExtractor()
        self.vehicle_props: VehicleProperties | None = None

        self.setWindowTitle("Vehicle Property Extractor")
        self.setMinimumSize(800, 700)
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
            "QTabWidget::pane { border: 1px solid #333; background-color: #16213e; }"
            "QTabBar::tab { background-color: #16213e; color: #aaa; padding: 6px 12px; "
            "  border: 1px solid #333; border-bottom: none; border-radius: 4px 4px 0 0; }"
            "QTabBar::tab:selected { background-color: #0f3460; color: #fff; }"
            "QTableWidget { background-color: #16213e; color: #ddd; gridline-color: #333; "
            "  border: 1px solid #333; }"
            "QTableWidget::item { padding: 2px; }"
            "QHeaderView::section { background-color: #0f3460; color: #ddd; "
            "  border: 1px solid #333; padding: 4px; }"
            "QTextEdit { background-color: #16213e; color: #aaa; border: 1px solid #333; "
            "  border-radius: 4px; font-family: monospace; font-size: 11px; }"
        )

        self._build_ui()

    def _build_ui(self):
        layout = QVBoxLayout(self)

        info = QLabel(
            f"Model: {self.rocket_model.model_path}\n"
            f"Parts: {', '.join(self.rocket_model.get_part_names())}\n"
            f"Assign materials to each part, set motor specs, then extract vehicle properties."
        )
        info.setWordWrap(True)
        info.setStyleSheet("padding: 8px; background-color: #16213e; border-radius: 4px;")
        layout.addWidget(info)

        tabs = QTabWidget()
        layout.addWidget(tabs)

        parts_tab = QWidget()
        parts_layout = QVBoxLayout(parts_tab)
        self._build_parts_table(parts_layout)
        tabs.addTab(parts_tab, "Part Materials")

        motor_tab = QWidget()
        motor_layout = QVBoxLayout(motor_tab)
        self._build_motor_specs(motor_layout)
        tabs.addTab(motor_tab, "Motor / Propellant")

        results_tab = QWidget()
        results_layout = QVBoxLayout(results_tab)
        self._build_results_panel(results_layout)
        tabs.addTab(results_tab, "Extracted Properties")

        btn_layout = QHBoxLayout()

        btn_extract = QPushButton("Extract Properties")
        btn_extract.setStyleSheet(
            "QPushButton { background-color: #0f3460; font-weight: bold; padding: 8px 20px; }"
            "QPushButton:hover { background-color: #1a5276; }"
        )
        btn_extract.clicked.connect(self._extract)
        btn_layout.addWidget(btn_extract)

        btn_save_scenario = QPushButton("Save Scenario")
        btn_save_scenario.clicked.connect(self._save_scenario)
        btn_layout.addWidget(btn_save_scenario)

        btn_layout.addStretch()

        btn_close = QPushButton("Close")
        btn_close.clicked.connect(self.accept)
        btn_layout.addWidget(btn_close)

        layout.addLayout(btn_layout)

    def _build_parts_table(self, layout):
        part_names = self.rocket_model.get_part_names()
        surface_parts = set()
        for ch, sa in self.rocket_model.surface_assignments.items():
            if sa.part_name:
                surface_parts.add(sa.part_name)

        self.parts_table = QTableWidget(len(part_names), 5)
        self.parts_table.setHorizontalHeaderLabels(
            ["Part Name", "Type", "Material", "Shell Thickness (mm)", "Density (kg/m³)"]
        )
        self.parts_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        materials_list = sorted(MATERIAL_DENSITIES.keys())
        self.material_combos = {}
        self.thickness_spins = {}
        self.density_labels = {}

        for row, name in enumerate(part_names):
            is_surface = name in surface_parts

            name_item = QTableWidgetItem(name)
            name_item.setFlags(name_item.flags() & ~Qt.ItemIsEditable)
            if is_surface:
                name_item.setForeground(QColor(100, 200, 255))
            self.parts_table.setItem(row, 0, name_item)

            type_str = "Control Surface" if is_surface else "Body"
            type_item = QTableWidgetItem(type_str)
            type_item.setFlags(type_item.flags() & ~Qt.ItemIsEditable)
            self.parts_table.setItem(row, 1, type_item)

            mat_combo = QComboBox()
            default_mat = "carbon_fiber_composite" if is_surface else "aluminum_6061"
            for mat_name in materials_list:
                mat_combo.addItem(mat_name)
            idx = mat_combo.findText(default_mat)
            if idx >= 0:
                mat_combo.setCurrentIndex(idx)
            mat_combo.currentTextChanged.connect(lambda text, r=row: self._material_changed(r, text))
            self.parts_table.setCellWidget(row, 2, mat_combo)
            self.material_combos[name] = mat_combo

            thickness_spin = QDoubleSpinBox()
            thickness_spin.setRange(0.1, 50.0)
            thickness_spin.setDecimals(1)
            thickness_spin.setValue(2.0)
            thickness_spin.setSuffix(" mm")
            self.parts_table.setCellWidget(row, 3, thickness_spin)
            self.thickness_spins[name] = thickness_spin

            density_label = QLabel(f"{MATERIAL_DENSITIES.get(default_mat, 2700.0):.0f}")
            density_label.setAlignment(Qt.AlignCenter)
            self.parts_table.setCellWidget(row, 4, density_label)
            self.density_labels[name] = density_label

        layout.addWidget(self.parts_table)

    def _material_changed(self, row, material_name):
        density = MATERIAL_DENSITIES.get(material_name, 2700.0)
        part_name = self.rocket_model.get_part_names()[row]
        self.density_labels[part_name].setText(f"{density:.0f}")

    def _build_motor_specs(self, layout):
        grid = QGridLayout()
        grid.setSpacing(8)

        grid.addWidget(QLabel("Motor Thrust (N):"), 0, 0)
        self.thrust_spin = QDoubleSpinBox()
        self.thrust_spin.setRange(10, 50000)
        self.thrust_spin.setDecimals(0)
        self.thrust_spin.setValue(500)
        self.thrust_spin.setSingleStep(50)
        grid.addWidget(self.thrust_spin, 0, 1)

        grid.addWidget(QLabel("Burn Time (s):"), 1, 0)
        self.burn_time_spin = QDoubleSpinBox()
        self.burn_time_spin.setRange(0.1, 60.0)
        self.burn_time_spin.setDecimals(1)
        self.burn_time_spin.setValue(2.5)
        self.burn_time_spin.setSingleStep(0.1)
        grid.addWidget(self.burn_time_spin, 1, 1)

        grid.addWidget(QLabel("Propellant Mass (kg) [auto if 0]:"), 2, 0)
        self.propellant_spin = QDoubleSpinBox()
        self.propellant_spin.setRange(0, 100)
        self.propellant_spin.setDecimals(2)
        self.propellant_spin.setValue(0)
        self.propellant_spin.setSingleStep(0.1)
        grid.addWidget(self.propellant_spin, 2, 1)

        grid.addWidget(QLabel("Propellant Density (kg/m³):"), 3, 0)
        self.propellant_density_spin = QDoubleSpinBox()
        self.propellant_density_spin.setRange(100, 5000)
        self.propellant_density_spin.setDecimals(0)
        self.propellant_density_spin.setValue(1750)
        self.propellant_density_spin.setSingleStep(50)
        grid.addWidget(self.propellant_density_spin, 3, 1)

        grid.addWidget(QLabel("Shell thickness default (mm):"), 4, 0)
        self.shell_default_spin = QDoubleSpinBox()
        self.shell_default_spin.setRange(0.1, 50.0)
        self.shell_default_spin.setDecimals(1)
        self.shell_default_spin.setValue(2.0)
        grid.addWidget(self.shell_default_spin, 4, 1)

        layout.addLayout(grid)

        note = QLabel(
            "Set propellant mass to 0 to auto-estimate from motor volume.\n"
            "Thrust profile will be generated as: ramp-up → sustained → tail-off."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color: #888; padding: 8px; background-color: #16213e; border-radius: 4px;")
        layout.addWidget(note)

    def _build_results_panel(self, layout):
        self.results_text = QTextEdit()
        self.results_text.setReadOnly(True)
        self.results_text.setPlaceholderText("Click 'Extract Properties' to compute vehicle data from CAD model...")
        layout.addWidget(self.results_text)

    def _extract(self):
        self.extractor.default_shell_thickness = self.shell_default_spin.value() / 1000.0
        self.extractor.motor_thrust_n = self.thrust_spin.value()
        self.extractor.motor_burn_time_s = self.burn_time_spin.value()
        self.extractor.propellant_density = self.propellant_density_spin.value()

        part_materials = {}
        for name, combo in self.material_combos.items():
            part_materials[name] = combo.currentText()

        self.vehicle_props = self.extractor.extract(
            self.rocket_model.all_meshes,
            surface_assignments=self.rocket_model.surface_assignments,
            part_materials=part_materials,
        )

        if self.propellant_spin.value() > 0:
            self.vehicle_props.propellant_mass_kg = self.propellant_spin.value()
            self.vehicle_props.total_mass_kg = self.vehicle_props.dry_mass_kg + self.vehicle_props.propellant_mass_kg

        vp = self.vehicle_props
        lines = []
        lines.append("===== VEHICLE PROPERTIES =====\n")
        lines.append(f"Total Mass:      {vp.total_mass_kg:.3f} kg")
        lines.append(f"  Dry Mass:      {vp.dry_mass_kg:.3f} kg")
        lines.append(f"  Propellant:    {vp.propellant_mass_kg:.3f} kg")
        lines.append(f"CG Position:     ({vp.cg_position[0]:.4f}, {vp.cg_position[1]:.4f}, {vp.cg_position[2]:.4f}) m")
        lines.append(f"MOI (Ixx,Iyy,Izz): ({vp.moi_body[0]:.6f}, {vp.moi_body[1]:.6f}, {vp.moi_body[2]:.6f}) kg·m²")
        lines.append(f"")
        lines.append(f"=== DIMENSIONS ===")
        lines.append(f"Ref Diameter:    {vp.ref_diameter_m:.4f} m")
        lines.append(f"Total Length:    {vp.total_length_m:.4f} m")
        lines.append(f"Ref Area:        {vp.ref_area_m2:.6f} m²")
        lines.append(f"CP-CG offset:    {vp.x_cp_minus_cg_m:.4f} m ({'stable' if vp.x_cp_minus_cg_m > 0 else 'UNSTABLE!'})")
        lines.append(f"")
        lines.append(f"=== AERO COEFFICIENTS ===")
        lines.append(f"Cd0:             {vp.Cd0:.3f}")
        lines.append(f"Cl_alpha:        {vp.Cl_alpha:.3f}")
        lines.append(f"Cl_delta:        {vp.Cl_delta:.3f}")
        lines.append(f"Cm_alpha:        {vp.Cm_alpha:.3f}")
        lines.append(f"Cn_delta:        {vp.Cn_delta:.3f}")
        lines.append(f"")
        lines.append(f"=== PART BREAKDOWN ===")

        for name, pp in vp.part_properties.items():
            lines.append(f"  {name}:")
            lines.append(f"    Material:     {pp.material} ({MATERIAL_DENSITIES.get(pp.material, 0):.0f} kg/m³)")
            lines.append(f"    Volume:       {pp.volume_m3 * 1e6:.2f} cm³ ({'shell' if pp.is_shell else 'solid'})")
            lines.append(f"    Area:         {pp.surface_area_m2 * 1e4:.2f} cm²")
            lines.append(f"    Mass:         {pp.mass_kg * 1000:.1f} g")
            lines.append(f"    Centroid:     ({pp.centroid[0]:.4f}, {pp.centroid[1]:.4f}, {pp.centroid[2]:.4f})")
            if pp.is_surface:
                lines.append(f"    Channel:      Ch{pp.surface_channel}")
                lines.append(f"    Planform:     {pp.planform_area_m2 * 1e4:.2f} cm²")
                lines.append(f"    Span:         {pp.span_m * 1000:.1f} mm")
                lines.append(f"    Chord:        {pp.chord_m * 1000:.1f} mm")
                lines.append(f"    AR:           {pp.aspect_ratio:.2f}")

        thrust = self.thrust_spin.value()
        burn = self.burn_time_spin.value()
        scenario_dict = vp.to_scenario_dict(thrust, burn)
        lines.append(f"")
        lines.append(f"=== GENERATED SCENARIO JSON ===")
        lines.append(json.dumps(scenario_dict, indent=2))

        self.results_text.setText("\n".join(lines))

    def _save_scenario(self):
        if self.vehicle_props is None:
            self._extract()

        if self.vehicle_props is None:
            QMessageBox.warning(self, "Error", "Failed to extract vehicle properties")
            return

        default_name = os.path.splitext(os.path.basename(self.rocket_model.model_path))[0]
        path, _ = QFileDialog.getSaveFileName(
            self, "Save Scenario", f"scenarios/{default_name}_extracted.json",
            "JSON Files (*.json)"
        )
        if path:
            thrust = self.thrust_spin.value()
            burn = self.burn_time_spin.value()
            scenario_dict = self.vehicle_props.to_scenario_dict(thrust, burn)
            with open(path, "w") as f:
                json.dump(scenario_dict, f, indent=2)
            QMessageBox.information(self, "Saved", f"Scenario saved to:\n{path}")
