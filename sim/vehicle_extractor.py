import math
import numpy as np
import trimesh
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field


MATERIAL_DENSITIES = {
    "aluminum_6061": 2700.0,
    "aluminum_7075": 2810.0,
    "carbon_fiber_composite": 1600.0,
    "fiberglass_composite": 1850.0,
    "pcb_fr4": 1850.0,
    "abs_plastic": 1040.0,
    "pla_plastic": 1240.0,
    "petg_plastic": 1270.0,
    "nylon_pa12": 1010.0,
    "steel_stainless": 7850.0,
    "propellant_ammonium_perchlorate": 1750.0,
}

AERO_DEFAULTS = {
    "Cd0_minimum_drag": 0.35,
    "Cd0_blunt_body": 0.50,
    "Cl_alpha_thin_fin": 3.5,
    "Cl_alpha_low_ar": 2.0,
    "Cl_delta_effective": 2.5,
    "Cn_delta_canard": 2.0,
    "Cm_alpha_stable": -2.5,
}


@dataclass
class PartProperties:
    name: str = ""
    volume_m3: float = 0.0
    surface_area_m2: float = 0.0
    mass_kg: float = 0.0
    centroid: np.ndarray = field(default_factory=lambda: np.zeros(3))
    bounding_box: np.ndarray = field(default_factory=lambda: np.zeros((2, 3)))
    is_shell: bool = False
    wall_thickness_m: float = 0.002
    material: str = "aluminum_6061"
    is_surface: bool = False
    surface_channel: int = -1
    planform_area_m2: float = 0.0
    span_m: float = 0.0
    chord_m: float = 0.0
    aspect_ratio: float = 0.0
    hinge_x: float = 0.0


@dataclass
class VehicleProperties:
    total_mass_kg: float = 0.0
    cg_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    moi_body: np.ndarray = field(default_factory=lambda: np.zeros(3))
    ref_diameter_m: float = 0.0
    ref_length_m: float = 0.0
    ref_area_m2: float = 0.0
    total_length_m: float = 0.0
    x_cp_minus_cg_m: float = 0.0
    Cd0: float = 0.45
    Cl_alpha: float = 2.5
    Cl_delta: float = 1.8
    Cm_alpha: float = -2.5
    Cn_delta: float = 1.5
    propellant_mass_kg: float = 0.0
    dry_mass_kg: float = 0.0
    part_properties: Dict[str, PartProperties] = field(default_factory=dict)

    def to_scenario_dict(self, motor_thrust_n: float = 500.0,
                         burn_time_s: float = 2.5,
                         thrust_profile: list = None) -> dict:
        if thrust_profile is None:
            thrust_profile = [
                [0, 0], [0.1, motor_thrust_n],
                [burn_time_s - 0.3, motor_thrust_n],
                [burn_time_s - 0.1, motor_thrust_n * 0.2],
                [burn_time_s, 0],
            ]

        return {
            "name": "Auto-generated from CAD",
            "description": "Vehicle properties extracted from 3D model",
            "mass_dry_kg": round(self.dry_mass_kg, 3),
            "mass_propellant_kg": round(self.propellant_mass_kg, 3),
            "cg_offset_m": round(self.cg_position[2], 4),
            "moi": {
                "Ixx": round(self.moi_body[0], 6),
                "Iyy": round(self.moi_body[1], 6),
                "Izz": round(self.moi_body[2], 6),
            },
            "aero": {
                "Cd0": round(self.Cd0, 3),
                "Cl_alpha": round(self.Cl_alpha, 3),
                "Cl_delta": round(self.Cl_delta, 3),
                "Cm_alpha": round(self.Cm_alpha, 3),
                "Cn_delta": round(self.Cn_delta, 3),
                "ref_area_m2": round(self.ref_area_m2, 6),
                "ref_length_m": round(self.ref_length_m, 4),
                "x_cp_minus_cg_m": round(self.x_cp_minus_cg_m, 4),
            },
            "motor": {
                "burn_time_s": burn_time_s,
                "thrust_profile_n": thrust_profile,
            },
            "baro": {"noise_std_m": 0.5, "bias_m": 0},
            "accel_noise_std": 0.05,
            "gravity_m_s2": 9.80665,
            "air_density_kg_m3": 1.225,
            "scale_height_m": 8500,
            "wind_schedule": [],
        }


class VehicleExtractor:
    def __init__(self):
        self.default_shell_thickness = 0.002
        self.default_body_material = "aluminum_6061"
        self.default_fin_material = "carbon_fiber_composite"
        self.propellant_density = MATERIAL_DENSITIES["propellant_ammonium_perchlorate"]
        self.motor_thrust_n = 500.0
        self.motor_burn_time_s = 2.5

    def extract(self, scene_or_meshes: Dict[str, trimesh.Trimesh],
                surface_assignments: Dict[int, 'SurfaceAssignment'] = None,
                part_materials: Dict[str, str] = None) -> VehicleProperties:
        meshes = scene_or_meshes
        if isinstance(scene_or_meshes, trimesh.Scene):
            meshes = {}
            for name, geom in scene_or_meshes.geometry.items():
                if isinstance(geom, trimesh.Trimesh):
                    meshes[name] = geom

        if part_materials is None:
            part_materials = {}

        surface_parts = set()
        if surface_assignments:
            for ch, sa in surface_assignments.items():
                if sa.part_name:
                    surface_parts.add(sa.part_name)

        part_props = {}
        all_centroids = []
        all_masses = []
        body_verts = []

        for name, mesh in meshes.items():
            pp = PartProperties()
            pp.name = name
            pp.bounding_box = np.array([mesh.bounds[0], mesh.bounds[1]])

            is_surface_part = name in surface_parts
            pp.is_surface = is_surface_part

            material = part_materials.get(name,
                                          self.default_fin_material if is_surface_part
                                          else self.default_body_material)
            pp.material = material
            density = MATERIAL_DENSITIES.get(material, 2700.0)

            try:
                if mesh.is_watertight:
                    pp.volume_m3 = mesh.volume
                    pp.is_shell = False
                else:
                    pp.is_shell = True
                    area = mesh.area
                    thickness = self.default_shell_thickness
                    if is_surface_part:
                        thickness = 0.001
                    pp.volume_m3 = area * thickness
                    pp.wall_thickness_m = thickness
            except Exception:
                pp.is_shell = True
                pp.volume_m3 = mesh.area * self.default_shell_thickness

            pp.surface_area_m2 = mesh.area
            pp.centroid = mesh.centroid.copy()

            pp.mass_kg = pp.volume_m3 * density
            all_centroids.append(pp.centroid)
            all_masses.append(pp.mass_kg)

            if not is_surface_part:
                try:
                    body_verts.append(mesh.vertices)
                except Exception:
                    pass

            if is_surface_part:
                self._compute_fin_properties(pp, mesh, surface_assignments)

            part_props[name] = pp

        if not all_masses:
            return VehicleProperties()

        total_mass = sum(all_masses)
        cg = np.zeros(3)
        for pp in all_masses, all_centroids:
            pass

        cg = np.zeros(3)
        for i, (mass, centroid) in enumerate(
            zip([pp.mass_kg for pp in part_props.values()],
                [pp.centroid for pp in part_props.values()])):
            cg += mass * centroid
        if total_mass > 0:
            cg /= total_mass

        moi = self._compute_moi(part_props, cg)

        ref_d, total_len, x_cp_cg = self._compute_body_dimensions(
            part_props, surface_parts, cg
        )

        aero = self._estimate_aero_coefficients(part_props, surface_parts,
                                                  ref_d, total_len, x_cp_cg)

        dry_mass = total_mass
        propellant_mass = self._estimate_propellant(ref_d, total_len)

        props = VehicleProperties(
            total_mass_kg=total_mass + propellant_mass,
            cg_position=cg,
            moi_body=moi,
            ref_diameter_m=ref_d,
            ref_length_m=ref_d,
            ref_area_m2=math.pi * (ref_d / 2) ** 2,
            total_length_m=total_len,
            x_cp_minus_cg_m=x_cp_cg,
            Cd0=aero["Cd0"],
            Cl_alpha=aero["Cl_alpha"],
            Cl_delta=aero["Cl_delta"],
            Cm_alpha=aero["Cm_alpha"],
            Cn_delta=aero["Cn_delta"],
            propellant_mass_kg=propellant_mass,
            dry_mass_kg=dry_mass,
            part_properties=part_props,
        )

        return props

    def _compute_fin_properties(self, pp: PartProperties, mesh: trimesh.Trimesh,
                                 surface_assignments: dict):
        bb_min = mesh.bounds[0]
        bb_max = mesh.bounds[1]
        extents = bb_max - bb_min

        if pp.is_surface and pp.surface_channel >= 4:
            span_idx = np.argmax(extents[:2])
        else:
            span_idx = np.argmax(extents[:2])

        pp.chord_m = extents[2] if span_idx != 2 else extents[np.argmax(
            [extents[i] for i in range(3) if i != span_idx]
        )]
        pp.span_m = extents[span_idx] if span_idx < 3 else extents[0]

        if pp.span_m > 0 and pp.chord_m > 0:
            pp.planform_area_m2 = pp.span_m * pp.chord_m
            pp.aspect_ratio = pp.span_m ** 2 / max(pp.planform_area_m2, 1e-10)

        sa = surface_assignments.get(pp.surface_channel)
        if sa is not None:
            pp.hinge_x = sa.pivot_point[0]

    def _compute_moi(self, part_props: Dict[str, PartProperties],
                      cg: np.ndarray) -> np.ndarray:
        Ixx = Iyy = Izz = 0.0

        for pp in part_props.values():
            r = pp.centroid - cg
            d_sq = np.sum(r ** 2)

            bb = pp.bounding_box
            extents = bb[1] - bb[0]
            lx, ly, lz = extents

            I_local_x = pp.mass_kg * (ly ** 2 + lz ** 2) / 12.0
            I_local_y = pp.mass_kg * (lx ** 2 + lz ** 2) / 12.0
            I_local_z = pp.mass_kg * (lx ** 2 + ly ** 2) / 12.0

            Ixx += I_local_x + pp.mass_kg * (r[1] ** 2 + r[2] ** 2)
            Iyy += I_local_y + pp.mass_kg * (r[0] ** 2 + r[2] ** 2)
            Izz += I_local_z + pp.mass_kg * (r[0] ** 2 + r[1] ** 2)

        return np.array([Ixx, Iyy, Izz])

    def _compute_body_dimensions(self, part_props: Dict[str, PartProperties],
                                  surface_parts: set, cg: np.ndarray) -> Tuple[float, float, float]:
        body_verts_list = []
        for name, pp in part_props.items():
            if name not in surface_parts:
                body_verts_list.append(pp.bounding_box)

        if not body_verts_list:
            all_bounds = [pp.bounding_box for pp in part_props.values()]
        else:
            all_bounds = body_verts_list

        if not all_bounds:
            return 0.08, 1.0, 0.15

        all_mins = np.min([b[0] for b in all_bounds], axis=0)
        all_maxs = np.max([b[1] for b in all_bounds], axis=0)
        extents = all_maxs - all_mins

        total_length = extents[2] if extents[2] == max(extents) else max(extents)
        ref_diameter = min(extents[0], extents[1])
        if ref_diameter < 0.01:
            ref_diameter = min(extents) if min(extents) > 0.01 else 0.08

        body_centroid_z = (all_mins[2] + all_maxs[2]) / 2.0
        cp_z = all_mins[2] + total_length * 0.65
        x_cp_cg = cp_z - cg[2] if total_length > 0 else 0.15

        return ref_diameter, total_length, x_cp_cg

    def _estimate_aero_coefficients(self, part_props: Dict[str, PartProperties],
                                     surface_parts: set,
                                     ref_d: float, total_len: float,
                                     x_cp_cg: float) -> dict:
        fin_data = []
        canard_data = []

        for name, pp in part_props.items():
            if name in surface_parts:
                ch = pp.surface_channel
                if ch < 4:
                    canard_data.append(pp)
                else:
                    fin_data.append(pp)

        total_fin_area = sum(p.planform_area_m2 for p in fin_data) if fin_data else 0.01
        total_canard_area = sum(p.planform_area_m2 for p in canard_data) if canard_data else 0.005
        ref_area = math.pi * (ref_d / 2) ** 2

        fineness = total_len / max(ref_d, 0.01)
        if fineness > 10:
            Cd0 = 0.30
        elif fineness > 5:
            Cd0 = 0.40
        else:
            Cd0 = 0.50

        avg_fin_ar = 0.0
        n_fins = 0
        for p in fin_data + canard_data:
            if p.aspect_ratio > 0:
                avg_fin_ar += p.aspect_ratio
                n_fins += 1
        avg_fin_ar = avg_fin_ar / max(n_fins, 1)

        Cl_alpha = 2.0 * math.pi * avg_fin_ar / (2 + avg_fin_ar) if avg_fin_ar > 0 else 2.5
        Cl_alpha = max(1.5, min(Cl_alpha, 5.0))

        S_canard_ratio = total_canard_area / max(ref_area, 1e-10)
        Cl_delta = 0.8 * Cl_alpha * S_canard_ratio
        Cl_delta = max(1.0, min(Cl_delta, 4.0))

        Cn_delta = Cl_delta * 1.0

        if x_cp_cg > 0:
            Cm_alpha = -Cl_alpha * x_cp_cg / max(ref_d, 0.01) * 0.5
        else:
            Cm_alpha = -2.5

        return {
            "Cd0": round(Cd0, 3),
            "Cl_alpha": round(Cl_alpha, 3),
            "Cl_delta": round(Cl_delta, 3),
            "Cm_alpha": round(Cm_alpha, 3),
            "Cn_delta": round(Cn_delta, 3),
        }

    def _estimate_propellant(self, ref_d: float, total_len: float) -> float:
        motor_length_frac = 0.4
        motor_outer_r = ref_d / 2.0
        motor_inner_r = motor_outer_r * 0.7
        motor_length = total_len * motor_length_frac

        grain_volume = math.pi * (motor_outer_r ** 2 - motor_inner_r ** 2) * motor_length
        propellant_mass = grain_volume * self.propellant_density

        return max(propellant_mass, 0.1)
