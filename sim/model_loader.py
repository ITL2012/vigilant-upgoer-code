import os
import json
import math
import numpy as np
from pathlib import Path
from typing import Optional, Dict, List, Tuple

import trimesh


SURFACE_CHANNELS = [
    {"id": 0, "label": "C0-P+", "axis": "pitch", "sign": +1.0},
    {"id": 1, "label": "C1-P-", "axis": "pitch", "sign": -1.0},
    {"id": 2, "label": "C2-Y+", "axis": "yaw",   "sign": +1.0},
    {"id": 3, "label": "C3-Y-", "axis": "yaw",   "sign": -1.0},
    {"id": 4, "label": "F4-R+", "axis": "roll",  "sign": +1.0},
    {"id": 5, "label": "F5-R-", "axis": "roll",  "sign": -1.0},
    {"id": 6, "label": "F6-R+", "axis": "roll",  "sign": +1.0},
    {"id": 7, "label": "F7-R-", "axis": "roll",  "sign": -1.0},
]

SURFACE_COLORS = [
    (0.94, 0.33, 0.31),
    (0.96, 0.26, 0.21),
    (0.67, 0.28, 0.74),
    (0.61, 0.15, 0.69),
    (0.13, 0.59, 0.95),
    (0.12, 0.53, 0.90),
    (0.00, 0.74, 0.83),
    (0.00, 0.59, 0.53),
]


def _convert_step_to_obj(step_path: str, output_dir: str) -> Optional[str]:
    out_stem = Path(step_path).stem + ".obj"
    out_path = os.path.join(output_dir, out_stem)
    if os.path.exists(out_path):
        return out_path
    try:
        import subprocess
        result = subprocess.run(
            ["FreeCADCmd", "-c",
             f"import FreeCAD, Part, Mesh; s=Part.read('{step_path}'); "
             f"m=Mesh.Mesh(s.Shape.tessellate(0.1)); m.write('{out_path}')"],
            capture_output=True, text=True, timeout=60,
        )
        if os.path.exists(out_path):
            return out_path
    except Exception:
        pass
    return None


class SurfaceAssignment:
    def __init__(self):
        self.part_name: str = ""
        self.pivot_point: np.ndarray = np.zeros(3)
        self.hinge_axis: np.ndarray = np.array([0.0, 0.0, 1.0])
        self.neutral_angle_deg: float = 0.0
        self.max_deflection_deg: float = 30.0
        self.channel_index: int = -1

    def to_dict(self) -> dict:
        return {
            "part_name": self.part_name,
            "pivot_point": self.pivot_point.tolist(),
            "hinge_axis": self.hinge_axis.tolist(),
            "neutral_angle_deg": self.neutral_angle_deg,
            "max_deflection_deg": self.max_deflection_deg,
            "channel_index": self.channel_index,
        }

    @staticmethod
    def from_dict(d: dict) -> "SurfaceAssignment":
        sa = SurfaceAssignment()
        sa.part_name = d.get("part_name", "")
        sa.pivot_point = np.array(d.get("pivot_point", [0, 0, 0]))
        sa.hinge_axis = np.array(d.get("hinge_axis", [0, 0, 1]))
        sa.neutral_angle_deg = d.get("neutral_angle_deg", 0.0)
        sa.max_deflection_deg = d.get("max_deflection_deg", 30.0)
        sa.channel_index = d.get("channel_index", -1)
        return sa


class RocketModel:
    def __init__(self):
        self.scene: Optional[trimesh.Scene] = None
        self.body_mesh: Optional[trimesh.Trimesh] = None
        self.all_meshes: Dict[str, trimesh.Trimesh] = {}
        self.surface_assignments: Dict[int, SurfaceAssignment] = {}
        self.mesh_paths: List[str] = []
        self.model_center: np.ndarray = np.zeros(3)
        self.model_scale: float = 1.0
        self.model_path: str = ""
        self.valid: bool = False

    def load_model(self, file_path: str) -> bool:
        self.model_path = file_path
        self.valid = False
        self.all_meshes = {}
        self.mesh_paths = []

        ext = Path(file_path).suffix.lower()

        if ext == ".step" or ext == ".stp":
            obj_path = _convert_step_to_obj(file_path, os.path.dirname(file_path))
            if obj_path is None:
                return False
            file_path = obj_path
            ext = ".obj"

        try:
            if ext in (".obj", ".stl", ".ply", ".glb", ".gltf"):
                loaded = trimesh.load(file_path, force="scene")
            else:
                loaded = trimesh.load(file_path)

            if isinstance(loaded, trimesh.Scene):
                self.scene = loaded
                for name, geom in loaded.geometry.items():
                    if isinstance(geom, trimesh.Trimesh):
                        self.all_meshes[name] = geom
                        self.mesh_paths.append(name)
                if self.all_meshes:
                    first = list(self.all_meshes.values())[0]
                    self.body_mesh = first
            elif isinstance(loaded, trimesh.Trimesh):
                self.all_meshes["body"] = loaded
                self.mesh_paths.append("body")
                self.scene = trimesh.Scene({"body": loaded})
                self.body_mesh = loaded
            else:
                return False

            all_verts = []
            for m in self.all_meshes.values():
                all_verts.append(m.vertices)
            if all_verts:
                combined = np.vstack(all_verts)
                self.model_center = combined.mean(axis=0)
                extent = combined.max(axis=0) - combined.min(axis=0)
                max_dim = extent.max()
                if max_dim > 0:
                    self.model_scale = 1.0 / max_dim

            self.valid = True
            return True

        except Exception as e:
            print(f"Model load error: {e}")
            return False

    def get_part_names(self) -> List[str]:
        return list(self.mesh_paths)

    def get_triangulated(self, part_name: str) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        if part_name not in self.all_meshes:
            return None
        mesh = self.all_meshes[part_name]
        verts = mesh.vertices.copy()
        faces = mesh.faces.copy()
        return verts, faces

    def set_surface_assignment(self, channel: int, assignment: SurfaceAssignment):
        self.surface_assignments[channel] = assignment

    def remove_surface_assignment(self, channel: int):
        self.surface_assignments.pop(channel, None)

    def get_body_parts(self) -> Dict[str, trimesh.Trimesh]:
        assigned = set()
        for sa in self.surface_assignments.values():
            if sa.part_name:
                assigned.add(sa.part_name)
        return {k: v for k, v in self.all_meshes.items() if k not in assigned}

    def compute_deflection_transform(self, channel: int, deflection_deg: float) -> np.ndarray:
        if channel not in self.surface_assignments:
            return np.eye(4)

        sa = self.surface_assignments[channel]
        pivot = sa.pivot_point
        axis = sa.hinge_axis / max(np.linalg.norm(sa.hinge_axis), 1e-10)

        total_angle = math.radians(sa.neutral_angle_deg + deflection_deg)

        T_to_origin = np.eye(4)
        T_to_origin[:3, 3] = -pivot

        K = np.array([
            [0, -axis[2], axis[1]],
            [axis[2], 0, -axis[0]],
            [-axis[1], axis[0], 0],
        ])
        R = np.eye(3) + math.sin(total_angle) * K + (1 - math.cos(total_angle)) * (K @ K)

        T_rotate = np.eye(4)
        T_rotate[:3, :3] = R

        T_back = np.eye(4)
        T_back[:3, 3] = pivot

        return T_back @ T_rotate @ T_to_origin

    def save_config(self, path: str):
        config = {
            "model_path": self.model_path,
            "surface_assignments": {
                str(k): v.to_dict() for k, v in self.surface_assignments.items()
            },
        }
        with open(path, "w") as f:
            json.dump(config, f, indent=2)

    def load_config(self, path: str) -> bool:
        try:
            with open(path, "r") as f:
                config = json.load(f)
            model_path = config.get("model_path", "")
            if model_path and os.path.exists(model_path):
                if not self.load_model(model_path):
                    return False
            for k, v in config.get("surface_assignments", {}).items():
                self.surface_assignments[int(k)] = SurfaceAssignment.from_dict(v)
            return True
        except Exception:
            return False
