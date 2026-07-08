import ctypes
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np

from rocket_plant import RocketPlant

FLIGHT_PHASE = {
    "TRANSPORT": 0,
    "PAD": 1,
    "READY": 2,
    "BOOST": 3,
    "COAST": 4,
    "DESCENT": 5,
}

PHASE_NAMES = {v: k for k, v in FLIGHT_PHASE.items()}


class FlightController:
    def __init__(self, lib_path: Optional[str] = None):
        if lib_path is None:
            sim_dir = Path(__file__).parent
            for candidate in [sim_dir / "libflightsim.so", sim_dir / "build" / "libflightsim.so"]:
                if candidate.exists():
                    lib_path = str(candidate)
                    break
            if lib_path is None:
                lib_path = str(sim_dir / "libflightsim.so")
        self.lib = ctypes.CDLL(lib_path)

        self.lib.sim_init.restype = None
        self.lib.sim_init.argtypes = []

        self.lib.sim_set_phase.restype = None
        self.lib.sim_set_phase.argtypes = [ctypes.c_uint8]

        self.lib.sim_get_phase.restype = ctypes.c_uint8
        self.lib.sim_get_phase.argtypes = []

        self.lib.sim_set_armed.restype = None
        self.lib.sim_set_armed.argtypes = [ctypes.c_int]

        self.lib.sim_set_mode.restype = None
        self.lib.sim_set_mode.argtypes = [ctypes.c_uint8]

        self.lib.sim_step.restype = None
        self.lib.sim_step.argtypes = [
            ctypes.c_float, ctypes.c_float, ctypes.c_float,
            ctypes.c_float, ctypes.c_float, ctypes.c_float,
            ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_uint32,
        ]

        self.lib.sim_get_servos.restype = None
        self.lib.sim_get_servos.argtypes = [
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
        ]

        self.lib.sim_get_gains.restype = None
        self.lib.sim_get_gains.argtypes = [ctypes.POINTER(ctypes.c_float)]

        self.lib.sim_get_airspeed.restype = ctypes.c_float
        self.lib.sim_get_airspeed.argtypes = []

        self.lib.sim_get_kalman_p.restype = ctypes.c_float
        self.lib.sim_get_kalman_p.argtypes = []

        self.lib.sim_get_baro_alpha.restype = ctypes.c_float
        self.lib.sim_get_baro_alpha.argtypes = []

        self.lib.sim_get_vel_z.restype = ctypes.c_float
        self.lib.sim_get_vel_z.argtypes = []

        self.lib.sim_get_filter_alt.restype = ctypes.c_float
        self.lib.sim_get_filter_alt.argtypes = []

        self.lib.sim_get_log_drops.restype = ctypes.c_uint32
        self.lib.sim_get_log_drops.argtypes = []

    def init(self):
        self.lib.sim_init()

    def set_phase(self, phase: int):
        self.lib.sim_set_phase(ctypes.c_uint8(phase))

    def get_phase(self) -> int:
        return self.lib.sim_get_phase()

    def set_armed(self, armed: bool):
        self.lib.sim_set_armed(1 if armed else 0)

    def set_mode(self, mode: int):
        self.lib.sim_set_mode(ctypes.c_uint8(mode))

    def step(self, roll: float, pitch: float, yaw: float,
             ax: float, ay: float, az: float,
             qx: float, qy: float, qz: float, qw: float,
             raw_alt: float, dt: float, elapsed_us: int):
        self.lib.sim_step(
            ctypes.c_float(roll), ctypes.c_float(pitch), ctypes.c_float(yaw),
            ctypes.c_float(ax), ctypes.c_float(ay), ctypes.c_float(az),
            ctypes.c_float(qx), ctypes.c_float(qy), ctypes.c_float(qz), ctypes.c_float(qw),
            ctypes.c_float(raw_alt),
            ctypes.c_float(dt),
            ctypes.c_uint32(elapsed_us),
        )

    def get_servos(self):
        angles = (ctypes.c_float * 8)()
        pid_outputs = (ctypes.c_float * 8)()
        self.lib.sim_get_servos(angles, pid_outputs)
        return [angles[i] for i in range(8)], [pid_outputs[i] for i in range(8)]

    def get_gains(self):
        gains = (ctypes.c_float * 3)()
        self.lib.sim_get_gains(gains)
        return [gains[i] for i in range(3)]

    def get_airspeed(self) -> float:
        return self.lib.sim_get_airspeed()

    def get_kalman_p(self) -> float:
        return self.lib.sim_get_kalman_p()

    def get_baro_alpha(self) -> float:
        return self.lib.sim_get_baro_alpha()

    def get_vel_z(self) -> float:
        return self.lib.sim_get_vel_z()

    def get_filter_alt(self) -> float:
        return self.lib.sim_get_filter_alt()

    def get_log_drops(self) -> int:
        return self.lib.sim_get_log_drops()


class CoSimulation:
    def __init__(self, scenario_path: str, lib_path: Optional[str] = None):
        with open(scenario_path, "r") as f:
            self.scenario = json.load(f)

        self.plant = RocketPlant(self.scenario)
        self.controller = FlightController(lib_path)
        self.controller.init()
        self.controller.set_phase(FLIGHT_PHASE["PAD"])
        self.controller.set_armed(True)
        self.controller.set_mode(1)

        self.dt = 0.01
        self.elapsed_us = 0
        self.time = 0.0

        self.servo_angles = [90.0] * 8
        self.servo_pid_outputs = [0.0] * 8
        self.active_gains = [0.0, 0.0, 0.0]
        self.controller_airspeed = 0.0
        self.controller_vel_z = 0.0
        self.controller_filter_alt = 0.0
        self.controller_phase = FLIGHT_PHASE["PAD"]

        self.log: list = []

        self._phase_overridden = False

    def step(self) -> dict:
        state = self.plant.step(np.array(self.servo_angles), self.dt)

        if not self._phase_overridden and self.time < 0.1:
            pass

        if not self._phase_overridden and self.time >= 0.1 and self.controller.get_phase() == FLIGHT_PHASE["PAD"]:
            self.controller.set_phase(FLIGHT_PHASE["BOOST"])
            self._phase_overridden = True

        self.controller.step(
            roll=state["roll"],
            pitch=state["pitch"],
            yaw=state["yaw"],
            ax=-state["ax"],
            ay=-state["ay"],
            az=-state["az"],
            qx=state["qx"],
            qy=state["qy"],
            qz=state["qz"],
            qw=state["qw"],
            raw_alt=state["baro_altitude"],
            dt=self.dt,
            elapsed_us=self.elapsed_us,
        )

        self.servo_angles, self.servo_pid_outputs = self.controller.get_servos()
        self.active_gains = self.controller.get_gains()
        self.controller_airspeed = self.controller.get_airspeed()
        self.controller_vel_z = self.controller.get_vel_z()
        self.controller_filter_alt = self.controller.get_filter_alt()
        self.controller_phase = self.controller.get_phase()

        entry = {
            **state,
            "servo_angles": list(self.servo_angles),
            "servo_pid_outputs": list(self.servo_pid_outputs),
            "active_gains": list(self.active_gains),
            "controller_airspeed": self.controller_airspeed,
            "controller_vel_z": self.controller_vel_z,
            "controller_filter_alt": self.controller_filter_alt,
            "controller_phase": PHASE_NAMES.get(self.controller_phase, f"UNKNOWN({self.controller_phase})"),
        }
        self.log.append(entry)

        self.elapsed_us += int(self.dt * 1e6)
        self.time += self.dt

        return entry

    def run(self, duration_s: float = 30.0, callback=None) -> list:
        steps = int(duration_s / self.dt)
        for i in range(steps):
            entry = self.step()
            if callback:
                callback(i, steps, entry)
            if self.plant.altitude < -5.0 and self.time > 3.0:
                break
        return self.log

    def reset(self):
        self.plant.reset(self.scenario)
        self.controller.init()
        self.controller.set_phase(FLIGHT_PHASE["PAD"])
        self.controller.set_armed(True)
        self.controller.set_mode(1)
        self.elapsed_us = 0
        self.time = 0.0
        self.servo_angles = [90.0] * 8
        self.servo_pid_outputs = [0.0] * 8
        self._phase_overridden = False
        self.log = []


def run_headless(scenario_path: str, duration: float = 30.0):
    sim = CoSimulation(scenario_path)
    phase_name = ""

    def progress_cb(i, total, entry):
        nonlocal phase_name
        new_phase = entry.get("controller_phase", "")
        if new_phase != phase_name:
            phase_name = new_phase
            print(f"  [{phase_name}] t={entry['time']:.2f}s alt={entry['altitude']:.1f}m")

    print(f"Running scenario: {scenario_path}")
    log = sim.run(duration_s=duration, callback=progress_cb)

    if log:
        max_alt = max(e["altitude"] for e in log)
        max_vel = max(abs(e["velocity_z"]) for e in log)
        max_g = max(e["g_load"] for e in log)
        print(f"\nResults:")
        print(f"  Max altitude:  {max_alt:.1f} m")
        print(f"  Max velocity:  {max_vel:.1f} m/s")
        print(f"  Max G-load:    {max_g:.1f} g")
        print(f"  Final Phase:   {log[-1].get('controller_phase', 'N/A')}")
        print(f"  Steps logged:  {len(log)}")

    return log


if __name__ == "__main__":
    if len(sys.argv) < 2:
        default = Path(__file__).parent / "scenarios" / "nominal.json"
        print(f"Usage: python simulate.py <scenario.json> [duration_s]")
        print(f"Using default: {default}")
        run_headless(str(default), duration=30.0)
    else:
        scenario = sys.argv[1]
        duration = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
        run_headless(scenario, duration)
