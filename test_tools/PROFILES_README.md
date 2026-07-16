# Flight Profile JSON + 3D Visualizer

## JSON Profile Format

Profiles are stored as JSON and loaded from SD (`/profiles/*.json`) at boot, or
uploaded at runtime via the web UI / HTTP API. The firmware parses them with
ArduinoJson into the same `FlightProfile` structure used by the flash-builtin
profiles (`MATH_TEST`, `ROLL_CAL`, `RECOVERY`).

```json
{
  "name": "MY_PROFILE",
  "loop": false,
  "steps": [
    {"roll": 180, "pitch": 0, "yaw": 0, "duration": 5.0,
     "trigger": "time", "value": 0, "rate": 40},
    {"roll": 0, "pitch": 0, "yaw": 0, "trigger": "apogee", "rate": 25}
  ]
}
```

### Step fields

| Field     | Type   | Meaning                                                    |
|-----------|--------|------------------------------------------------------------|
| `roll`    | float  | Target roll in degrees                                    |
| `pitch`   | float  | Target pitch in degrees                                   |
| `yaw`     | float  | Target yaw in degrees                                     |
| `duration`| float  | Step length in seconds (used by `time` trigger)           |
| `trigger` | string | `time` \| `apogee` \| `altitude` \| `velocity` \| `manual`|
| `value`   | float  | Altitude (m) or velocity (m/s) for the trigger            |
| `rate`    | float  | Max slew rate deg/s for this step (0 = unlimited)         |

Setpoints are generated with a quintic S-curve between the previous and target
attitude, so servo motion is smooth (zero velocity/acceleration at step ends).

## Loading from SD

At boot (after SD init) the firmware scans `/profiles/` and loads every `.json`
file. The CLI command `profile reload` re-scans at runtime.

```
profile list      -> list flash + SD profiles
profile start X   -> start by name (checks flash then SD)
profile reload    -> re-scan SD
profile stop      -> abort
profile status    -> active step + setpoints
```

## HTTP API

| Route            | Method | Args                | Purpose                       |
|------------------|--------|---------------------|-------------------------------|
| `/profile_list`  | GET    | —                   | list all (flash + SD)        |
| `/profile_start` | POST   | `name` or `json`    | start by name or inline JSON  |
| `/profile_stop`  | POST   | —                   | abort active profile         |
| `/profile_status`| GET    | —                   | active setpoints + progress   |
| `/profile_upload`| POST   | `name`, `json`      | save JSON to SD + register   |
| `/profile_delete`| POST   | `name`              | delete SD profile file       |

## 3D Visualizer (`test_tools/profile_visualizer.py`)

A PyQt5 + matplotlib tool to view, edit, and export profiles as a 3D attitude
trajectory (the rocket's nose vector over time).

```bash
# Interactive editor
python3 test_tools/profile_visualizer.py
python3 test_tools/profile_visualizer.py test_tools/sample_profiles/math_test.json

# Headless upload to a running controller (no GUI needed)
python3 test_tools/profile_visualizer.py test_tools/sample_profiles/math_test.json \
    --upload http://192.168.4.1 --name MATH_TEST
```

Features:
- Drag-step editing of roll/pitch/yaw/duration/trigger/rate
- Animated 3D nose-vector path with rocket glyphs at each step boundary
- Save / open JSON, "Export to SD" (copy into mounted SD `/profiles/`)
- One-click "Upload to Controller" (POSTs to `/profile_upload`)
