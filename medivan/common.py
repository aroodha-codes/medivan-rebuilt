"""Units: metres, radians, seconds. Coordinates: x forward, y left, CCW yaw."""

from dataclasses import dataclass, field, asdict
from pathlib import Path
import json, math, os, tempfile

ROOT = Path(__file__).resolve().parents[1]


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=".save-")
    try:
        with os.fdopen(fd, "w") as f:
            json.dump(value, f, allow_nan=False)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


def load_config(path=None):
    c = json.loads(Path(path or ROOT / "config/robot.json").read_text())
    g, m, n = c["geometry"], c["motor"], c["navigation"]
    for k in [
        "wheel_diameter_m",
        "track_width_m",
        "camera_height_m",
        "robot_width_m",
        "robot_length_m",
    ]:
        if not math.isfinite(g[k]) or not 0 < g[k] < 2:
            raise ValueError(f"Invalid geometry: {k}")
    pins = m["left"] + m["right"]
    if len(set(pins)) != 6 or any(
        type(p) is not int or p not in range(2, 28) for p in pins
    ):
        raise ValueError("Six distinct BCM motor pins required")
    if not 0 < n["speed_m_s"] <= 0.2:
        raise ValueError("Invalid navigation speed")
    if not 0 < n["resolution_m"] < 1 or not 2 <= n["map_size_m"] <= 30:
        raise ValueError("Invalid map dimensions")
    if not 0 < m["watchdog_s"] <= 1:
        raise ValueError("Invalid watchdog")

    def bounded(section, key, lo, hi):
        value = c[section][key]
        if (
            isinstance(value, bool)
            or not isinstance(value, (float, int))
            or not math.isfinite(value)
            or not lo <= value <= hi
        ):
            raise ValueError(f"Invalid {section}.{key}")

    for side in ["left", "right"]:
        bounded("motor", "full_speed_" + side + "_m_s", 0.02, 1.0)
        if m[side + "_sign"] not in [-1, 1]:
            raise ValueError("Motor sign must be ±1")
    bounded("motor", "deadband", 0, 0.8)
    bounded("motor", "pwm_hz", 10, 2000)
    bounded("navigation", "physical_max_travel_m", 0.1, 100)
    bounded("navigation", "clearance_m", 0.02, 0.5)
    bounded("navigation", "angular_speed_rad_s", 0.1, 0.5)
    bounded("navigation", "stop_distance_m", 0.15, 1.0)
    bounded("geometry", "camera_forward_m", 0, 1.0)
    bounded("geometry", "camera_pitch_deg", -10, 60)
    bounded("camera", "stale_s", 0.1, 1.0)
    bounded("camera", "min_confidence", 0.2, 1.0)
    bounded("camera", "floor_tolerance", 5, 60)
    bounded("camera", "max_range_m", 0.3, 5.0)
    bounded("dock", "reverse_speed_m_s", 0.005, 0.04)
    bounded("dock", "reverse_distance_m", 0.02, 0.5)
    bounded("dock", "reverse_timeout_s", 1, 30)
    bounded("dock", "staging_distance_m", 0.2, 1.5)
    bounded("dock", "charge_confirm_seconds", 1, 10)
    bounded("dock", "current_threshold_a", -2, -0.02)
    bounded("sensors", "divider_ratio", 1, 20)
    bounded("sensors", "voltage_correction", 0.5, 2)
    bounded("sensors", "shunt_ohm", 0.01, 1.0)
    if c["sensors"]["current_sign"] not in [-1, 1]:
        raise ValueError("Current sign must be ±1")
    if (
        not 5.8
        <= c["battery"]["critical_voltage"]
        < c["battery"]["low_voltage"]
        < c["battery"]["full_voltage"]
        <= 8.5
    ):
        raise ValueError("Invalid 2S battery thresholds")
    if c["vision"]["every_n_frames"] < 1:
        raise ValueError("YOLO interval must be positive")
    return c


@dataclass
class Observation:
    stamp: float
    yaw_rate: float = 0.0
    tilt_deg: float = 0.0
    voltage: float | None = None
    current: float | None = None
    rays: list = field(default_factory=list)  # angle, metres, hit
    confidence: float = 1.0
    frame_stamp: float = 0.0
    battery_stamp: float = 0.0
    marker: dict | None = None
    detections: list = field(default_factory=list)
    faults: list = field(default_factory=list)


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    travel: float = 0.0
    uncertainty_m: float = 0.02

    def advance(self, v, w, dt, hardware=False):
        mid = self.yaw + w * dt / 2
        self.x += v * math.cos(mid) * dt
        self.y += v * math.sin(mid) * dt
        self.yaw = wrap(self.yaw + w * dt)
        self.travel += abs(v) * dt
        self.uncertainty_m += abs(v) * dt * (0.15 if hardware else 0.001)

    def dict(self):
        return asdict(self)
