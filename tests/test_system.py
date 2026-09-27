import copy, math, sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from medivan.common import load_config, Observation, Pose
from medivan.controller import Controller
from medivan.simulation import Simulator
from medivan.mapping import Grid
from medivan.server import Runner, create_app


class Stub:
    hardware = False

    def __init__(self):
        self.command = (0, 0)
        self.failure = None
        self.voltage = 8.0
        self.current = 0.5
        self.tilt = 0
        self.stale = False

    def stop(self):
        self.command = (0, 0)

    def drive(self, v, w):
        self.command = (v, w)

    def close(self):
        self.stop()

    def jpeg(self):
        return None

    def reset_pose(self, p):
        pass

    def sample(self, dt, now):
        return Observation(
            now,
            yaw_rate=self.command[1],
            tilt_deg=self.tilt,
            voltage=self.voltage,
            current=self.current,
            rays=[(0.0, 2.0, False)],
            confidence=1.0,
            frame_stamp=now - 2 if self.stale else now,
            battery_stamp=now,
            faults=[self.failure] if self.failure else [],
        )


@pytest.fixture
def rig():
    t = [10.0]
    io = Stub()
    c = Controller(load_config(), io, lambda: t[0])
    c.tick(0.05, t[0])
    return c, io, t


def test_hardware_configuration():
    cfg = load_config()
    assert cfg["motor"]["left"] == [5, 6, 12]
    assert cfg["motor"]["right"] == [13, 19, 18]
    assert (
        cfg["geometry"]["camera_height_m"] == 0.11
        and cfg["geometry"]["wheel_diameter_m"] == 0.05
    )


def test_unarmed_never_moves(rig):
    c, io, t = rig
    with pytest.raises(ValueError):
        c.action("explore")
    assert io.command == (0, 0)


def test_estop_latches_and_reset_disarms(rig):
    c, io, t = rig
    c.action("arm")
    c.action("explore")
    c.action("estop")
    with pytest.raises(ValueError):
        c.action("arm")
    c.tick(0.05, t[0])
    assert io.command == (0, 0)
    c.action("reset")
    assert not c.armed and not c.estop


@pytest.mark.parametrize("fault", ["I2C disconnected", "Camera failed", "collision"])
def test_sensor_faults_stop(rig, fault):
    c, io, t = rig
    c.action("arm")
    c.action("explore")
    io.failure = fault
    t[0] += 0.05
    c.tick(0.05, t[0])
    assert c.state == "fault" and io.command == (0, 0) and not c.armed


def test_stale_frame_stop(rig):
    c, io, t = rig
    c.action("arm")
    c.action("explore")
    io.stale = True
    c.tick(0.05, t[0])
    assert c.state == "fault"


def test_tilt_stop(rig):
    c, io, t = rig
    c.action("arm")
    io.tilt = 25
    c.tick(0.05, t[0])
    assert c.state == "fault"


def test_critical_battery_stop(rig):
    c, io, t = rig
    c.action("arm")
    io.voltage = 6.2
    c.tick(0.05, t[0])
    assert c.state == "fault"


def test_missed_deadline_stop(rig):
    c, io, t = rig
    c.action("arm")
    c.tick(0.5, t[0])
    assert c.state == "fault"


def test_manual_deadman(rig):
    c, io, t = rig
    c.action("arm")
    c.action("manual", {"v": 0.04, "w": 0})
    c.tick(0.05, t[0])
    assert io.command[0] > 0
    t[0] += 0.3
    c.tick(0.05, t[0])
    assert io.command == (0, 0)


def test_reverse_manual_rejected(rig):
    c, _, _ = rig
    c.action("arm")
    with pytest.raises(ValueError):
        c.action("manual", {"v": -0.02})


def test_physical_gates(rig):
    c, io, t = rig
    io.hardware = True
    with pytest.raises(ValueError, match="footprint"):
        c.action("arm")


def test_unknown_is_not_traversable():
    g = Grid(load_config())
    g.seed(0, 0)
    assert not g.plan((0, 0), (1, 0))


def test_path_avoids_inflated_obstacle():
    g = Grid(load_config())
    g.odds[:] = -4
    x, y = g.cell(0, 0)
    g.odds[y - 10 : y + 11, x] = 5
    p = g.plan((-1, 0), (1, 0))
    assert p
    safe = g.safe_mask()
    assert all(safe[g.cell(*q)[1], g.cell(*q)[0]] for q in p)
    assert max(abs(q[1]) for q in p) > 0.6


def test_saved_map_roundtrip_and_mismatch(tmp_path):
    g = Grid(load_config())
    g.seed(0, 0)
    p = tmp_path / "map.json"
    g.save(p, {"Ward": [1, 2]}, {"x": 0, "y": 0, "yaw": 0})
    h = Grid(load_config())
    d = h.load(p)
    assert np.array_equal(g.odds, h.odds)
    assert d["locations"]["Ward"] == [1, 2]
    cfg = load_config()
    cfg["navigation"]["resolution_m"] = 0.1
    with pytest.raises(ValueError):
        Grid(cfg).load(p)


def test_pose_required_after_load(rig, tmp_path, monkeypatch):
    import medivan.controller as mod

    c, io, t = rig
    monkeypatch.setattr(mod, "ROOT", tmp_path)
    (tmp_path / "data").mkdir()
    c.action("save")
    c.action("load")
    assert c.state == "needs_pose"
    with pytest.raises(ValueError, match="pose"):
        c.action("arm")
    c.action("reset")
    with pytest.raises(ValueError, match="pose"):
        c.action("arm")


def test_delivery_confirmation_and_queue(rig):
    c, io, t = rig
    c.grid.odds[:] = -4
    c.map_finished = True
    c.action("enqueue", {"x": 0.03, "y": 0.03, "name": "Ward 1"})
    c.action("arm")
    c.action("start")
    c.tick(0.05, t[0])
    assert c.state == "awaiting_collection"
    c.action("confirm")
    assert c.state == "returning" and c.active is None


def test_reverse_contact_timeout(rig):
    c, io, t = rig
    c.action("arm")
    c.state = "dock_reverse"
    c.reverse_start = 0
    c.turn_target = 0
    c.entered = t[0] - 20
    c.tick(0.05, t[0])
    assert c.state == "fault" and io.command == (0, 0)


def test_charge_requires_sustained_current(rig):
    c, io, t = rig
    c.action("arm")
    c.state = "dock_reverse"
    c.reverse_start = 0
    c.turn_target = 0
    io.current = -0.2
    for _ in range(43):
        t[0] += 0.05
        c.tick(0.05, t[0])
    assert c.state == "charging" and io.command == (0, 0)


def test_dock_requires_supervision(rig):
    c, io, t = rig
    c.action("arm")
    c.state = "awaiting_dock"
    with pytest.raises(ValueError, match="Supervised"):
        c.action("dock")


def test_api_auth_and_validation(rig):
    c, io, t = rig
    app = create_app(Runner(c), "test-token")
    client = app.test_client()
    h = {"X-MediVan-Token": "test-token"}
    assert client.get("/api/state").status_code == 401
    assert client.get("/api/state", headers=h).status_code == 200
    assert (
        client.post(
            "/api/action",
            headers=h,
            json={"action": "enqueue", "data": {"x": float("nan"), "y": 0}},
        ).status_code
        == 400
    )
    assert (
        client.post(
            "/api/action",
            headers={**h, "Origin": "https://other.example"},
            json={"action": "arm"},
        ).status_code
        == 403
    )
    assert client.get("/").status_code == 200
    assert client.get("/static/app.js").status_code == 200


def test_diagnostic_adapter_refuses_motion():
    from medivan.hardware import Hardware

    h = Hardware.__new__(Hardware)
    h.motors = None
    with pytest.raises(RuntimeError):
        h.drive(0.01, 0)


def test_yolo_supplied_model_loads():
    from medivan.vision import Vision

    v = Vision(load_config(), False)
    assert isinstance(v.objects(np.zeros((480, 640, 3), np.uint8)), list)


def test_simulation_delivery_roundtrip():
    t = [1.0]
    cfg = load_config()
    c = Controller(cfg, Simulator(cfg), lambda: t[0])
    c.tick(0.05, t[0])
    c.action("arm")
    c.action("explore")
    # A physical-style 360-degree scan, not injected ground truth.
    for _ in range(270):
        t[0] += 0.05
        c.tick(0.05, t[0])
    c.action("finish_mapping")
    c.action("enqueue", {"name": "Ward 1", "x": 0.6, "y": 0})
    c.action("start")
    for _ in range(1100):
        t[0] += 0.05
        c.tick(0.05, t[0])
        if c.state in {"awaiting_collection", "fault"}:
            break
    assert c.state == "awaiting_collection", c.reason
    c.action("confirm")
    for _ in range(1100):
        t[0] += 0.05
        c.tick(0.05, t[0])
        if c.state in {"awaiting_dock", "fault"}:
            break
    assert c.state == "awaiting_dock", c.reason
    assert not c.io.collision


def test_entire_simulated_docking_sequence():
    t = [1.0]
    cfg = load_config()
    io = Simulator(cfg)
    c = Controller(cfg, io, lambda: t[0])
    c.pose = Pose(0, 0, math.pi)
    io.reset_pose(c.pose)
    c.tick(0.05, t[0])
    c.action("arm")
    c.state = "awaiting_dock"
    c.action("dock", {"supervised": True})
    seen = set()
    for _ in range(2000):
        t[0] += 0.05
        c.tick(0.05, t[0])
        seen.add(c.state)
        if c.state in {"charging", "fault"}:
            break
    assert c.state == "charging", c.reason
    assert {"dock_align", "dock_turn", "dock_reverse", "charging"} <= seen
    assert io.command == (0, 0)


def test_motor_watchdog_removes_output(monkeypatch):
    import types, time

    class Pin:
        def __init__(self, *a, **kw):
            self.value = 0

        def on(self):
            self.value = 1

        def off(self):
            self.value = 0

        def close(self):
            self.value = 0

    class Factory:
        def close(self):
            pass

    gpio = types.ModuleType("gpiozero")
    gpio.DigitalOutputDevice = Pin
    gpio.PWMOutputDevice = Pin
    lgpio = types.ModuleType("gpiozero.pins.lgpio")
    lgpio.LGPIOFactory = Factory
    monkeypatch.setitem(sys.modules, "gpiozero", gpio)
    monkeypatch.setitem(sys.modules, "gpiozero.pins.lgpio", lgpio)
    from medivan.hardware import Motors

    cfg = load_config()
    cfg["motor"]["watchdog_s"] = 0.05
    m = Motors(cfg)
    try:
        m.drive(0.04, 0)
        assert any(e.value for a, b, e in m.channels)
        time.sleep(0.12)
        assert all(e.value == 0 for a, b, e in m.channels)
    finally:
        m.close()


def test_aruco_pose_from_known_render(tmp_path, monkeypatch):
    import cv2, json
    import medivan.vision as mod

    cfg = load_config()
    cfg["vision"]["yolo_enabled"] = False
    cfg["camera"]["calibration"] = "camera.json"
    cfg["camera"]["floor_profile"] = "floor.json"
    (tmp_path / "camera.json").write_text(
        json.dumps(
            {
                "matrix": [[600, 0, 320], [0, 600, 240], [0, 0, 1]],
                "distortion": [0, 0, 0, 0, 0],
                "size": [640, 480],
            }
        )
    )
    lab = cv2.cvtColor(np.full((1, 1, 3), 255, np.uint8), cv2.COLOR_BGR2LAB)[
        0, 0
    ].tolist()
    (tmp_path / "floor.json").write_text(json.dumps({"lab": lab}))
    monkeypatch.setattr(mod, "ROOT", tmp_path)
    v = mod.Vision(cfg)
    frame = np.full((480, 640, 3), 255, np.uint8)
    marker = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 0, 120
    )
    frame[180:300, 260:380] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    _, _, pose, _, _ = v.process(frame)
    assert pose is not None
    assert abs(pose["distance"] - 0.5) < 0.02 and abs(pose["bearing"]) < 0.01
