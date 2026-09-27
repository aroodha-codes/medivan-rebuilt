"""One mission controller for both adapters. All mutations occur under the server lock."""

import math, time, json, logging
from collections import deque
from .common import Pose, ROOT, wrap
from .mapping import Grid

MOVING = {
    "exploring",
    "navigating",
    "returning",
    "dock_align",
    "dock_turn",
    "dock_reverse",
    "manual",
}


class Controller:
    def __init__(self, cfg, adapter, clock=time.monotonic):
        self.cfg = cfg
        self.io = adapter
        self.clock = clock
        self.grid = Grid(cfg)
        self.pose = Pose()
        self.pose_confirmed = not adapter.hardware
        self.grid.seed(0, 0)
        self.state = "idle"
        self.armed = False
        self.estop = False
        self.reason = "Ready"
        self.locations = {}
        self.queue = []
        self.active = None
        self.path = []
        self.dock = {"x": 0.0, "y": 0.0, "yaw": math.pi}
        self.command = (0.0, 0.0)
        self.obs = None
        self.events = deque(maxlen=100)
        self.trail = deque(maxlen=1500)
        self.entered = clock()
        self.last_plan = 0
        self.scan_stamp = -1
        self.excluded = []
        self.explore_start = 0
        self.map_finished = False
        self.blocked_since = None
        self.charge_since = None
        self.resume_state = None
        self.manual_until = 0
        self.manual_cmd = (0, 0)
        self.next_id = 1
        self.session_start_travel = 0
        self.log(
            "Controller ready in "
            + ("hardware" if adapter.hardware else "simulation")
            + " mode"
        )

    def log(self, message):
        logging.getLogger("medivan.events").info(str(message))
        self.events.appendleft(
            {"time": time.strftime("%H:%M:%S"), "message": str(message)}
        )

    def transition(self, state, reason):
        self.io.stop()
        self.command = (0.0, 0.0)
        self.state = state
        self.reason = reason
        self.entered = self.clock()
        self.path = []
        self.blocked_since = None
        self.log(reason)

    def latch_fault(self, reason):
        if self.state != "fault":
            self.transition("fault", reason)
        self.armed = False

    def hardware_gates(self):
        if not self.io.hardware:
            return []
        c = self.cfg
        flags = [
            (c["geometry"]["footprint_confirmed"], "Measure footprint"),
            (c["motor"]["calibrated"], "Calibrate motor speed and signs"),
            (c["sensors"]["battery_calibrated"], "Calibrate battery sensors"),
            (c["sensors"]["imu_flat_confirmed"], "Confirm flat IMU"),
            (
                c["navigation"]["allow_physical_autonomy"],
                "Enable calibrated physical autonomy",
            ),
        ]
        return [label for ok, label in flags if not ok]

    def safety(self, now):
        o = self.obs
        if not o:
            return "Waiting for sensors"
        if o.faults:
            return "; ".join(o.faults)
        if now - o.stamp > 0.25:
            return "IMU data stale"
        if now - o.frame_stamp > self.cfg["camera"]["stale_s"]:
            return "Camera data stale"
        if now - o.battery_stamp > 1.5:
            return "Battery data stale"
        if not all(math.isfinite(x) for x in [o.yaw_rate, o.tilt_deg]):
            return "Invalid IMU data"
        if abs(o.tilt_deg) > 20:
            return "Tilt exceeds 20 degrees"
        if o.voltage is None or not math.isfinite(o.voltage):
            return "Battery voltage unavailable"
        if o.voltage < self.cfg["battery"]["critical_voltage"]:
            return "Critical battery voltage"
        if (
            self.io.hardware
            and self.pose.travel - self.session_start_travel
            > self.cfg["navigation"]["physical_max_travel_m"]
        ):
            return (
                "Unverified odometry travel limit reached; stop and re-establish pose"
            )
        return None

    def action(self, name, data=None):
        data = data or {}
        now = self.clock()
        if name == "estop":
            if hasattr(self.io, "latch_stop"):
                self.io.latch_stop()
            self.estop = True
            self.armed = False
            self.transition("estop", "Emergency stop latched")
            return
        if name == "reset":
            self.io.stop()
            self.armed = False
            self.estop = False
            self.active = None
            self.queue = []
            self.transition(
                "idle" if self.pose_confirmed else "needs_pose",
                "Stopped and disarmed; mission cleared",
            )
            return
        if self.estop:
            raise ValueError("Reset the emergency stop first")
        if name == "arm":
            if not self.pose_confirmed:
                raise ValueError("Confirm physical placement and pose before arming")
            errors = self.hardware_gates()
            fault = self.safety(now)
            if errors or fault:
                raise ValueError("; ".join(errors + ([fault] if fault else [])))
            if self.state == "fault":
                raise ValueError("Reset fault before arming")
            if hasattr(self.io, "rearm"):
                self.io.rearm()
            self.armed = True
            self.log("Motion armed")
            return
        if name == "pause":
            self.resume_state = self.state
            self.transition("paused", "Operator pause")
            return
        if name == "resume":
            if self.state != "paused" or self.resume_state not in {
                "exploring",
                "navigating",
                "returning",
            }:
                raise ValueError("Restart this operation explicitly")
            self.require_arm()
            s = self.resume_state
            self.transition(s, "Resumed")
            return
        if name == "save":
            self.grid.save(ROOT / "data/map.json", self.locations, self.dock)
            self.log("Map and destinations saved")
            return
        if name == "load":
            if self.armed or self.state in MOVING:
                raise ValueError("Disarm before loading")
            d = self.grid.load(ROOT / "data/map.json")
            self.locations = d["locations"]
            self.dock = d["dock"]
            self.map_finished = True
            self.pose_confirmed = False
            self.transition(
                "needs_pose", "Map loaded. Confirm physical placement and heading."
            )
            return
        if name == "set_pose":
            if self.armed:
                raise ValueError("Disarm before setting pose")
            x, y = self.point(data)
            yaw = float(data.get("yaw", 0))
            if not math.isfinite(yaw):
                raise ValueError("Finite yaw required")
            self.pose = Pose(x, y, wrap(yaw))
            self.pose_confirmed = True
            self.session_start_travel = 0
            if not self.map_finished:
                self.grid.seed(x, y)
            if not self.io.hardware:
                self.io.reset_pose(self.pose)
            self.transition("idle", "Pose established by operator")
            return
        if name == "location":
            x, y = self.point(data)
            label = str(data.get("name", "")).strip()
            if not label or len(label) > 60:
                raise ValueError("Provide a name up to 60 characters")
            if not self.grid.plan((self.pose.x, self.pose.y), (x, y)):
                raise ValueError("Location must be reachable in observed free space")
            self.locations[label] = [x, y]
            self.log("Saved destination " + label)
            return
        if name == "dock_here":
            if self.armed:
                raise ValueError("Disarm first")
            self.dock = {"x": self.pose.x, "y": self.pose.y, "yaw": self.pose.yaw}
            self.log("Current pose saved as front-facing dock staging pose")
            return
        if name == "enqueue":
            if not self.map_finished:
                raise ValueError("Finish or load mapping first")
            if data.get("name") in self.locations:
                x, y = self.locations[data["name"]]
            else:
                x, y = self.point(data)
            if not self.grid.plan((self.pose.x, self.pose.y), (x, y)):
                raise ValueError("Destination has no observed safe route")
            mission = {
                "id": self.next_id,
                "name": str(data.get("name") or "Map destination")[:60],
                "x": x,
                "y": y,
            }
            self.next_id += 1
            self.queue.append(mission)
            self.log("Queued " + mission["name"])
            return
        if name == "cancel":
            self.queue = [q for q in self.queue if q["id"] != data.get("id")]
            return
        if name == "finish_mapping":
            self.map_finished = True
            self.transition("idle", "Mapping finished; label destinations and save map")
            return
        self.require_arm()
        if self.state == "needs_pose":
            raise ValueError("Establish pose first")
        if name == "explore":
            if self.state != "idle":
                raise ValueError(
                    "Stop the current operation before starting exploration"
                )
            self.explore_start = now
            self.excluded = []
            self.sweep_start = self.pose.yaw
            self.sweep_accum = 0
            self.transition("exploring", "Exploring observed free space")
            return
        if name == "start":
            if self.state != "idle":
                raise ValueError(
                    "Stop the current operation before starting another delivery"
                )
            if not self.queue:
                raise ValueError("Queue a delivery first")
            if self.obs.voltage < self.cfg["battery"]["low_voltage"]:
                raise ValueError("Battery too low to start delivery")
            self.active = self.queue.pop(0)
            self.transition("navigating", "Delivering to " + self.active["name"])
            return
        if name == "confirm":
            if self.state != "awaiting_collection":
                raise ValueError("No delivery awaiting collection")
            self.log("Collection confirmed for " + self.active["name"])
            self.active = None
            if self.queue:
                self.active = self.queue.pop(0)
                self.transition("navigating", "Next delivery: " + self.active["name"])
            else:
                self.transition(
                    "returning", "Deliveries complete; returning to staging point"
                )
            return
        if name == "return":
            self.transition("returning", "Return to dock requested")
            return
        if name == "dock":
            if self.state != "awaiting_dock":
                raise ValueError("Return to staging point first")
            gates = []
            if self.io.hardware:
                c = self.cfg["dock"]
                for k in [
                    "geometry_calibrated",
                    "rear_clearance_confirmed",
                    "charger_verified",
                ]:
                    if not c[k]:
                        gates.append(k)
                if not self.cfg["sensors"]["current_sees_charge"]:
                    gates.append("current_sees_charge")
            if gates:
                raise ValueError("Dock checks incomplete: " + ", ".join(gates))
            if data.get("supervised") is not True:
                raise ValueError("Supervised docking confirmation required")
            self.transition("dock_align", "Looking for dock marker")
            return
        if name == "manual":
            v = float(data.get("v", 0))
            w = float(data.get("w", 0))
            if not math.isfinite(v + w) or abs(v) > 0.06 or abs(w) > 0.5 or v < 0:
                raise ValueError(
                    "Manual command outside limits; reverse reserved for supervised docking"
                )
            if self.state != "manual":
                self.transition("manual", "Manual deadman control")
            self.manual_cmd = (v, w)
            self.manual_until = now + 0.25
            return
        raise ValueError("Unknown action")

    def point(self, d):
        x = float(d["x"])
        y = float(d["y"])
        if not math.isfinite(x + y) or not self.grid.inside(self.grid.cell(x, y)):
            raise ValueError("Point outside map")
        return x, y

    def require_arm(self):
        if not self.armed:
            raise ValueError("Arm motion first")

    def tick(self, dt, now=None):
        now = self.clock() if now is None else now
        if not 0 < dt <= 0.25:
            self.latch_fault("Control loop missed timing deadline")
            return
        self.obs = self.io.sample(dt, now)
        if self.state not in {"fault", "estop", "needs_pose"}:
            v, w = self.command
            self.pose.advance(
                v, self.obs.yaw_rate if self.io.hardware else w, dt, self.io.hardware
            )
        if (
            self.pose_confirmed
            and self.obs.frame_stamp != self.scan_stamp
            and self.obs.confidence >= self.cfg["camera"]["min_confidence"]
        ):
            self.grid.update(self.pose, self.obs.rays)
            self.scan_stamp = self.obs.frame_stamp
        fault = self.safety(now)
        if self.armed and fault:
            self.latch_fault(fault)
        if self.estop or not self.armed or self.state not in MOVING:
            self.io.stop()
            self.command = (0.0, 0.0)
            self.monitor_charge(now)
            return
        if self.obs.voltage < self.cfg["battery"]["low_voltage"] and self.state in {
            "exploring",
            "navigating",
        }:
            self.transition("returning", "Low battery; returning to dock staging point")
        v = w = 0.0
        if self.state == "manual":
            if now < self.manual_until:
                v, w = self.manual_cmd
            else:
                self.transition("idle", "Manual command released")
        elif self.state == "exploring":
            if now - self.explore_start > self.cfg["navigation"]["exploration_seconds"]:
                self.transition("idle", "Exploration time budget reached; inspect map")
                return
            if self.sweep_accum < 2 * math.pi:
                w = self.cfg["navigation"]["angular_speed_rad_s"]
                self.sweep_accum += abs(self.obs.yaw_rate) * dt
            else:
                if not self.path:
                    self.path = self.grid.frontier_path(
                        (self.pose.x, self.pose.y), self.excluded
                    )
                    if not self.path:
                        self.transition(
                            "idle",
                            "No reachable frontier; inspect map and finish mapping",
                        )
                        return
                target = self.path[-1]
                if math.dist((self.pose.x, self.pose.y), target) < 0.1:
                    self.excluded.append(target)
                    self.path = []
                    self.sweep_accum = 0
                else:
                    v, w = self.follow(target, now)
        elif self.state in {"navigating", "returning"}:
            target = (
                (self.active["x"], self.active["y"])
                if self.state == "navigating"
                else (self.dock["x"], self.dock["y"])
            )
            if math.dist((self.pose.x, self.pose.y), target) < 0.1:
                if self.state == "navigating":
                    self.transition(
                        "awaiting_collection",
                        "Arrived; waiting for collection confirmation",
                    )
                else:
                    err = wrap(self.dock["yaw"] - self.pose.yaw)
                    if abs(err) > 0.08:
                        w = max(-0.4, min(0.4, 1.2 * err))
                    else:
                        self.transition(
                            "awaiting_dock",
                            "At staging point; supervised docking available",
                        )
            elif now - self.entered > self.cfg["navigation"]["max_leg_seconds"]:
                self.latch_fault("Navigation timeout")
            else:
                v, w = self.follow(target, now)
        elif self.state == "dock_align":
            m = self.obs.marker
            c = self.cfg["dock"]
            if now - self.entered > 30:
                self.latch_fault("Dock marker alignment timeout")
            elif m:
                b = m["bearing"]
                normal = m.get("normal_error", 0)
                distance = m["distance"]
                if (
                    abs(b) < 0.05
                    and abs(normal) < 0.12
                    and abs(distance - c["staging_distance_m"]) < 0.025
                ):
                    self.turn_target = wrap(self.pose.yaw + math.pi)
                    self.transition(
                        "dock_turn", "Aligned; turning rear contacts toward dock"
                    )
                else:
                    w = max(-0.35, min(0.35, b * 1.5))
                    if abs(b) < 0.1 and distance > c["staging_distance_m"] + 0.02:
                        v = 0.025
                    if distance < c["staging_distance_m"] - 0.06:
                        self.latch_fault(
                            "Too close for calibrated docking; reposition manually"
                        )
            else:
                w = 0.2
        elif self.state == "dock_turn":
            err = wrap(self.turn_target - self.pose.yaw)
            if now - self.entered > 25:
                self.latch_fault("Dock turn timeout")
            elif abs(err) < 0.06:
                self.reverse_start = self.pose.travel
                self.charge_since = None
                self.transition(
                    "dock_reverse",
                    "Supervised reverse; front marker is no longer visible",
                )
            else:
                w = max(-0.35, min(0.35, err * 1.3))
        elif self.state == "dock_reverse":
            c = self.cfg["dock"]
            charging = (
                self.obs.current is not None
                and self.obs.current < c["current_threshold_a"]
            )
            if charging:
                self.charge_since = self.charge_since or now
                if now - self.charge_since >= c["charge_confirm_seconds"]:
                    self.transition(
                        "charging", "Charging current confirmed; motors stopped"
                    )
            else:
                self.charge_since = None
                if (
                    now - self.entered > c["reverse_timeout_s"]
                    or self.pose.travel - self.reverse_start >= c["reverse_distance_m"]
                ):
                    self.latch_fault(
                        "Reverse limit reached without verified charging contact"
                    )
                else:
                    v = -c["reverse_speed_m_s"]
                    w = max(-0.15, min(0.15, wrap(self.turn_target - self.pose.yaw)))
        if self.state in MOVING:
            # Reverse is constrained separately; forward camera cannot certify rear clearance.
            if (
                self.state != "dock_reverse"
                and self.obs.confidence < self.cfg["camera"]["min_confidence"]
            ):
                self.latch_fault("Floor perception confidence too low")
                v = w = 0
            if v > 0 and any(
                abs(a) < 0.45 and hit and d < self.cfg["navigation"]["stop_distance_m"]
                for a, d, hit in self.obs.rays
            ):
                v = w = 0
                self.blocked_since = self.blocked_since or now
                if now - self.blocked_since > 10:
                    self.latch_fault("Obstacle blocks route; inspect and restart")
            else:
                self.blocked_since = None
        if self.state not in MOVING or not self.armed:
            v = w = 0
        self.command = (v, w)
        self.io.drive(v, w)
        if (
            len(self.trail) == 0
            or math.dist(self.trail[-1], (self.pose.x, self.pose.y)) > 0.03
        ):
            self.trail.append([self.pose.x, self.pose.y])

    def follow(self, target, now):
        # Replan periodically so changed obstacles invalidate a previously safe route.
        if not self.path or now - self.last_plan > 0.75:
            self.path = self.grid.plan((self.pose.x, self.pose.y), target)
            self.last_plan = now
        if not self.path:
            self.latch_fault("No safe observed route to destination")
            return 0.0, 0.0
        while (
            len(self.path) > 1
            and math.dist(self.path[0], (self.pose.x, self.pose.y)) < 0.12
        ):
            self.path.pop(0)
        x, y = self.path[0]
        err = wrap(math.atan2(y - self.pose.y, x - self.pose.x) - self.pose.yaw)
        w = max(-0.5, min(0.5, 1.8 * err))
        v = (
            self.cfg["navigation"]["speed_m_s"] * max(0, math.cos(err))
            if abs(err) < 0.6
            else 0.0
        )
        return v, w

    def monitor_charge(self, now):
        if self.state == "charging":
            if self.obs.current is None or now - self.obs.battery_stamp > 1.5:
                self.latch_fault("Charging telemetry lost")
            elif self.obs.current >= self.cfg["dock"]["current_threshold_a"]:
                self.transition(
                    "charge_idle",
                    "Charge current ended; software does not control charger",
                )

    def snapshot(self):
        o = self.obs
        return {
            "mode": "hardware" if self.io.hardware else "simulation",
            "state": self.state,
            "armed": self.armed,
            "reason": self.reason,
            "pose": self.pose.dict(),
            "voltage": o.voltage if o else None,
            "current": o.current if o else None,
            "battery_percent_estimate": (
                None
                if not o or o.voltage is None
                else round(max(0, min(100, (o.voltage - 6.4) / 2 * 100)))
            ),
            "battery_note": "Voltage-based estimate, not measured state of charge",
            "confidence": o.confidence if o else 0,
            "sensor_fault": self.safety(self.clock()),
            "gates": self.hardware_gates(),
            "queue": self.queue,
            "active": self.active,
            "locations": self.locations,
            "dock": self.dock,
            "path": self.path,
            "trail": list(self.trail),
            "events": list(self.events),
            "map_finished": self.map_finished,
            "pose_confirmed": self.pose_confirmed,
            "detections": o.detections if o else [],
            "command": self.command,
            "ranges": o.rays if o else [],
            "marker": o.marker if o else None,
            "tilt_deg": o.tilt_deg if o else None,
        }
