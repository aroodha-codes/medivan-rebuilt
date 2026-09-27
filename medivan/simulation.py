"""Independent simulated world. Mapping receives only ray measurements."""

import math, time
import numpy as np
from .common import Observation, Pose, wrap


class Simulator:
    hardware = False

    def __init__(self, cfg):
        self.cfg = cfg
        self.truth = Pose()
        self.command = (0.0, 0.0)
        self.voltage = 8.0
        self.current = 0.5
        self.collision = False
        self.rectangles = [
            (-3, -3, 3, -2.8),
            (-3, 2.8, 3, 3),
            (-3, -3, -2.8, 3),
            (2.8, -3, 3, 3),
            (0.9, 0.7, 1.4, 1.2),
            (-1.5, -1.3, -0.9, -0.7),
        ]
        self.radius = (
            math.hypot(
                cfg["geometry"]["robot_width_m"], cfg["geometry"]["robot_length_m"]
            )
            / 2
        )

    def blocked(self, x, y):
        return any(
            a - self.radius < x < c + self.radius
            and b - self.radius < y < d + self.radius
            for a, b, c, d in self.rectangles
        )

    def drive(self, v, w):
        self.command = (v, w)

    def stop(self):
        self.command = (0.0, 0.0)

    def close(self):
        self.stop()

    def reset_pose(self, pose):
        self.truth = Pose(pose.x, pose.y, pose.yaw)

    def sample(self, dt, now):
        v, w = self.command
        p = self.truth
        nx = p.x + v * math.cos(p.yaw + w * dt / 2) * dt
        ny = p.y + v * math.sin(p.yaw + w * dt / 2) * dt
        if self.blocked(nx, ny):
            self.collision = True
            v = 0
        p.advance(v, w, dt)
        rays = []
        for a in np.linspace(-math.pi / 3, math.pi / 3, 81):
            hit = False
            distance = 2.5
            for d in np.arange(0.03, 2.51, 0.03):
                x = p.x + d * math.cos(p.yaw + a)
                y = p.y + d * math.sin(p.yaw + a)
                if any(
                    x1 <= x <= x2 and y1 <= y <= y2
                    for x1, y1, x2, y2 in self.rectangles
                ):
                    distance = float(d)
                    hit = True
                    break
            rays.append((float(a), distance, hit))
        self.voltage = max(6.2, self.voltage - abs(v) * dt * 0.0004)
        # Virtual dock at x=-.8, visible when facing west. Only simulation.
        dx = -0.8 - p.x
        dy = -p.y
        bearing = wrap(math.atan2(dy, dx) - p.yaw)
        dist = math.hypot(dx, dy)
        marker = (
            {
                "distance": dist,
                "bearing": bearing,
                "normal_error": wrap(math.pi - p.yaw),
            }
            if abs(bearing) < 0.65 and dist < 2
            else None
        )
        # Contact plane x=-0.60; simulated rear contacts extend 0.15 m.
        charging = p.x < -0.45 and abs(p.y) < 0.09 and abs(wrap(p.yaw)) < 0.15
        self.current = -0.3 if charging else 0.5
        if charging:
            self.voltage = min(8.4, self.voltage + 0.01 * dt)
        return Observation(
            now,
            w,
            0,
            self.voltage,
            self.current,
            rays,
            1,
            now,
            now,
            marker,
            [],
            ["collision"] if self.collision else [],
        )

    def jpeg(self):
        return None
