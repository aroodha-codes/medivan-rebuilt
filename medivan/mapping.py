"""Log-odds mapping and conservative A*. Unknown space is not traversable."""

import heapq, math
import numpy as np
from .common import atomic_json


class Grid:
    def __init__(self, cfg):
        n = cfg["navigation"]
        self.res = n["resolution_m"]
        self.size = int(n["map_size_m"] / self.res)
        self.origin = -self.size * self.res / 2
        self.odds = np.zeros((self.size, self.size), np.float32)
        g = cfg["geometry"]
        self.radius = (
            math.hypot(g["robot_width_m"], g["robot_length_m"]) / 2 + n["clearance_m"]
        )

    def cell(self, x, y):
        return int(math.floor((x - self.origin) / self.res)), int(
            math.floor((y - self.origin) / self.res)
        )

    def world(self, c):
        return (
            self.origin + (c[0] + 0.5) * self.res,
            self.origin + (c[1] + 0.5) * self.res,
        )

    def inside(self, c):
        return 0 <= c[0] < self.size and 0 <= c[1] < self.size

    def seed(self, x, y):
        # Only the occupied footprint is assumed clear at confirmed placement.
        cx, cy = self.cell(x, y)
        r = int(math.ceil(self.radius / self.res)) + 1
        for yy in range(max(0, cy - r), min(self.size, cy + r + 1)):
            for xx in range(max(0, cx - r), min(self.size, cx + r + 1)):
                if math.hypot(xx - cx, yy - cy) <= r:
                    self.odds[yy, xx] = -3

    def update(self, pose, rays):
        for a, d, hit in rays:
            if not math.isfinite(d) or d <= 0:
                continue
            angle = pose.yaw + a
            distances = np.arange(0, d, self.res * 0.7)
            seen = set()
            for t in distances:
                cell = self.cell(
                    pose.x + t * math.cos(angle), pose.y + t * math.sin(angle)
                )
                if not self.inside(cell):
                    break
                if cell not in seen:
                    self.odds[cell[1], cell[0]] -= 0.35
                    seen.add(cell)
            end = self.cell(pose.x + d * math.cos(angle), pose.y + d * math.sin(angle))
            if hit and self.inside(end):
                self.odds[end[1], end[0]] += 1.5
        np.clip(self.odds, -5, 5, out=self.odds)

    def safe_mask(self):
        import cv2

        occupied = (self.odds > 0.6).astype(np.uint8)
        r = int(math.ceil(self.radius / self.res))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
        inflated = cv2.dilate(occupied, kernel) > 0
        # Erode known free space so the entire swept footprint is observed.
        free = (
            cv2.erode(
                (self.odds < -0.5).astype(np.uint8),
                kernel,
                borderType=cv2.BORDER_CONSTANT,
                borderValue=0,
            )
            > 0
        )
        return free & ~inflated

    def plan(self, start, goal):
        s = self.cell(*start)
        g = self.cell(*goal)
        safe = self.safe_mask()
        if (
            not self.inside(s)
            or not self.inside(g)
            or not safe[s[1], s[0]]
            or not safe[g[1], g[0]]
        ):
            return []
        queue = [(0, 0, s)]
        cost = {s: 0}
        prev = {}
        while queue:
            _, val, c = heapq.heappop(queue)
            if val > cost[c]:
                continue
            if c == g:
                cells = [c]
                while c != s:
                    c = prev[c]
                    cells.append(c)
                return [self.world(p) for p in reversed(cells)]
            for dx, dy in [
                (1, 0),
                (-1, 0),
                (0, 1),
                (0, -1),
                (1, 1),
                (1, -1),
                (-1, 1),
                (-1, -1),
            ]:
                q = (c[0] + dx, c[1] + dy)
                if not self.inside(q) or not safe[q[1], q[0]]:
                    continue
                if dx and dy and (not safe[c[1], q[0]] or not safe[q[1], c[0]]):
                    continue
                score = val + math.hypot(dx, dy)
                if score < cost.get(q, float("inf")):
                    cost[q] = score
                    prev[q] = c
                    heapq.heappush(
                        queue, (score + math.hypot(q[0] - g[0], q[1] - g[1]), score, q)
                    )
        return []

    def frontier_path(self, start, excluded=()):
        import cv2

        unknown = (abs(self.odds) < 0.1).astype(np.uint8)
        r = int(math.ceil(self.radius / self.res)) + 3
        near = cv2.dilate(unknown, np.ones((2 * r + 1, 2 * r + 1), np.uint8)) > 0
        candidates = np.argwhere(self.safe_mask() & near)
        ordered = sorted(
            candidates, key=lambda p: math.dist(self.world((p[1], p[0])), start)
        )
        for y, x in ordered[:: max(1, len(ordered) // 150)]:
            w = self.world((x, y))
            if math.dist(w, start) < 0.25 or any(
                math.dist(w, e) < 0.3 for e in excluded
            ):
                continue
            path = self.plan(start, w)
            if path:
                return path
        return []

    def save(self, path, locations, dock):
        atomic_json(
            path,
            {
                "version": 1,
                "resolution": self.res,
                "size": self.size,
                "odds": self.odds.tolist(),
                "locations": locations,
                "dock": dock,
            },
        )

    def load(self, path):
        import json

        d = json.loads(path.read_text())
        a = np.asarray(d["odds"], dtype=np.float32)
        if (
            d["version"] != 1
            or a.shape != self.odds.shape
            or d["resolution"] != self.res
            or not np.isfinite(a).all()
        ):
            raise ValueError("Map configuration mismatch or corrupt values")
        self.odds[:] = np.clip(a, -5, 5)
        return d

    def payload(self):
        state = np.zeros_like(self.odds, dtype=np.uint8)
        state[self.odds < -0.5] = 1
        state[self.odds > 0.6] = 2
        return {
            "cells": state.tolist(),
            "resolution": self.res,
            "origin": self.origin,
            "size": self.size,
        }
