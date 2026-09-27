"""Calibrated flat-floor geometry; a floor mask is a heuristic, not depth sensing."""

import json, math, time
from pathlib import Path
import cv2
import numpy as np
from .common import ROOT


class Vision:
    def __init__(self, cfg, require_calibration=True):
        self.cfg = cfg
        self.count = 0
        self.detections = []
        self.net = None
        c = cfg["camera"]
        cal = ROOT / c["calibration"]
        floor = ROOT / c["floor_profile"]
        self.ready = cal.exists() and floor.exists()
        if require_calibration and not self.ready:
            raise RuntimeError("Camera calibration and floor profile are required")
        self.K = None
        self.dist = None
        self.floor = None
        if self.ready:
            d = json.loads(cal.read_text())
            self.K = np.asarray(d["matrix"], np.float64)
            self.dist = np.asarray(d["distortion"], np.float64)
            if d["size"] != [c["width"], c["height"]]:
                raise ValueError("Camera calibration resolution mismatch")
            if (
                self.K.shape != (3, 3)
                or not np.isfinite(self.K).all()
                or min(self.K[0, 0], self.K[1, 1]) <= 0
            ):
                raise ValueError("Invalid camera matrix")
            self.floor = np.asarray(json.loads(floor.read_text())["lab"], np.float32)
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = (
            cv2.aruco.ArucoDetector(self.dictionary)
            if hasattr(cv2.aruco, "ArucoDetector")
            else None
        )
        if cfg["vision"]["yolo_enabled"]:
            p = ROOT / cfg["vision"]["model"]
            if not p.exists():
                raise RuntimeError(
                    "YOLO model missing; explicitly disable or supply model"
                )
            self.net = cv2.dnn.readNetFromONNX(str(p))

    def objects(self, frame):
        if self.net is None:
            return []
        n = self.cfg["vision"]["input_size"]
        h, w = frame.shape[:2]
        scale = min(n / w, n / h)
        nw, nh = round(w * scale), round(h * scale)
        dx = (n - nw) // 2
        dy = (n - nh) // 2
        padded = np.full((n, n, 3), 114, np.uint8)
        padded[dy : dy + nh, dx : dx + nw] = cv2.resize(frame, (nw, nh))
        self.net.setInput(cv2.dnn.blobFromImage(padded, 1 / 255.0, (n, n), swapRB=True))
        out = self.net.forward().squeeze()
        if out.ndim != 2:
            raise RuntimeError("Unexpected YOLO output dimensions")
        if out.shape[0] == 84:
            out = out.T
        if out.shape[1] != 84:
            raise RuntimeError("Expected raw COCO YOLOv8 output with 84 channels")
        boxes = []
        scores = []
        classes = []
        for row in out:
            label = int(np.argmax(row[4:]))
            score = float(row[4 + label])
            if score < self.cfg["vision"]["confidence"]:
                continue
            cx, cy, bw, bh = row[:4]
            boxes.append(
                [
                    float((cx - bw / 2 - dx) / scale),
                    float((cy - bh / 2 - dy) / scale),
                    float(bw / scale),
                    float(bh / scale),
                ]
            )
            scores.append(score)
            classes.append(label)
        keep = cv2.dnn.NMSBoxes(boxes, scores, self.cfg["vision"]["confidence"], 0.45)
        return [
            {"class_id": classes[int(i)], "score": scores[int(i)], "box": boxes[int(i)]}
            for i in np.asarray(keep).reshape(-1)
        ]

    def process(self, frame):
        self.count += 1
        if (
            self.count % self.cfg["vision"]["every_n_frames"] == 1
            or self.cfg["vision"]["every_n_frames"] == 1
        ):
            self.detections = self.objects(frame)
        if not self.ready:
            return [], 0.0, None, self.detections, frame
        image = cv2.undistort(frame, self.K, self.dist)
        h, w = image.shape[:2]
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB).astype(np.float32)
        delta = np.linalg.norm(lab - self.floor, axis=2)
        mask = (delta < self.cfg["camera"]["floor_tolerance"]).astype(np.uint8)
        # Person/object boxes are excluded from free-space observations. YOLO is supplemental.
        for obj in self.detections:
            x, y, bw, bh = obj["box"]
            x1 = max(0, int(x))
            x2 = min(w, int(x + bw))
            y1 = max(0, int(y))
            y2 = min(h, int(y + bh))
            mask[y1:y2, x1:x2] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        fx, fy, cx, cy = self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2]
        pitch = math.radians(self.cfg["geometry"]["camera_pitch_deg"])
        height = self.cfg["geometry"]["camera_height_m"]
        rays = []
        valid = 0
        for u in np.linspace(w * 0.08, w * 0.92, 65).astype(int):
            if not mask[h - 4 : h, u].all():
                continue
            v = h - 4
            while v > int(h * 0.38) and mask[v, u]:
                v -= 1
            # Camera ray: right, down, forward. Positive pitch points downward.
            down = (v - cy) / fy
            right = (u - cx) / fx
            rz = math.sin(pitch) + down * math.cos(pitch)
            forward = math.cos(pitch) - down * math.sin(pitch)
            if rz <= 0.015:
                continue
            z = height * forward / rz
            left = -height * right / rz
            z += self.cfg["geometry"]["camera_forward_m"]
            dist = math.hypot(z, left)
            if dist < 0.05:
                continue
            maxr = self.cfg["camera"]["max_range_m"]
            hit = dist < maxr and v > int(h * 0.38)
            rays.append((math.atan2(left, z), min(dist, maxr), hit))
            valid += 1
        confidence = valid / 65
        corners, ids, _ = (
            self.detector.detectMarkers(image)
            if self.detector is not None
            else cv2.aruco.detectMarkers(image, self.dictionary)
        )
        marker = None
        if ids is not None:
            for corner, mid in zip(corners, ids.flatten()):
                if int(mid) != self.cfg["dock"]["marker_id"]:
                    continue
                s = self.cfg["dock"]["marker_size_m"] / 2
                object_pts = np.array(
                    [[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float32
                )
                ok, rvec, tvec = cv2.solvePnP(
                    object_pts,
                    corner.reshape(4, 2),
                    self.K,
                    np.zeros(5),
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )
                if ok and tvec[2, 0] > 0:
                    rot, _ = cv2.Rodrigues(rvec)
                    marker = {
                        "distance": float(np.linalg.norm(tvec)),
                        "bearing": float(math.atan2(-tvec[0, 0], tvec[2, 0])),
                        "normal_error": float(math.atan2(-rot[0, 2], -rot[2, 2])),
                    }
                cv2.aruco.drawDetectedMarkers(image, [corner])
        tinted = image.copy()
        tinted[mask.astype(bool)] = (
            (tinted[mask.astype(bool)] * 0.6 + np.array([35, 90, 25]))
            .clip(0, 255)
            .astype(np.uint8)
        )
        return rays, confidence, marker, self.detections, tinted
