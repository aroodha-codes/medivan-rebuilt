"""Collect camera frames separately, then estimate intrinsics without moving motors."""

import argparse, glob, json, sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import cv2, numpy as np
from medivan.common import atomic_json, ROOT

p = argparse.ArgumentParser()
p.add_argument("images", help="Quoted glob of checkerboard JPGs")
p.add_argument("--columns", type=int, default=9)
p.add_argument("--rows", type=int, default=6)
p.add_argument("--square-mm", type=float, required=True)
a = p.parse_args()
if a.square_mm <= 0:
    raise SystemExit("Positive square size required")
obj = np.zeros((a.rows * a.columns, 3), np.float32)
obj[:, :2] = np.mgrid[0 : a.columns, 0 : a.rows].T.reshape(-1, 2) * a.square_mm / 1000
objects = []
points = []
size = None
for file in glob.glob(a.images):
    im = cv2.imread(file)
    if im is None:
        continue
    gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    this_size = gray.shape[::-1]
    if size and this_size != size:
        raise SystemExit("All images must have the same resolution")
    size = this_size
    ok, corners = cv2.findChessboardCorners(gray, (a.columns, a.rows))
    if ok:
        points.append(
            cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001),
            )
        )
        objects.append(obj)
if len(points) < 15:
    raise SystemExit(
        f"Only {len(points)} usable images. Supply at least 15 varied checkerboard views."
    )
rms, K, dist, _, _ = cv2.calibrateCamera(objects, points, size, None, None)
if not np.isfinite(rms) or rms > 1.0:
    raise SystemExit(f"Reprojection RMS {rms:.3f} pixels is too high; improve captures")
atomic_json(
    ROOT / "data/camera.json",
    {
        "matrix": K.tolist(),
        "distortion": dist.tolist(),
        "size": list(size),
        "rms_pixels": rms,
        "views": len(points),
    },
)
print(
    f"Saved camera calibration; RMS {rms:.3f} px. Validate measured distances physically."
)
