"""Select a verified empty floor patch from an image at the configured resolution."""

import argparse, sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import cv2, numpy as np
from medivan.common import ROOT, atomic_json, load_config

p = argparse.ArgumentParser()
p.add_argument("image")
p.add_argument(
    "--roi", nargs=4, type=int, required=True, metavar=("X", "Y", "WIDTH", "HEIGHT")
)
a = p.parse_args()
im = cv2.imread(a.image)
c = load_config()["camera"]
if im is None or im.shape[:2] != (c["height"], c["width"]):
    raise SystemExit("Image missing or wrong resolution")
x, y, w, h = a.roi
if min(x, y) < 0 or min(w, h) < 5 or x + w > im.shape[1] or y + h > im.shape[0]:
    raise SystemExit("Invalid ROI")
lab = cv2.cvtColor(im, cv2.COLOR_BGR2LAB)[y : y + h, x : x + w].reshape(-1, 3)
atomic_json(
    ROOT / c["floor_profile"], {"lab": np.median(lab, axis=0).tolist(), "roi": a.roi}
)
print("Floor colour profile saved. Validate obstacle boundaries in diagnostics.")
