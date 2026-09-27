import sys, argparse
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import cv2, numpy as np
from medivan.common import ROOT, load_config

p = argparse.ArgumentParser()
p.add_argument("--pixels", type=int, default=800)
a = p.parse_args()
c = load_config()["dock"]
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
generate = getattr(cv2.aruco, "generateImageMarker", None) or cv2.aruco.drawMarker
im = generate(d, c["marker_id"], a.pixels)
im = cv2.copyMakeBorder(im, 100, 100, 100, 100, cv2.BORDER_CONSTANT, value=255)
cv2.imwrite(str(ROOT / "data/dock-marker.png"), im)
print(
    f"Print the BLACK marker square at {c['marker_size_m']*100:.1f} cm per side, excluding white margin."
)
