"""Save calibration frames from Picamera2. Does not initialize motor GPIO."""

import argparse, sys, time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from medivan.common import load_config, ROOT
import cv2
from picamera2 import Picamera2

p = argparse.ArgumentParser()
p.add_argument("--count", type=int, default=20)
p.add_argument("--folder", default="data/checkerboard")
a = p.parse_args()
c = load_config()["camera"]
out = ROOT / a.folder
out.mkdir(parents=True, exist_ok=True)
cam = Picamera2()
cam.configure(
    cam.create_still_configuration(
        main={"size": (c["width"], c["height"]), "format": "RGB888"}
    )
)
cam.start()
try:
    for i in range(a.count):
        input(f"Position checkerboard ({i+1}/{a.count}); Enter to capture: ")
        frame = cam.capture_array()
        target = out / f"{time.time_ns()}.jpg"
        cv2.imwrite(str(target), frame)
        print(target)
finally:
    cam.stop()
    cam.close()
