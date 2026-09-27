"""Explicit, bounded manual calibration. No autonomous navigation is involved."""

import argparse, sys, time, fcntl
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from medivan.common import load_config
from medivan.hardware import Motors

p = argparse.ArgumentParser()
p.add_argument("--seconds", type=float, default=1)
p.add_argument("--speed", type=float, default=0.03)
p.add_argument("--turn", type=float, default=0)
a = p.parse_args()
if not 0.1 <= a.seconds <= 3 or not 0 <= a.speed <= 0.06 or abs(a.turn) > 0.4:
    raise SystemExit("Bounds: 0.1–3 seconds, 0–0.06 m/s, turn ≤0.4 rad/s")
f = open("/tmp/medivan-gpio.lock", "w")
try:
    fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
except BlockingIOError:
    raise SystemExit("Stop the MediVan server first")
input(
    "First test with wheels raised. For distance calibration use a clear floor and supervisor. Enter to run: "
)
m = Motors(load_config())
start = time.monotonic()
try:
    while time.monotonic() - start < a.seconds:
        m.drive(a.speed, a.turn)
        time.sleep(0.04)
finally:
    m.close()
print(
    "Stopped. Record actual travel/turn. See docs/CALIBRATION.md for configuration updates."
)
