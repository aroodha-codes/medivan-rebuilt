"""System inventory only; never energizes motors."""

import sys, platform, importlib.util, json
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from medivan.common import load_config

c = load_config()
print("Python:", sys.version)
print("System:", platform.platform())
p = Path("/etc/os-release")
if p.exists():
    print(p.read_text())
for name in ["cv2", "numpy", "flask", "picamera2", "smbus2", "gpiozero", "lgpio"]:
    print(name, "available" if importlib.util.find_spec(name) else "MISSING")
print("Configured BCM:", json.dumps(c["motor"]))
if platform.system() == "Linux" and Path("/dev/i2c-1").exists():
    from smbus2 import SMBus

    with SMBus(c["sensors"]["i2c_bus"]) as bus:
        for key in ["imu_address", "ads_address", "ina_address"]:
            addr = c["sensors"][key]
            try:
                bus.read_byte(addr)
                print(key, hex(addr), "responding")
            except OSError as e:
                print(key, hex(addr), "unavailable", str(e))
