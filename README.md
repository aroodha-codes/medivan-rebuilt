# MediVan — rebuilt robot software

A local Python application for your Raspberry Pi 4 hospital-delivery prototype. The web dashboard and simulator use the same mission controller as the physical robot adapter.

**This is a new implementation, not a patch to the supplied project.** Keep your original ZIP as a backup. Unzip this project into a separate folder. Do not run the old and new motor programs together.

## What is implemented

- Explicit simulation, Pi hardware, and motion-free diagnostics modes.
- Real Picamera2 capture, MPU6050 angular velocity and tilt, ADS1115 pack voltage, INA219 current.
- Independent sensor threads, software motor watchdog, stale-data checks, latched emergency stop, tilt and voltage limits.
- Camera calibration, floor-colour segmentation, flat-floor range projection, and supplied YOLOv8n ONNX inference.
- Log-odds occupancy mapping, exploration toward reachable frontier regions, footprint clearance, A* replanning.
- Named destinations or map-click coordinates, delivery queue, collection confirmation, return to dock staging.
- Dock-marker detection, front alignment, 180-degree turn, and a bounded supervised reverse phase that stops on sustained charging current.
- Atomic map persistence, explicit pose establishment after loading, local dashboard and access-token authentication.

## Limits that matter

Physical operation has **not** been tested on your robot. Physical autonomous motion is disabled until the calibration checklist is complete. The project does not claim hospital deployment readiness.

The camera is at 11 cm and points straight ahead. Floor segmentation is a heuristic and cannot reliably detect every obstacle, transparent surface, stair edge, or floor-coloured object. YOLO supplements that heuristic; the supplied COCO model is not a custom hospital-equipment detector.

With no wheel encoders and no navigation markers, translation is estimated from calibrated motor commands. The gyro supplies angular rate, not drift-free absolute heading. Mapping does not include reliable visual loop closure or global relocalization; this is **camera-derived occupancy mapping with dead reckoning**, not a validated full visual SLAM system. A configurable travel limit bounds each unverified hardware session, initially 2 m. Increasing it requires measured evidence, not simply clearing an error.

The front camera loses sight of a dock behind the robot. Reverse docking therefore has a supervised, calibrated final approach and is disabled initially. Copper plates and magnets do not identify a charger or its safety circuitry. The software never switches a charging supply, guarantees battery balancing, or promises cutoff at 95%. It displays voltage/current and estimates percentage from voltage. See `docs/DOCKING.md`.

## Confirmed configuration

| Item | Value |
|---|---|
| Controller | Raspberry Pi 4 |
| Camera | Existing Pi camera; height 0.11 m; pitch 0° |
| Wheels | Diameter 0.05 m; centre spacing 0.10 m |
| Left L298N | IN1 BCM5, IN2 BCM6, ENA BCM12 |
| Right L298N | IN3 BCM13, IN4 BCM19, ENB BCM18 |
| I2C defaults from original ZIP | MPU6050 0x68, ADS1115 0x48, INA219 0x40 |
| Rear docking | Copper contacts with magnets; ArUco previously untested |

The PWM implementation uses independent gpiozero/lgpio PWM outputs. It does not claim that BCM12 and BCM18 provide distinct hardware PWM channels.

Robot width/length, camera forward offset from wheel axle, motor speeds, ADC divider values, current sign, and dock offsets need physical confirmation. Defaults are labelled in the calibration documentation. The ZIP specifies 64-bit Pi OS but cannot tell us the exact OS installed on your Pi.

## Start on Windows — simulation

Use Python 3.11 or newer. In PowerShell, inside this folder:

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python run.py --mode sim
```

If a newer compatible Python is your installed version, use `py -m venv .venv` instead. Open **http://127.0.0.1:8080**. Copy the access token printed in the terminal into the dashboard.

1. Click **Arm motion**, then **Explore**.
2. Allow the initial 360-degree scan. Free floor appears on the map.
3. Click **Finish mapping** once the intended destination and route are visible.
4. Click free space, enter a name such as `Ward 1`, then **Save location**.
5. Click **Add delivery**, then **Start next delivery**.
6. At arrival, click **Confirm collection**. The robot returns to the staging point.
7. **Save map** persists occupancy, destinations, and staging pose.
8. Use **Emergency stop** or Escape to stop. **Reset / disarm** clears the active mission and queue.

Saved maps live in `data/map.json`. After loading, disarm/reset and use **Confirm placement** with the actual map coordinates and yaw before arming. A saved map does not establish the robot's present location automatically.

## Start on Raspberry Pi

See `docs/PI_SETUP.md`. Start with `python tools/diagnose.py`, then motion-free diagnostics and calibration. Normal hardware mode deliberately refuses to start without camera and floor calibration files.

## Tests

```bash
python -m pip install -r requirements-dev.txt
python -m pytest tests -q
```

See `docs/VALIDATION.md` for results from this build and checks that require your hardware.

## Project structure

- `medivan/controller.py` — mission states and interlocks
- `medivan/hardware.py` — Pi adapters and motor timeout
- `medivan/vision.py` — camera geometry, ArUco, ONNX detector
- `medivan/mapping.py` — occupancy grid, clearance and planning
- `medivan/simulation.py` — independent simulated world
- `medivan/server.py` — local API, authentication, loop runner
- `web/` — responsive dashboard
- `config/robot.json` — all installation-specific settings
- `tools/` — calibration, marker generation and diagnostics
- `tests/` — failure handling, planning, API and mission tests

No Docker, cloud account, or database server is required. The dashboard uses a LAN HTTP server intended for a trusted lab network; do not expose it to the public internet.
