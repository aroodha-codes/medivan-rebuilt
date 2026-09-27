# Raspberry Pi setup

The original ZIP says Raspberry Pi OS 64-bit. It does not contain a reliable installed OS/version inventory. Run `python3 tools/diagnose.py` to capture the actual environment.

## Install

On a current 64-bit Raspberry Pi OS installation, use its packaged camera and GPIO libraries:

```bash
sudo apt update
sudo apt install python3-venv python3-picamera2 python3-opencv python3-numpy python3-gpiozero python3-lgpio python3-smbus i2c-tools
sudo raspi-config
```

Enable I2C under Interface Options, then reboot if prompted. CSI cameras use Picamera2/libcamera; do not enable the deprecated legacy camera stack.

In the new project folder:

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements-pi.txt
python tools/diagnose.py
python -c "import cv2; print(cv2.__version__); print(hasattr(cv2, 'aruco'))"
```

This build uses `cv2.aruco.ArucoDetector` when available and the legacy `cv2.aruco.detectMarkers` interface otherwise. If your package lacks the ArUco module entirely, stop and report the printed OS/OpenCV versions before replacing system camera dependencies. The version actually tested is recorded in VALIDATION.md; compatibility with your Pi package still requires a startup check. Do not install both `opencv-python` and `opencv-contrib-python` in one environment. The Windows requirements deliberately use only the contrib headless distribution.

Check that your user has access to `/dev/gpiochip*`, `/dev/i2c-1`, and the camera. Use the Pi OS `gpio`, `i2c`, and `video` groups where appropriate; do not run the whole application as root to hide permission errors.

## Camera and stationary diagnostics

```bash
python run.py --mode diagnostics --host 0.0.0.0
```

This mode never allocates motor GPIO. Open `http://<PI-LAN-IP>:8080` on the same network and enter the token printed in the terminal. The app stays functional before floor calibration, but floor confidence will be zero. Sensor and camera failures are explicit.

Keep the robot still during startup: the MPU6050 gyro bias is measured over 100 readings. The IMU must be mounted flat with its Z axis upward. Sensor orientation beyond that simple mounting is not transformed by this build.

## Hardware operation

Complete `CALIBRATION.md`, then:

```bash
python run.py --mode hardware --host 0.0.0.0
```

There is no auto-start motion. You must connect, confirm the robot placement in the dashboard, and arm. Hardware mode checks camera calibration at startup and further installation flags when arming. The terminal token changes per launch unless `MEDIVAN_TOKEN` is set to a secret of at least 16 characters. Never commit that secret.

An interprocess file lock prevents simultaneous MediVan hardware/calibration instances on Linux. Stop the previous project and any other GPIO program yourself; they do not participate in this lock.

The motor command watchdog is software, not a hardware emergency stop. Process/kernel failure or GPIO electrical faults can defeat software safeguards. The existing physical power switch remains necessary during supervised tests. Ctrl+C requests shutdown and de-energizes the motor outputs in the normal shutdown path.

## Troubleshooting

- **Camera calibration required:** run the camera calibration procedure, not a flag bypass.
- **Hardware gate message:** complete the corresponding measurement and then update configuration.
- **No safe route:** the destination or full vehicle corridor has not been observed free; map more space or choose a nearer destination.
- **Floor confidence low:** check lighting, profile, mounting, floor visibility and actual camera image. Do not lower the threshold blindly.
- **Travel limit reached:** stop and establish the true pose against your measurements. This limit is not an encoder substitute.
- **ADS/INA unavailable:** confirm actual I2C address and wiring; no fabricated battery values are substituted.
- **Runtime timing deadline:** reduce CPU load or disable supplemental YOLO explicitly while diagnosing. Capture and inference run off the motor loop, but excessive load still matters.
