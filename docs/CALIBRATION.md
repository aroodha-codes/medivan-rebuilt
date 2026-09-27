# Calibration before physical movement

The existing hardware is retained. No LiDAR, encoders, ultrasonic sensors or extra camera are assumed. All edits are in `config/robot.json`; restart the server after changes.

## 1. Measure the body and mounting

Confirmed: wheel diameter 0.05 m; track width 0.10 m; camera height 0.11 m; camera pitch 0 degrees.

Measure `robot_width_m`, `robot_length_m`, and `camera_forward_m` (lens distance forward from the wheel-axle centre). The shipped 0.20 m / 0.30 m / 0.12 m entries are **unverified placeholders**. Track width is between wheel centres, not the gap between the tyres. Confirm the measurements and set `footprint_confirmed` true. Planning uses the rectangle's half-diagonal plus clearance as a conservative turning radius.

Mount the IMU flat, Z upward. Check stationary tilt and a known left turn in diagnostics. Set `imu_flat_confirmed` only after observing sensible values. The gyro yaw sign must match positive counterclockwise rotation; unsupported mounting requires changing the adapter transformation before driving.

## 2. Camera intrinsics

At the configured 640 × 480 resolution, capture at least 15 varied views of a checkerboard with **9 × 6 inner corners**. Move and tilt the board so it covers different parts of the image. Measure a square's side.

```bash
python tools/capture.py --count 20
python tools/calibrate_camera.py 'data/checkerboard/*.jpg' --columns 9 --rows 6 --square-mm 25
```

The 25 mm value is an example: enter your measured square size. The tool refuses fewer than 15 detections or reprojection RMS above 1 pixel. It writes `data/camera.json`. Low reprojection error alone does not validate floor distances.

Capture a clear empty-floor image with the fixed camera mounting:

```bash
python tools/capture.py --count 1 --folder data/floor-capture
python tools/floor_profile.py data/floor-capture/YOUR_IMAGE.jpg --roi 200 350 240 100
```

The ROI is an example rectangle (x, y, width, height) in pixels; choose only real empty floor. It writes `data/floor.json`. Use diagnostics to compare the mask with actual floor and obstacles under expected lighting.

Place opaque obstacles at measured 0.3, 0.5 and 1.0 m distances from the axle. Verify ranges through the diagnostics API/camera and record errors. The projection assumes a flat ground plane and calibrated fixed mounting; shiny floors, shadows and changing illumination can invalidate it. Objects matching the floor colour can be missed. A person box is not a complete geometric measurement.

## 3. Motor calibration

Stop the server. Raise the drive wheels for the first direction test. BCM wiring is already set to the user-confirmed values. Verify each motor's direction; adjust `left_sign` or `right_sign` if necessary.

```bash
python tools/motor_calibration.py --seconds 1 --speed 0.03
```

For measured travel tests place the robot in a clear supervised area and use short bounded runs. Record time and actual left/right wheel travel. The output command model is `PWM = deadband + (1-deadband) × target_speed/full_speed`. Estimate each side's full speed from measurements at several PWM levels, and tune `deadband` to the lowest reliable start value. Neither commanded speed nor the 0.25 m/s default is a sensor reading.

Repeat with a bounded turn:

```bash
python tools/motor_calibration.py --seconds 1 --speed 0 --turn 0.3
```

Confirm gyro sign and measured rotation. Perform straight-line and turn trials under actual battery voltage and payload. Set `motor.calibrated` only when repeatability is acceptable. Slippage remains unobservable in translation.

## 4. Battery sensors

The ADS1115 defaults assume a 10 kΩ / 4.7 kΩ voltage divider on A0. These values came from the old code and **are not verified measurements of your installation**. Confirm the actual divider and input voltage range before relying on it. Compare reported pack voltage with a multimeter and set `divider_ratio` and `voltage_correction` appropriately.

The INA219 defaults assume a 0.1 Ω shunt. Confirm it from the board. Positive current means discharge; invert `current_sign` if necessary after checking actual wiring. Set `battery_calibrated` only after voltage and current comparison. `current_sees_charge` must remain false unless charging current actually passes through the measurement shunt and is observed with the expected negative sign.

No accurate battery state-of-charge calibration exists here; the dashboard primarily displays measured volts and amps. Low/critical limits are provisional 2S settings and must match the verified battery and its protection arrangement.

## 5. Enable short navigation trials

After the preceding checks, set `navigation.allow_physical_autonomy` true. Begin with a clear flat test area, low speed and a supervisor at the physical switch. Keep the initial 2 m cumulative movement limit. Place the robot at a known start pose, use **Confirm placement** in the dashboard, map a small area, select a nearby reachable point and measure final pose error.

Record actual distance, estimated distance, endpoint error, heading error, obstacle stopping distance, and trial outcome. Extend test distances only after measuring drift. Loading a map requires re-establishing the physical start pose; the program cannot recognize an arbitrary starting location automatically.
