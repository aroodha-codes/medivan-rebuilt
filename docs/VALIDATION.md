# Validation report — 27 September 2026

## Executed here

28 automated tests pass using `python -m pytest tests -q`.

Coverage includes:

- Confirmed motor pins and geometry settings.
- Disarmed motion rejection, latched emergency stop and reset semantics.
- Camera and I2C faults, stale frames, excessive tilt, critical voltage, missed timing deadline.
- Manual deadman timeout and reverse-command rejection.
- Physical calibration gates and required pose establishment after map load, including reset.
- Unknown-space rejection, obstacle inflation, route selection and atomic map roundtrip validation.
- Delivery queue and collection confirmation.
- Full simulated map scan, 0.6 m delivery, collection and return-to-staging without simulated collision.
- Full simulated marker alignment, 180-degree turn, reverse and sustained-current docking confirmation.
- Failure when reverse contact is not confirmed within the configured limit.
- GPIO adapter watchdog with mocked pin devices: stale commands de-energize outputs.
- API authentication, cross-origin rejection, malformed command handling and static assets.
- Diagnostics adapter motion rejection.
- Loading and executing the supplied ONNX model on a synthetic image.
- ArUco pose estimation from a rendered marker of known geometry (approximately 0.5 m).

Python source compilation and JavaScript syntax checks passed. Local server startup and authenticated API routes were exercised. Tests use independent simulated measurements rather than injecting the world map into the planner.

## Environment

Python 3.12.14 on Linux.
- Flask: 3.1.3
- numpy: 2.3.5
- opencv-contrib-python-headless: 4.14.0.94
- pytest: 9.1.1

## Not verified here

- Browser rendering, responsive layout and actual pointer interaction: Playwright was available but its browser binary download failed. HTTP asset tests and `node --check web/app.js` passed; these do not replace a visual browser test.
- Raspberry Pi installation and the actual installed OS package versions.
- GPIO waveforms, motor polarity, motor speed calibration, motion stopping distance and physical watchdog behaviour.
- Physical camera colour segmentation, floor-range accuracy, inference frame rate and obstacle detection reliability.
- ADS divider, INA shunt/current sign and actual charging circuit.
- Real mapping accuracy, odometry drift, destination accuracy and docking success rate.

No claims from the previous PPT such as 93–97% occupancy accuracy, 30 Hz performance, or a particular heading RMSE are attributed to this rebuild. The source is a prototype implementation with explicit calibration gates, not physical validation evidence.

## Physical acceptance record

For each supervised trial record: configuration version, surface/lighting, payload, battery voltage, requested route, measured route length, endpoint error, heading error, minimum stopping clearance, faults and outcome. Run progressively: stationary sensors → raised-wheel direction checks → short calibrated motion → perception tests → nearby delivery → docking without live contacts → verified charging trial.

The first session should use the supplied low speed and short travel cap. Failure to achieve repeatability with the confirmed sensors is a hardware/perception limitation to report, not a reason to claim success from simulation.
