# Implementation references and supplied assets

Primary documentation consulted for adapter/API details:

- Raspberry Pi Picamera2 manual: https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf — capture configuration and RGB888 buffer channel ordering for OpenCV.
- GPIO Zero output devices: https://gpiozero.readthedocs.io/en/stable/api_output.html — output device lifecycle and PWM interface.
- Texas Instruments INA219: https://www.ti.com/lit/gpn/ina219 — calibration/current registers and shunt scaling.
- Texas Instruments ADS1115: https://www.ti.com/product/ADS1115 — ADC capabilities and linked device documentation; TI configuration discussion https://e2e.ti.com/support/data-converters-group/data-converters/f/data-converters-forum/917941/ads1115-adc-intermittently-returning-zero.
- OpenCV ArUco detection: https://docs.opencv.org/4.7.0/d5/dae/tutorial_aruco_detection.html — marker detection and pose estimation.

The ONNX file in `assets/yolov8n.onnx` is copied unchanged from the user-supplied MediVan ZIP. Its ownership and redistribution terms were not established by this rebuild. Retain the model's upstream license when distributing the project. No new trained model or dataset is claimed.

User inputs used: MediVan source ZIP, Team 6 PPT, six robot photos, and confirmed dimensions/wiring in conversation. SnapClass was excluded as unrelated. Earlier PPT performance figures were not reused as validation results for this implementation.
