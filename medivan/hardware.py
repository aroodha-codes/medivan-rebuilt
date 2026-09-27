"""Pi adapters. No simulated fallback. Independent sensor acquisition and motor timeout."""

import math, time, threading
import numpy as np
from .common import Observation
from .vision import Vision


def signed16(v):
    return v - 65536 if v & 0x8000 else v


class Motors:
    def __init__(self, cfg):
        from gpiozero import DigitalOutputDevice, PWMOutputDevice
        from gpiozero.pins.lgpio import LGPIOFactory

        self.cfg = cfg
        self.lock = threading.RLock()
        self.pins = []
        self.last = 0
        self.running = True
        self.factory = LGPIOFactory()
        self.channels = []
        self.command = (0.0, 0.0)
        self.inhibited = False
        try:
            for forward, backward, enable in [
                cfg["motor"]["left"],
                cfg["motor"]["right"],
            ]:
                a = DigitalOutputDevice(forward, pin_factory=self.factory)
                self.pins.append(a)
                b = DigitalOutputDevice(backward, pin_factory=self.factory)
                self.pins.append(b)
                e = PWMOutputDevice(
                    enable,
                    frequency=cfg["motor"]["pwm_hz"],
                    initial_value=0,
                    pin_factory=self.factory,
                )
                self.pins.append(e)
                self.channels.append((a, b, e))
        except Exception:
            for p in self.pins:
                p.close()
            self.factory.close()
            raise
        self.thread = threading.Thread(target=self._watchdog, daemon=True)
        self.thread.start()

    def _zero(self):
        for a, b, e in self.channels:
            e.value = 0
            a.off()
            b.off()
        self.command = (0.0, 0.0)

    def drive(self, v, w):
        with self.lock:
            if self.inhibited:
                self._zero()
                return
            c = self.cfg["motor"]
            track = self.cfg["geometry"]["track_width_m"]
            for speed, side, (a, b, e) in zip(
                [v - w * track / 2, v + w * track / 2], ["left", "right"], self.channels
            ):
                speed *= c[side + "_sign"]
                e.value = 0
                a.off()
                b.off()
                if abs(speed) > 1e-5:
                    (a if speed > 0 else b).on()
                    fraction = min(1, abs(speed) / c["full_speed_" + side + "_m_s"])
                    e.value = c["deadband"] + (1 - c["deadband"]) * fraction
            self.last = time.monotonic()
            self.command = (v, w)

    def stop(self):
        with self.lock:
            self._zero()

    def latch_stop(self):
        with self.lock:
            self.inhibited = True
            self._zero()

    def rearm(self):
        with self.lock:
            self._zero()
            self.inhibited = False

    def _watchdog(self):
        while self.running:
            with self.lock:
                if time.monotonic() - self.last > self.cfg["motor"]["watchdog_s"]:
                    self._zero()
            time.sleep(0.03)

    def close(self):
        self.running = False
        self.stop()
        self.thread.join(1)
        for p in self.pins:
            p.close()
        self.factory.close()


class Hardware:
    hardware = True

    def __init__(self, cfg, diagnostics=False):
        from smbus2 import SMBus
        from picamera2 import Picamera2

        self.cfg = cfg
        self.lock = threading.RLock()
        self.running = False
        self.frame = None
        self.latest = Observation(
            time.monotonic(), confidence=0, faults=["Sensors starting"]
        )
        self.motors = None
        self.camera = None
        self.bus = None
        try:
            self.vision = Vision(cfg, require_calibration=not diagnostics)
            self.bus = SMBus(cfg["sensors"]["i2c_bus"])
            s = cfg["sensors"]
            self.s = s
            self.bus.write_byte_data(s["imu_address"], 0x6B, 0)
            self.bus.write_byte_data(s["imu_address"], 0x1B, 0)
            self.bus.write_byte_data(s["imu_address"], 0x1C, 0)
            self.bus.write_byte_data(s["imu_address"], 0x1A, 3)
            who = self.bus.read_byte_data(s["imu_address"], 0x75)
            if who & 0x7E != 0x68:
                raise RuntimeError("MPU6050 identity check failed")
            self._write16(s["ina_address"], 0, 0x399F)
            # 100 uA / bit; 0.1-ohm default shunt => calibration 4096.
            self.current_lsb = 0.0001
            self.ina_cal = int(0.04096 / (self.current_lsb * s["shunt_ohm"])) & 0xFFFE
            if not 0 < self.ina_cal < 65536:
                raise ValueError("Invalid INA219 shunt calibration")
            self._write16(s["ina_address"], 5, self.ina_cal)
            self.bias = 0.0
            rates = []
            for _ in range(100):
                rates.append(signed16(self._read16(s["imu_address"], 0x47)) / 131.0)
                time.sleep(0.005)
            if np.std(rates) > 1:
                raise RuntimeError("Keep robot stationary during gyro calibration")
            self.bias = float(np.mean(rates))
            self.camera = Picamera2()
            c = cfg["camera"]
            self.camera.configure(
                self.camera.create_video_configuration(
                    main={"size": (c["width"], c["height"]), "format": "RGB888"},
                    controls={"FrameRate": 15},
                    buffer_count=4,
                )
            )
            self.camera.start()
            # No motor GPIO allocation in diagnostic mode.
            if not diagnostics:
                self.motors = Motors(cfg)
            self.running = True
            self.threads = [
                threading.Thread(target=self._sensors, daemon=True),
                threading.Thread(target=self._frames, daemon=True),
            ]
            for t in self.threads:
                t.start()
        except Exception:
            self.close()
            raise

    def _read16(self, address, reg):
        b = self.bus.read_i2c_block_data(address, reg, 2)
        return (b[0] << 8) | b[1]

    def _write16(self, address, reg, v):
        self.bus.write_i2c_block_data(address, reg, [(v >> 8) & 255, v & 255])

    def _sensors(self):
        battery_at = 0
        while self.running:
            try:
                now = time.monotonic()
                s = self.s
                raw = self.bus.read_i2c_block_data(s["imu_address"], 0x3B, 14)
                values = [signed16((raw[i] << 8) | raw[i + 1]) for i in range(0, 14, 2)]
                ax, ay, az = [v / 16384.0 for v in values[:3]]
                tilt = math.degrees(math.atan2(math.hypot(ax, ay), az))
                rate = math.radians(values[6] / 131.0 - self.bias)
                with self.lock:
                    self.latest.yaw_rate = rate
                    self.latest.tilt_deg = tilt
                    self.latest.stamp = now
                if now - battery_at > 0.3:
                    self._write16(
                        s["ads_address"], 1, 0xC383
                    )  # A0, ±4.096V, single shot, 128SPS
                    deadline = time.monotonic() + 0.05
                    while not self._read16(s["ads_address"], 1) & 0x8000:
                        if time.monotonic() > deadline:
                            raise RuntimeError("ADS1115 conversion timeout")
                        time.sleep(0.002)
                    voltage = (
                        signed16(self._read16(s["ads_address"], 0))
                        * 4.096
                        / 32768
                        * s["divider_ratio"]
                        * s["voltage_correction"]
                    )
                    if self._read16(s["ina_address"], 5) != self.ina_cal:
                        raise RuntimeError("INA219 calibration lost")
                    if self._read16(s["ina_address"], 2) & 1:
                        raise RuntimeError("INA219 overflow")
                    current = (
                        signed16(self._read16(s["ina_address"], 4))
                        * self.current_lsb
                        * s["current_sign"]
                    )
                    if not 4 < voltage < 9:
                        raise RuntimeError("Implausible 2S battery voltage")
                    with self.lock:
                        self.latest.voltage = voltage
                        self.latest.current = current
                        self.latest.battery_stamp = now
                    battery_at = now
            except Exception as e:
                with self.lock:
                    self.latest.faults = [f"Sensor error: {e}"]
                self.stop()
                return
            time.sleep(0.01)

    def _frames(self):
        import cv2

        while self.running:
            try:
                # Timestamp capture before processing, so inference lag counts toward staleness.
                frame = self.camera.capture_array()
                stamp = time.monotonic()
                rays, confidence, marker, objects, view = self.vision.process(frame)
                ok, jpg = cv2.imencode(".jpg", view, [cv2.IMWRITE_JPEG_QUALITY, 75])
                with self.lock:
                    self.latest.rays = rays
                    self.latest.confidence = confidence
                    self.latest.marker = marker
                    self.latest.detections = objects
                    self.latest.frame_stamp = stamp
                    self.frame = jpg.tobytes() if ok else None
            except Exception as e:
                with self.lock:
                    self.latest.faults = [f"Camera error: {e}"]
                self.stop()
                return

    def sample(self, dt, now):
        from copy import deepcopy

        with self.lock:
            o = deepcopy(self.latest)
            if o.frame_stamp and o.battery_stamp and o.faults == ["Sensors starting"]:
                o.faults = []
            return o

    def drive(self, v, w):
        if self.motors:
            self.motors.drive(v, w)
        elif v or w:
            raise RuntimeError("Diagnostics mode cannot move motors")

    def stop(self):
        if self.motors:
            self.motors.stop()

    def latch_stop(self):
        if self.motors:
            self.motors.latch_stop()

    def rearm(self):
        if self.motors:
            self.motors.rearm()

    def jpeg(self):
        with self.lock:
            return self.frame

    def close(self):
        self.running = False
        if self.motors:
            self.motors.close()
            self.motors = None
        if self.camera:
            try:
                self.camera.stop()
                self.camera.close()
            except Exception:
                pass
        for t in getattr(self, "threads", []):
            t.join(1)
        if self.bus:
            try:
                self.bus.close()
            except Exception:
                pass
