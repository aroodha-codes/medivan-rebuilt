"""Local/LAN dashboard. No cloud service is required."""

import argparse, atexit, json, logging, os, secrets, signal, threading, time
from flask import Flask, Response, jsonify, request, send_from_directory
from .common import ROOT, load_config
from .controller import Controller
from .simulation import Simulator


class Runner:
    def __init__(self, controller):
        self.controller = controller
        self.lock = threading.RLock()
        self.halt = threading.Event()
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def loop(self):
        previous = time.monotonic()
        while not self.halt.is_set():
            now = time.monotonic()
            dt = now - previous
            previous = now
            try:
                with self.lock:
                    self.controller.tick(max(0.001, dt), now)
            except Exception as e:
                self.controller.io.stop()
                with self.lock:
                    self.controller.latch_fault("Runtime fault: " + str(e))
                logging.exception("Control loop fault")
            self.halt.wait(max(0, 0.05 - (time.monotonic() - now)))

    def stop(self):
        self.halt.set()
        self.controller.io.stop()
        if self.thread:
            self.thread.join(2)
        self.controller.io.close()


def create_app(runner, token):
    app = Flask(__name__, static_folder=None)
    app.config["MAX_CONTENT_LENGTH"] = 16384

    @app.before_request
    def authenticate():
        if request.path.startswith("/api/"):
            if not secrets.compare_digest(
                request.headers.get("X-MediVan-Token", ""), token
            ):
                return (
                    jsonify(error="Enter the access token printed in the terminal"),
                    401,
                )
            origin = request.headers.get("Origin")
            if origin and origin.rstrip("/") != request.host_url.rstrip("/"):
                return jsonify(error="Cross-origin command rejected"), 403

    @app.after_request
    def headers(resp):
        resp.headers["Cache-Control"] = "no-store"
        resp.headers["X-Content-Type-Options"] = "nosniff"
        resp.headers["Content-Security-Policy"] = (
            "default-src 'self'; img-src 'self' blob:; style-src 'self'; script-src 'self'; frame-ancestors 'none'"
        )
        return resp

    @app.get("/")
    def home():
        return send_from_directory(ROOT / "web", "index.html")

    @app.get("/static/<path:name>")
    def static_file(name):
        return send_from_directory(ROOT / "web", name)

    @app.get("/api/state")
    def state():
        with runner.lock:
            return jsonify(runner.controller.snapshot())

    @app.get("/api/map")
    def map_view():
        with runner.lock:
            return jsonify(runner.controller.grid.payload())

    @app.get("/api/camera")
    def camera():
        frame = runner.controller.io.jpeg()
        if frame is None:
            return Response(status=204)
        return Response(frame, mimetype="image/jpeg")

    @app.post("/api/action")
    def action():
        try:
            data = request.get_json()
            if not isinstance(data, dict) or not isinstance(data.get("action"), str):
                raise ValueError("Action required")
            payload = data.get("data", {})
            if not isinstance(payload, dict):
                raise ValueError("Object payload required")
            if data["action"] == "estop" and hasattr(
                runner.controller.io, "latch_stop"
            ):
                runner.controller.io.latch_stop()
            with runner.lock:
                runner.controller.action(data["action"], payload)
            return jsonify(ok=True)
        except (ValueError, KeyError, TypeError) as e:
            return jsonify(error=str(e)), 400
        except Exception as e:
            runner.controller.io.stop()
            with runner.lock:
                runner.controller.latch_fault("Command failed: " + str(e))
            logging.exception("Command failed")
            return jsonify(error="Command failed; robot stopped. Check terminal."), 500

    return app


def main():
    p = argparse.ArgumentParser(description="MediVan local robot dashboard")
    p.add_argument("--mode", choices=["sim", "hardware", "diagnostics"], default="sim")
    p.add_argument("--config")
    p.add_argument("--host")
    p.add_argument("--port", type=int)
    args = p.parse_args()
    cfg = load_config(args.config)
    from logging.handlers import RotatingFileHandler

    (ROOT / "data").mkdir(exist_ok=True)
    handler = RotatingFileHandler(
        ROOT / "data/events.log", maxBytes=2_000_000, backupCount=3
    )
    handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    logging.getLogger("medivan.events").addHandler(handler)
    logging.getLogger("medivan.events").setLevel(logging.INFO)
    # Interprocess lock prevents two services from owning the same GPIO on Linux.
    lockfile = None
    if args.mode != "sim":
        import fcntl

        lockfile = open("/tmp/medivan-gpio.lock", "w")
        try:
            fcntl.flock(lockfile, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit("Another MediVan hardware instance is running")
        from .hardware import Hardware

        io = Hardware(cfg, diagnostics=args.mode == "diagnostics")
    else:
        io = Simulator(cfg)
    ctrl = Controller(cfg, io)
    runner = Runner(ctrl)
    token = os.environ.get("MEDIVAN_TOKEN") or secrets.token_urlsafe(24)
    if len(token) < 16:
        io.close()
        raise SystemExit("MEDIVAN_TOKEN must contain at least 16 characters")
    host = args.host or cfg["server"]["host"]
    port = args.port or cfg["server"]["port"]
    print(
        f"\nMediVan {args.mode} — http://{host}:{port}\nAccess token: {token}\n",
        flush=True,
    )
    app = create_app(runner, token)
    runner.start()

    def shutdown(*_):
        runner.stop()
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    try:
        app.run(host=host, port=port, debug=False, use_reloader=False, threaded=True)
    finally:
        runner.stop()


if __name__ == "__main__":
    main()
