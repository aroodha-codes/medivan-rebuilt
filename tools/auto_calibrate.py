"""Automatic view selection for MediVan camera calibration; never imports motor drivers.
Place in MediVan/tools. Move a printed 10x7-square board by hand while robot stays still.
"""

import argparse
import json
import math
from pathlib import Path
import secrets
import shutil
import sys
import threading
import time

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from medivan.common import ROOT, atomic_json, load_config


def descriptor(corners, size, pattern):
    p = corners.reshape(-1, 2) / np.array(size)
    cols, rows = pattern
    # Canonical orientation avoids counting detector corner-order flips as new views.
    if tuple(p[-1]) < tuple(p[0]):
        p = p[::-1]
    return p[[0, cols - 1, (rows - 1) * cols, rows * cols - 1]].reshape(-1)


def coverage(corners_list, size):
    cells = set()
    scales = []
    for corners in corners_list:
        p = corners.reshape(-1, 2) / np.array(size)
        center = p.mean(axis=0)
        cells.add(tuple(np.clip((center * 3).astype(int), 0, 2)))
        scales.append(float(np.linalg.norm(np.ptp(p, axis=0))))
    ratio = max(scales) / min(scales) if scales and min(scales) > 0 else 0
    return len(cells), ratio


def fit_calibration(points, size, pattern, square_mm):
    cols, rows = pattern
    obj = np.zeros((cols * rows, 3), np.float32)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_mm / 1000
    rms, K, dist, rotations, translations = cv2.calibrateCamera(
        [obj.copy() for _ in points], points, size, None, None
    )
    normals = [cv2.Rodrigues(r)[0][:, 2] for r in rotations]
    spread = max(
        math.acos(float(np.clip(np.dot(a, b), -1, 1))) for a in normals for b in normals
    )
    if spread < math.radians(15):
        raise ValueError(
            "Insufficient tilt variation; tilt the board left/right and up/down"
        )
    errors = []
    for observed, rot, trans in zip(points, rotations, translations):
        predicted, _ = cv2.projectPoints(obj, rot, trans, K, dist)
        errors.append(
            float(np.sqrt(np.mean(np.sum((predicted - observed) ** 2, axis=2))))
        )
    if (
        not np.isfinite(K).all()
        or not np.isfinite(dist).all()
        or not math.isfinite(rms)
    ):
        raise ValueError("Calibration produced nonfinite values")
    if (
        not 0.2 * size[0] < K[0, 0] < 5 * size[0]
        or not 0.2 * size[1] < K[1, 1] < 5 * size[1]
    ):
        raise ValueError(
            "Implausible focal length; include more tilted checkerboard views"
        )
    if not 0 < K[0, 2] < size[0] or not 0 < K[1, 2] < size[1]:
        raise ValueError(
            "Principal point outside image; capture better-distributed views"
        )
    return {
        "matrix": K.tolist(),
        "distortion": dist.tolist(),
        "size": list(size),
        "rms_pixels": float(rms),
        "views": len(points),
        "per_view_rms": errors,
        "pattern_inner_corners": list(pattern),
        "square_mm": square_mm,
    }


def save_result(path, result, session):
    """Preserve existing calibration until a complete replacement passes validation."""
    if path.exists():
        shutil.copy2(path, session / "previous-camera.json")
    atomic_json(path, result)


HTML = """<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>MediVan camera calibration</title><style>
body{background:#101b27;color:#e2edf6;font:17px system-ui;max-width:900px;margin:25px auto;padding:20px}
h1{font-size:26px}img{width:100%;max-width:640px;border:1px solid #526779}#message{color:#7ee6c8;min-height:50px}
small{color:#b5c7d6}progress{width:100%;height:24px}
</style><h1>Automatic camera calibration</h1>
<p>Keep the robot still. Move and tilt the printed checkerboard. Pause briefly at each position.</p>
<progress id="progress" max="20" value="0"></progress><p id="message">Connecting…</p>
<img id="image" alt="Pi camera preview with detected checkerboard"><p id="details"></p>
<small>This page controls no motors. Stop the tool with Ctrl+C in SSH. Keep this page URL private.</small>
<script>
const token=new URLSearchParams(location.search).get('token'),headers={'X-Calibration-Token':token};let previous=null,busy=false;
async function poll(){if(busy)return;busy=true;try{
const response=await fetch('/status',{headers});if(!response.ok)throw Error('Authentication failed');const s=await response.json();
document.getElementById('message').textContent=s.message;let p=document.getElementById('progress');p.max=s.target;p.value=s.accepted;
document.getElementById('details').textContent=`Accepted: ${s.accepted}/${s.target} · Image regions: ${s.regions}/4 · Size variation: ${s.scale_ratio.toFixed(2)}× (need 1.25×)`;
const r=await fetch('/frame',{headers});if(r.status===200){const u=URL.createObjectURL(await r.blob());document.getElementById('image').src=u;if(previous)URL.revokeObjectURL(previous);previous=u}
}catch(e){document.getElementById('message').textContent='Preview stopped. Check the SSH terminal for the result.'}finally{busy=false}}
setInterval(poll,600);poll();</script></html>"""


def start_preview(shared, lock, host, port, token):
    from flask import Flask, Response, jsonify, request
    from werkzeug.serving import make_server
    import logging

    logging.getLogger("werkzeug").setLevel(logging.ERROR)
    app = Flask(__name__, static_folder=None)

    @app.before_request
    def auth():
        given = (
            request.args.get("token", "")
            if request.path == "/"
            else request.headers.get("X-Calibration-Token", "")
        )
        if not secrets.compare_digest(given, token):
            return Response("Unauthorized", status=401)

    @app.after_request
    def protect(response):
        response.headers["Cache-Control"] = "no-store"
        response.headers["Referrer-Policy"] = "no-referrer"
        response.headers["X-Frame-Options"] = "DENY"
        return response

    @app.get("/")
    def home():
        return Response(HTML, mimetype="text/html")

    @app.get("/status")
    def status():
        with lock:
            return jsonify({k: v for k, v in shared.items() if k != "jpeg"})

    @app.get("/frame")
    def frame():
        with lock:
            jpg = shared.get("jpeg")
        return Response(jpg, mimetype="image/jpeg") if jpg else Response(status=204)

    server = make_server(host, port, app, threaded=True)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--square-mm", type=float, required=True)
    parser.add_argument("--columns", type=int, default=9)
    parser.add_argument("--rows", type=int, default=6)
    parser.add_argument("--views", type=int, default=20)
    parser.add_argument("--timeout", type=float, default=900)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8081)
    args = parser.parse_args()
    if (
        not 1 <= args.square_mm <= 100
        or not 3 <= args.rows <= 15
        or not 3 <= args.columns <= 15
    ):
        parser.error("Check board dimensions and measured square size (1–100 mm)")
    if not 15 <= args.views <= 50 or not 30 <= args.timeout <= 3600:
        parser.error("Use 15–50 views and a timeout of 30–3600 seconds")
    # Uses the same interprocess lock as MediVan; stop diagnostics first.
    import fcntl
    from picamera2 import Picamera2

    ownership = open("/tmp/medivan-gpio.lock", "w")
    try:
        fcntl.flock(ownership, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit(
            "Stop the MediVan server with Ctrl+C before camera calibration."
        )
    config = load_config()
    size = (config["camera"]["width"], config["camera"]["height"])
    pattern = (args.columns, args.rows)
    session = (
        ROOT
        / "data"
        / (
            "auto-calibration-"
            + time.strftime("%Y%m%d-%H%M%S")
            + "-"
            + secrets.token_hex(2)
        )
    )
    session.mkdir(parents=True)
    lock = threading.Lock()
    shared = {
        "message": "Show the full checkerboard",
        "accepted": 0,
        "target": args.views,
        "regions": 0,
        "scale_ratio": 0.0,
        "jpeg": None,
    }
    token = secrets.token_urlsafe(18)
    server = None
    camera = None
    points = []
    descs = []
    files = []
    previous = None
    stable_since = None
    last_capture = -100
    started = time.monotonic()
    next_fit = args.views
    try:
        server = start_preview(shared, lock, args.host, args.port, token)
        print(f"Preview: http://YOUR_PI_IP:{args.port}/?token={token}", flush=True)
        print(
            f"Pattern: {args.columns} × {args.rows} INNER corners. No motors are initialized.",
            flush=True,
        )
        camera = Picamera2()
        camera.configure(
            camera.create_video_configuration(
                main={"size": size, "format": "RGB888"}, controls={"FrameRate": 10}
            )
        )
        camera.start()
        while time.monotonic() - started < args.timeout:
            frame = camera.capture_array()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(
                gray,
                pattern,
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH
                | cv2.CALIB_CB_NORMALIZE_IMAGE
                | cv2.CALIB_CB_FAST_CHECK,
            )
            message = "Show the complete checkerboard, including its outer squares."
            now = time.monotonic()
            if found:
                corners = cv2.cornerSubPix(
                    gray,
                    corners,
                    (7, 7),
                    (-1, -1),
                    (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001),
                )
                d = descriptor(corners, size, pattern)
                p = corners.reshape(-1, 2)
                x, y = np.floor(p.min(axis=0)).astype(int)
                x2, y2 = np.ceil(p.max(axis=0)).astype(int)
                sharpness = cv2.Laplacian(
                    gray[
                        max(0, y) : min(size[1], y2 + 1),
                        max(0, x) : min(size[0], x2 + 1),
                    ],
                    cv2.CV_64F,
                ).var()
                stable = previous is not None and np.linalg.norm(d - previous) < 0.015
                if not stable:
                    stable_since = now
                previous = d
                distinct = all(np.linalg.norm(d - old) > 0.09 for old in descs)
                if x < 8 or y < 8 or x2 > size[0] - 8 or y2 > size[1] - 8:
                    message = "Move the board away from the image edge."
                elif min(x2 - x, y2 - y) < 70:
                    message = "Move the board closer: it is too small."
                elif sharpness < 45:
                    message = (
                        "Image is blurred. Improve light and hold the board still."
                    )
                elif not stable or stable_since is None or now - stable_since < 0.45:
                    message = "Hold this position briefly…"
                elif not distinct:
                    message = "Already captured this view. Move to another region, change distance, or tilt."
                elif now - last_capture < 1.2:
                    message = "Captured. Move to a new view."
                else:
                    file = session / f"view-{len(points)+1:03d}.jpg"
                    if not cv2.imwrite(str(file), frame):
                        raise RuntimeError("Could not save capture")
                    points.append(corners.copy())
                    descs.append(d.copy())
                    files.append(file.name)
                    last_capture = now
                    print(f"Accepted view {len(points)}", flush=True)
                    message = "Captured. Move and moderately tilt the board for the next view."
                cv2.drawChessboardCorners(frame, pattern, corners, True)
            else:
                previous = None
                stable_since = None
            regions, ratio = coverage(points, size)
            if len(points) >= args.views:
                if regions < 4:
                    message = "Move the board centre to more image regions (left/right/top/bottom)."
                elif ratio < 1.25:
                    message = (
                        "Change board distance while keeping every square visible."
                    )
                elif len(points) >= next_fit:
                    print("Calculating calibration…", flush=True)
                    try:
                        result = fit_calibration(points, size, pattern, args.square_mm)
                        if (
                            result["rms_pixels"] > 1.0
                            or max(result["per_view_rms"]) > 2.0
                        ):
                            raise ValueError(
                                f"Reprojection error too high: RMS {result['rms_pixels']:.3f} px"
                            )
                        result["captures"] = str(session.relative_to(ROOT))
                        result["coverage_regions"] = regions
                        result["scale_ratio"] = ratio
                        path = ROOT / config["camera"]["calibration"]
                        save_result(path, result, session)
                        print(
                            f"SUCCESS: {len(points)} views; RMS {result['rms_pixels']:.3f} pixels. Saved {path}",
                            flush=True,
                        )
                        print(
                            "Camera intrinsics saved. Validate real distances; floor and motor calibration are separate.",
                            flush=True,
                        )
                        return
                    except ValueError as e:
                        print(
                            str(e) + "; capture five more varied, tilted views.",
                            flush=True,
                        )
                        message = str(e) + ". Add varied tilted views."
                        next_fit = len(points) + 5
            if len(points) >= 60:
                raise RuntimeError(
                    "60 views collected without a valid fit. Existing calibration unchanged; inspect board and captures."
                )
            ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            with lock:
                shared.update(
                    message=message,
                    accepted=len(points),
                    regions=regions,
                    scale_ratio=ratio,
                )
                if ok:
                    shared["jpeg"] = jpg.tobytes()
            time.sleep(0.12)
        raise RuntimeError(
            "Capture timed out. Existing calibration unchanged; use preview guidance and retry."
        )
    finally:
        try:
            atomic_json(
                session / "capture-summary.json",
                {
                    "files": files,
                    "views": len(points),
                    "size": list(size),
                    "pattern": list(pattern),
                    "square_mm": args.square_mm,
                },
            )
        except OSError as error:
            print(f"Could not save capture summary: {error}", file=sys.stderr)
        if camera:
            try:
                camera.stop()
            finally:
                camera.close()
        if server:
            server.shutdown()
            server.server_close()
        ownership.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print(
            "\nStopped. Previous calibration is preserved unless SUCCESS was printed."
        )
    except Exception as error:
        print(f"Calibration stopped: {error}", file=sys.stderr)
        sys.exit(1)
