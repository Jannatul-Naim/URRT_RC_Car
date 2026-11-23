"""Image processing on the laptop.

A background thread reads the ESP32-CAM's MJPEG stream, then for every frame:
  1. decodes QR codes (OpenCV's built-in QR detector)
  2. detects motion (difference between frames)
  3. classifies hazmat signs with the optional exported CNN
and keeps the newest frame, with boxes drawn on it, for the dashboard.
"""
import json
from collections import deque
import re
import threading
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np

IPV4 = re.compile(r"\b(?:(?:25[0-5]|2[0-4]\d|1?\d?\d)\.){3}(?:25[0-5]|2[0-4]\d|1?\d?\d)\b")

HAZMAT_MODEL = Path(__file__).parent / "models" / "hazmat.onnx"
HAZMAT_METADATA = Path(__file__).parent / "models" / "hazmat.json"
HAZMAT_CONFIDENCE = 0.80
# A class folder with one of these names means "no sign": it is trained like the
# others but never reported on the dashboard.
NO_SIGN_CLASSES = {"none", "background", "no_sign"}
# Motion gaps shorter than this still count as the same motion (5 s task).
MOTION_GAP_S = 1.0
# Compare each frame with one about this much older (not the previous frame):
# at 20 fps two neighbouring frames barely differ for a slow hand.
MOTION_REF_AGE_S = 0.25
MOTION_MIN_AREA = 60           # px at 160x120: smaller patches are noise
MOTION_SMOOTH = 0.6            # 0..1, how fast the box follows (lower = smoother, laggier)
MOTION_MAX_FRACTION = 0.5      # changes bigger than this share of the frame are ignored
MOTION_BOX_HOLD_S = 0.3        # keep drawing the box this long after motion stops

_hazmat = None                 # loaded model + settings, or None
_hazmat_load_attempted = False


def now_iso():
    return time.strftime("%H:%M:%S")


def _load_hazmat():
    """Load the ONNX model once. Uses onnxruntime: OpenCV's ONNX reader gives
    wrong results for this MobileNetV3 export (it always said "Explosive")."""
    if not HAZMAT_MODEL.is_file() or not HAZMAT_METADATA.is_file():
        print("[HAZMAT] No model in laptop/models/, hazmat detection off")
        return None
    try:
        import onnxruntime as ort
    except ImportError:
        print("[HAZMAT] onnxruntime is not installed (pip install onnxruntime), hazmat detection off")
        return None
    try:
        meta = json.loads(HAZMAT_METADATA.read_text(encoding="utf-8"))
        session = ort.InferenceSession(str(HAZMAT_MODEL), providers=["CPUExecutionProvider"])
        model = {
            "session": session,
            "input": session.get_inputs()[0].name,
            "classes": meta["classes"],
            "size": int(meta.get("image_size", 128)),
            "threshold": float(meta.get("confidence_threshold", HAZMAT_CONFIDENCE)),
            "mean": np.asarray(meta.get("normalization_mean", (0.485, 0.456, 0.406)), np.float32),
            "std": np.asarray(meta.get("normalization_std", (0.229, 0.224, 0.225)), np.float32),
        }
        print(f"[HAZMAT] Model loaded: {', '.join(model['classes'])}")
        return model
    except Exception as exc:
        print(f"[HAZMAT] Model could not be loaded: {exc}")
        return None


def hazmat_probabilities(frame):
    """Return (classes, probabilities) for a BGR frame, or None without a model.

    The frame is prepared exactly like the training images: grey, 320x240,
    then resized to the model's input size and normalised.
    """
    global _hazmat, _hazmat_load_attempted
    if not _hazmat_load_attempted:
        _hazmat_load_attempted = True
        _hazmat = _load_hazmat()
    if _hazmat is None:
        return None
    m = _hazmat
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.resize(gray, (320, 240), interpolation=cv2.INTER_AREA)
    gray = cv2.resize(gray, (m["size"], m["size"]), interpolation=cv2.INTER_AREA)
    image = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB).astype(np.float32) / 255.0
    image = (image - m["mean"]) / m["std"]
    blob = np.transpose(image, (2, 0, 1))[None, ...].copy()
    scores = m["session"].run(None, {m["input"]: blob})[0].reshape(-1)
    scores = np.exp(scores - scores.max())
    return m["classes"], scores / scores.sum()


def classify_hazmat(frame):
    """Return the hazard label for a camera frame, or None.

    None when there is no model, the best class is a "no sign" class, or the
    model is less sure than its confidence threshold.
    """
    result = hazmat_probabilities(frame)
    if result is None:
        return None
    classes, probabilities = result
    best = int(np.argmax(probabilities))
    label = classes[best]
    if label.lower() in NO_SIGN_CLASSES or probabilities[best] < _hazmat["threshold"]:
        return None
    return label


class Vision(threading.Thread):
    def __init__(self, stream_url):
        super().__init__(daemon=True)
        self.url = stream_url
        self.lock = threading.Lock()

        self.online = False
        self.fps = 0.0
        self.frame_id = 0
        self.jpeg = None              # newest annotated frame, JPEG bytes
        self.raw_jpeg = None          # newest frame exactly as the camera sent it (for Capture)

        self.qr_text = None           # last decoded QR text
        self.qr_ip = None             # IPv4 address found in it
        self.qr_time = None

        self.motion = False
        self.motion_x = 0.5           # centre of the moving thing, 0 = left, 1 = right
        self.motion_since = None      # time.time() when the current motion started
        self.motion_time = None

        self.hazmat = None
        self.hazmat_time = None

        self._qr = cv2.QRCodeDetector()
        self._grays = deque()         # (time, small grey frame), the last ~MOTION_REF_AGE_S
        self._last_motion = 0.0       # time.time() of the last frame with motion
        self._box = None              # smoothed motion box (x0, y0, x1, y1) at 160x120

    # ---------------------------------------------------------------- stream
    def run(self):
        while True:
            try:
                stream = urllib.request.urlopen(self.url, timeout=5)
                self.online = True
                buf = b""
                frames, t0 = 0, time.time()
                while True:
                    chunk = stream.read(8192)
                    if not chunk:
                        break
                    buf += chunk
                    start = buf.find(b"\xff\xd8")             # JPEG start
                    end = buf.find(b"\xff\xd9", start + 2)    # JPEG end
                    if start != -1 and end != -1:
                        jpg, buf = buf[start:end + 2], buf[end + 2:]
                        self.process(jpg)
                        frames += 1
                        if time.time() - t0 >= 1.0:
                            self.fps = frames / (time.time() - t0)
                            frames, t0 = 0, time.time()
                    if len(buf) > 2_000_000:                  # junk, start over
                        buf = b""
            except Exception:
                pass
            self.online = False
            self.fps = 0.0
            self._reset_motion()     # a frozen last frame must not keep "motion" alive
            time.sleep(1.0)

    def _reset_motion(self):
        with self.lock:
            self.motion = False
            self.motion_since = None
        self._grays.clear()
        self._box = None
        self._last_motion = 0.0

    # ---------------------------------------------------------------- per frame
    def process(self, jpg):
        frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        h, w = frame.shape[:2]
        with self.lock:
            self.raw_jpeg = jpg
        # Classify the unannotated camera image before QR/motion boxes are drawn.
        label = classify_hazmat(frame)

        # 1. QR code
        try:
            text, points, _ = self._qr.detectAndDecode(frame)
        except cv2.error:
            text, points = "", None
        if points is not None:
            cv2.polylines(frame, [points.astype(int).reshape(-1, 1, 2)], True, (0, 255, 0), 3)
        if text:
            with self.lock:
                if text != self.qr_text:
                    print(f"[QR] {text}")
                self.qr_text = text
                found = IPV4.search(text)
                self.qr_ip = found.group(0) if found else None
                self.qr_time = now_iso()
            cv2.putText(frame, text[:40], (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # 2. Motion: compare with a frame ~MOTION_REF_AGE_S old, at low resolution
        small = cv2.resize(frame, (160, 120))
        gray = cv2.GaussianBlur(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY), (7, 7), 0)
        t_now = time.time()
        self._grays.append((t_now, gray))
        while len(self._grays) > 2 and t_now - self._grays[1][0] >= MOTION_REF_AGE_S:
            self._grays.popleft()
        ref_t, ref = self._grays[0]
        moving = False
        if ref is not gray and t_now - ref_t > 0.03:
            diff = cv2.absdiff(ref, gray)
            _, mask = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)
            mask = cv2.dilate(mask, None, iterations=2)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            parts = [c for c in contours if cv2.contourArea(c) > MOTION_MIN_AREA]
            area = sum(cv2.contourArea(c) for c in parts)
            # Too small = noise. More than MOTION_MAX_FRACTION of the picture = the camera
            # itself moved (pan servo, rover vibration) or the light changed: not a target.
            if parts and 120 < area < MOTION_MAX_FRACTION * 160 * 120:
                # One box around every moving part (a hand is often several blobs)
                x, y, bw, bh = cv2.boundingRect(np.vstack(parts))
                box = (x, y, x + bw, y + bh)
                if self._box is None or t_now - self._last_motion > MOTION_GAP_S:
                    self._box = box
                else:
                    a = MOTION_SMOOTH
                    self._box = tuple(o + a * (n - o) for o, n in zip(self._box, box))
                moving = True
                with self.lock:
                    self.motion_x = (self._box[0] + self._box[2]) / 2 / 160
        # Keep drawing the last box through short gaps so it does not flicker
        if self._box is not None and (moving or t_now - self._last_motion <= MOTION_BOX_HOLD_S):
            sx, sy = w / 160, h / 120
            x0, y0, x1, y1 = self._box
            cv2.rectangle(frame, (int(x0 * sx), int(y0 * sy)), (int(x1 * sx), int(y1 * sy)),
                          (0, 165, 255), 2)
        # The 5 s motion task counts one continuous event: gaps up to
        # MOTION_GAP_S (a hand pausing, a frame where it barely moved) do not
        # restart the timer. self.motion stays the raw per-frame result, so
        # tracking only moves the servo toward a target seen in this frame.
        t = time.time()
        with self.lock:
            if moving:
                if self.motion_since is None or t - self._last_motion > MOTION_GAP_S:
                    self.motion_since = t
                self._last_motion = t
            elif self.motion_since is not None and t - self._last_motion > MOTION_GAP_S:
                self.motion_since = None
            self.motion = moving
            if moving:
                self.motion_time = now_iso()

        # 3. Publish the current hazard label and overlay detections.
        with self.lock:
            if label != self.hazmat:
                self.hazmat = label
                if label is not None:
                    self.hazmat_time = now_iso()
        if label:
            cv2.putText(frame, f"HAZMAT: {label[:32]}", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        ok, out = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ok:
            with self.lock:
                self.jpeg = out.tobytes()
                self.frame_id += 1
