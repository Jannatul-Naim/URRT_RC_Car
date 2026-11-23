"""Eco Rover laptop server: hosts the dashboard and connects to the three boards.

    pip install -r requirements.txt
    python server.py              then open http://localhost:8080

Who does what:
  ESP32-S3     motors, sonars, pan servo, ping       <- WebSocket ws://S3:81
  ESP32-CAM    video only                            <- MJPEG http://CAM:81/stream
  ESP32 DevKit microphone + speaker                  <- UDP port 5005
  this laptop  dashboard + image processing (QR, motion, later hazmat)

Rulebook interface (Software & Autonomous rulebook, Table 6):
  GET /api/status  GET /api/sensors  POST /api/mode  POST /api/drive
  POST /api/estop  POST /api/reset   GET /api/camera/stream   WS /ws

E-stop: ON while the S3, the camera and the audio board are all offline. It stays
on after a board comes back until RESET. The S3 also latches it on its own when
the motor battery is unplugged or the E-stop switch is OFF.
"""
import asyncio
import json
import socket
import time
from datetime import datetime
from pathlib import Path

from aiohttp import WSMsgType, web, ClientSession

import config
from vision import Vision, now_iso

WEB_DIR = Path(__file__).parent / "web"
S3_SILENT_S = 1.5                # no message from the S3 for this long = offline
CAPTURE_DIR = (Path(__file__).parent / config.CAPTURE_DIR).resolve()

# ------------------------------------------------------------------ shared state
state = {
    "mode": "teleop",
    "mode_changed": now_iso(),
    "estop": False,
    "estop_time": None,
    "s3_online": False,
    "audio_online": False,
    "battery_v": -1.0,
    "sonar_cm": [-1, -1, -1, -1],
    "sonar_time": None,
    "servo_deg": 90,
    "heading_deg": -1,        # compass, from the S3 telemetry
    "ping": {"ip": None, "ok": None, "rtt_ms": None, "time": None},
    "listening": 0,              # browsers listening to the rover microphone
    "talking": False,
    "audio_time": None,
    "tracking": False,           # pan servo follows motion
}
browsers = set()                 # open dashboard WebSockets
listeners = set()                # the ones that want the rover microphone
s3_ws = None                     # WebSocket to the S3 (None when offline)
vision = Vision(config.CAM_STREAM_URL)
last_pinged_ip = None
last_mic_packet = 0.0
last_talk_packet = 0.0
drive_owner = None               # the page that is driving right now
drive_owner_time = 0.0


# ------------------------------------------------------------------ helpers
async def send_s3(line):
    """Send one text command to the S3. Returns False if it is offline."""
    if s3_ws is None or s3_ws.closed:
        return False
    try:
        await s3_ws.send_str(line)
        return True
    except Exception:
        return False


async def broadcast_text(text):
    for ws in list(browsers):
        try:
            await ws.send_str(text)
        except Exception:
            browsers.discard(ws)


def battery_percent():
    v = state["battery_v"]
    if v < 0:
        return None
    pct = (v - config.BATTERY_EMPTY_V) / (config.BATTERY_FULL_V - config.BATTERY_EMPTY_V) * 100
    return max(0, min(100, round(pct)))


def status_json():
    return {
        "connected": {"s3": state["s3_online"], "camera": vision.online, "audio": state["audio_online"]},
        "battery": {"volts": state["battery_v"] if state["battery_v"] >= 0 else None,
                    "percent": battery_percent()},
        "mode": state["mode"],
        "mode_changed": state["mode_changed"],
        "estop": state["estop"],
        "estop_time": state["estop_time"],
    }


def sensors_json():
    with vision.lock:
        motion_for = round(time.time() - vision.motion_since, 1) if vision.motion_since else 0
        return {
            "sonar": {"front_center_cm": state["sonar_cm"][0], "front_left_cm": state["sonar_cm"][1],
                      "front_right_cm": state["sonar_cm"][2], "rear_cm": state["sonar_cm"][3],
                      "time": state["sonar_time"]},
            "qr": {"text": vision.qr_text, "ip": vision.qr_ip, "time": vision.qr_time},
            "ping": state["ping"],
            "motion": {"detected": vision.motion and vision.online, "seconds": motion_for if vision.online else 0, "tracking": state["tracking"],
                       "time": vision.motion_time},
            "audio": {"online": state["audio_online"], "listening": state["listening"] > 0,
                      "talking": state["talking"], "time": state["audio_time"]},
            "hazmat": {"label": vision.hazmat, "time": vision.hazmat_time},
            "camera": {"online": vision.online, "fps": round(vision.fps, 1)},
            "servo_deg": state["servo_deg"],
            "heading_deg": state["heading_deg"] if state["heading_deg"] >= 0 else None,
        }


def any_board_online():
    return state["s3_online"] or vision.online or state["audio_online"]


async def set_estop(on):
    """Returns False when a RESET is refused."""
    if on:
        state["estop"] = True
        state["estop_time"] = now_iso()
        await send_s3("CMD:ESTOP")
        return True
    if not any_board_online():
        print("[E-STOP] reset refused: no board online (S3, camera, audio)")
        await broadcast_text("L,warn,[E-STOP] Reset refused: no board online (S3, camera, audio)")
        return False
    if state["s3_online"]:
        await send_s3("CMD:RESET")     # the S3 confirms in its telemetry
    else:
        state["estop"] = False         # no S3 to hold it; it gets CMD:ESTOP again if needed
    return True


async def board_watch():
    """E-stop rule: with the S3, the camera and the audio board all offline the
    system is in E-stop. It stays latched when a board comes back, until RESET."""
    while True:
        if not any_board_online() and not state["estop"]:
            state["estop"] = True
            state["estop_time"] = now_iso()
            print("[E-STOP] no board online (S3, camera, audio): E-stop ON until RESET")
            await broadcast_text("L,warn,[E-STOP] No board online: E-stop ON until RESET")
        await asyncio.sleep(0.2)


# ------------------------------------------------------------------ S3 link
def handle_s3_line(line):
    """Read one message from the S3 and update the state."""
    if line.startswith("T,"):
        p = line.split(",")
        if len(p) >= 11:
            state["battery_v"] = float(p[5])
            estop = p[6] == "1"
            if estop and not state["estop"]:
                state["estop_time"] = now_iso()
            state["estop"] = estop
            state["sonar_cm"] = [int(x) for x in p[7:11]]
            state["sonar_time"] = now_iso()
        if len(p) >= 12:
            state["servo_deg"] = int(p[11])
        if len(p) >= 13:
            state["heading_deg"] = int(p[12])   # compass, -1 = no reading
    elif line.startswith("G,"):
        _, ip, ok, rtt = line.split(",")
        state["ping"] = {"ip": ip, "ok": ok == "1", "rtt_ms": int(rtt), "time": now_iso()}


async def s3_link():
    global s3_ws
    url = f"ws://{config.S3_HOST}:{config.S3_WS_PORT}/"
    last_error = None
    async with ClientSession() as session:
        while True:
            received = 0
            connected_at = None
            try:
                # compress=0: the S3 does not support compression, so do not offer it
                async with session.ws_connect(url, autoping=False, compress=0) as ws:
                    s3_ws = ws
                    state["s3_online"] = True
                    connected_at = time.time()
                    last_error = None
                    print(f"[S3] connected to {url}")
                    if state["estop"]:
                        await ws.send_str("CMD:ESTOP")   # E-stop pressed while the S3 was away
                    await ws.send_str("T")
                    while True:
                        # The S3 sends telemetry 25 times a second. If it loses power it
                        # never closes the socket, so silence means it is gone.
                        try:
                            msg = await asyncio.wait_for(ws.receive(), timeout=S3_SILENT_S)
                        except asyncio.TimeoutError:
                            print(f"[S3] no data for {S3_SILENT_S} s, marking it offline")
                            break
                        if msg.type in (WSMsgType.CLOSE, WSMsgType.CLOSING, WSMsgType.CLOSED,
                                        WSMsgType.ERROR):
                            break
                        if msg.type != WSMsgType.TEXT:
                            print(f"[S3] non-text message: {msg.type!r} {msg.data!r}")
                            continue
                        received += 1
                        try:
                            handle_s3_line(msg.data)
                        except Exception as e:      # one bad line must not drop the link
                            print(f"[S3] could not read {msg.data!r}: {e!r}")
                        if msg.data.startswith(("T,", "L,")):
                            await broadcast_text(msg.data)   # the dashboard reads these as before
                    s3_ws = None                         # offline now, before the slow close
                    state["s3_online"] = False
                    try:
                        await asyncio.wait_for(ws.close(), timeout=1.0)   # a dead S3 never answers
                    except Exception:
                        pass
                    print(f"[S3] link closed after {time.time() - connected_at:.1f} s, "
                          f"{received} messages, close code {ws.close_code}, error {ws.exception()!r}")
            except Exception as e:
                if connected_at is not None:
                    print(f"[S3] link dropped after {time.time() - connected_at:.1f} s, "
                          f"{received} messages: {e!r}")
                elif repr(e) != last_error:          # print each new connect error once
                    print(f"[S3] cannot connect to {url}: {e!r}")
                last_error = repr(e)
            s3_ws = None
            state["s3_online"] = False
            await asyncio.sleep(1.0)


# ------------------------------------------------------------------ audio board link (UDP)
class AudioProtocol(asyncio.DatagramProtocol):
    def __init__(self):
        self.transport = None
        self.board_addr = None

    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data, addr):
        global last_mic_packet
        if data[:1] == b"M":
            last_mic_packet = time.time()
            self.board_addr = addr[0]
            pcm = data[1:]
            for ws in list(listeners):
                asyncio.ensure_future(ws.send_bytes(pcm))

    def send_speaker(self, pcm):
        if not self.transport or not self.board_addr:
            return
        for i in range(0, len(pcm), 1024):                 # max 512 samples per packet
            self.transport.sendto(b"A" + pcm[i:i + 1024], (self.board_addr, config.AUDIO_UDP_PORT))


audio = AudioProtocol()


async def audio_link():
    loop = asyncio.get_running_loop()
    await loop.create_datagram_endpoint(lambda: audio, local_addr=("0.0.0.0", config.AUDIO_UDP_PORT))
    while True:
        try:   # look the board up again every second, its IP may change
            info = await loop.getaddrinfo(config.AUDIO_HOST, config.AUDIO_UDP_PORT, family=socket.AF_INET)
            audio.board_addr = info[0][4][0]
            audio.transport.sendto(b"H", (audio.board_addr, config.AUDIO_UDP_PORT))
        except Exception:
            pass
        online = time.time() - last_mic_packet < 2.0
        if online != state["audio_online"]:
            print(f"[AUDIO] board {'online' if online else 'offline'}")
        state["audio_online"] = online
        state["talking"] = time.time() - last_talk_packet < 0.5
        await asyncio.sleep(1.0)


# ------------------------------------------------------------------ 10 Hz loop: push sensors, auto-ping, tracking
async def sensor_loop():
    global last_pinged_ip
    last_track = 0.0
    while True:
        # QR -> ping. Done by the S3 (rule 2.1.3), sent once per new address,
        # with no operator input, so it also counts in autonomous mode.
        qr_ip = vision.qr_ip
        if qr_ip and qr_ip != last_pinged_ip and state["s3_online"]:
            if await send_s3(f"P,{qr_ip}"):
                last_pinged_ip = qr_ip
                state["ping"] = {"ip": qr_ip, "ok": None, "rtt_ms": None, "time": now_iso()}

        # Motion tracking: turn the pan servo toward the moving target
        if state["tracking"] and vision.motion and time.time() - last_track > 0.1:
            error = vision.motion_x - 0.5
            if abs(error) > config.TRACK_DEADBAND:
                target = max(0, min(180, round(state["servo_deg"] - error * config.TRACK_GAIN)))
                await send_s3(f"S,{target}")
                last_track = time.time()

        msg = json.dumps({"type": "sensors", "status": status_json(), "sensors": sensors_json()})
        await broadcast_text(msg)
        await asyncio.sleep(0.1)


# ------------------------------------------------------------------ browser WebSocket /ws
async def handle_browser_text(ws, text):
    global last_pinged_ip
    if text in ("T", "PING"):
        await ws.send_str("PONG")
    elif text.startswith("D,") or text.startswith("S,"):
        # Controls are locked in autonomous mode and while the E-stop is on
        if state["mode"] == "teleop" and not state["estop"]:
            if text.startswith("S,") and state["tracking"]:
                return                                   # tracking owns the servo
            if text.startswith("D,") and not drive_allowed(ws, text):
                return                                   # another page is driving
            await send_s3(text)
    elif text == "CMD:ESTOP":
        await set_estop(True)
    elif text == "CMD:RESET":
        await set_estop(False)
    elif text.startswith("{"):
        cmd = json.loads(text)
        kind = cmd.get("type")
        if kind == "mode":
            await set_mode(cmd.get("mode"))
        elif kind == "listen":
            if cmd.get("on"):
                listeners.add(ws)
            else:
                listeners.discard(ws)
            state["listening"] = len(listeners)
            state["audio_time"] = now_iso()
        elif kind == "track":
            state["tracking"] = bool(cmd.get("on"))
        elif kind == "ping" and vision.qr_ip:
            last_pinged_ip = None                        # ping the last QR address again


def drive_allowed(ws, text):
    """Both pages (main and traversal) send drive packets all the time, mostly
    "stop" (0,0). The page that last sent a real movement owns the drive; while
    it owns it (and for 0.5 s after), other pages' stop packets are ignored."""
    global drive_owner, drive_owner_time
    parts = text.split(",")
    moving = len(parts) >= 3 and (parts[1].strip() not in ("0", "") or parts[2].strip() not in ("0", ""))
    now = time.time()
    if moving:
        drive_owner, drive_owner_time = ws, now
        return True
    if ws is drive_owner:
        drive_owner_time = now
        return True
    return drive_owner is None or drive_owner.closed or now - drive_owner_time > 0.5


async def ws_handler(request):
    global last_talk_packet
    ws = web.WebSocketResponse(heartbeat=5)
    await ws.prepare(request)
    browsers.add(ws)
    await ws.send_str(json.dumps({"type": "sensors", "status": status_json(), "sensors": sensors_json()}))
    try:
        async for msg in ws:
            if msg.type == WSMsgType.TEXT:
                await handle_browser_text(ws, msg.data)
            elif msg.type == WSMsgType.BINARY:           # operator's voice -> rover speaker
                last_talk_packet = time.time()
                state["audio_time"] = now_iso()
                audio.send_speaker(msg.data)
    finally:
        browsers.discard(ws)
        listeners.discard(ws)
        state["listening"] = len(listeners)
    return ws


# ------------------------------------------------------------------ REST API
async def set_mode(mode):
    if mode not in ("teleop", "auto"):
        return False
    if mode != state["mode"]:
        state["mode"] = mode
        state["mode_changed"] = now_iso()
        await send_s3("D,0,0,0")                         # stop when the mode changes
        print(f"[MODE] {mode}")
    return True


async def api_status(request):
    return web.json_response(status_json())


async def api_sensors(request):
    return web.json_response(sensors_json())


async def api_mode(request):
    body = await request.json()
    if not await set_mode(body.get("mode")):
        return web.json_response({"ok": False, "error": 'mode must be "teleop" or "auto"'}, status=400)
    return web.json_response({"ok": True, "mode": state["mode"], "mode_changed": state["mode_changed"]})


async def api_drive(request):
    """Body: {"left": -100..100, "right": -100..100, "pwm": 0..100, "servo": 0..180 (optional)}.
    One command lasts 450 ms (the S3 watchdog), so repeat it to keep moving."""
    if state["mode"] != "teleop":
        return web.json_response({"ok": False, "error": "controls are locked in autonomous mode"}, status=409)
    if state["estop"]:
        return web.json_response({"ok": False, "error": "E-stop is engaged"}, status=409)
    body = await request.json()
    left, right = int(body.get("left", 0)), int(body.get("right", 0))
    pwm = int(body.get("pwm", 75))
    ok = await send_s3(f"D,{left},{right},{pwm}")
    if "servo" in body:
        ok = await send_s3(f"S,{int(body['servo'])}") and ok
    if not ok:
        return web.json_response({"ok": False, "error": "S3 is offline"}, status=503)
    return web.json_response({"ok": True})


async def api_estop(request):
    await set_estop(True)
    return web.json_response({"ok": True, "estop": True})


async def api_reset(request):
    if not await set_estop(False):
        return web.json_response({"ok": False, "error": "no board online (S3, camera, audio)"}, status=409)
    return web.json_response({"ok": True, "note": "check /api/status: the S3 refuses reset while the E-stop switch is OFF"})


async def api_camera_stream(request):
    """MJPEG stream of the processed camera image (QR and motion boxes drawn on it)."""
    resp = web.StreamResponse(headers={"Content-Type": "multipart/x-mixed-replace; boundary=frame",
                                       "Cache-Control": "no-cache"})
    await resp.prepare(request)
    last_id = -1
    try:
        while True:
            with vision.lock:
                frame_id, jpeg = vision.frame_id, vision.jpeg
            if jpeg and frame_id != last_id:
                last_id = frame_id
                await resp.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
            await asyncio.sleep(0.02)
    except (ConnectionResetError, asyncio.CancelledError):
        pass
    return resp


async def api_capture(request):
    """Save the newest camera picture (no boxes drawn) for labelling later."""
    with vision.lock:
        jpeg = vision.raw_jpeg
    if not jpeg or not vision.online:
        return web.json_response({"ok": False, "error": "no camera picture yet"}, status=503)
    CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
    name = datetime.now().strftime("cap_%Y%m%d_%H%M%S_%f")[:-3] + ".jpg"
    (CAPTURE_DIR / name).write_bytes(jpeg)
    count = len(list(CAPTURE_DIR.glob("*.jpg")))
    print(f"[CAPTURE] {name} ({count} in {CAPTURE_DIR})")
    return web.json_response({"ok": True, "file": name, "count": count, "folder": str(CAPTURE_DIR)})


async def capture_file(request):
    """Lets the traversal page show a thumbnail of the last capture."""
    path = (CAPTURE_DIR / request.match_info["name"]).resolve()
    if path.parent != CAPTURE_DIR or not path.is_file():
        raise web.HTTPNotFound()
    return web.FileResponse(path)


async def index(request):
    return web.FileResponse(WEB_DIR / "index.html")


async def traversal(request):
    return web.FileResponse(WEB_DIR / "traversal.html")


async def start_background(app):
    vision.start()
    app["tasks"] = [asyncio.create_task(s3_link()), asyncio.create_task(audio_link()),
                    asyncio.create_task(sensor_loop()), asyncio.create_task(board_watch())]


def main():
    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_get("/traversal", traversal)
    app.router.add_post("/api/capture", api_capture)
    app.router.add_get("/captures/{name}", capture_file)
    app.router.add_get("/ws", ws_handler)
    app.router.add_get("/api/status", api_status)
    app.router.add_get("/api/sensors", api_sensors)
    app.router.add_post("/api/mode", api_mode)
    app.router.add_post("/api/drive", api_drive)
    app.router.add_post("/api/estop", api_estop)
    app.router.add_post("/api/reset", api_reset)
    app.router.add_get("/api/camera/stream", api_camera_stream)
    app.on_startup.append(start_background)
    print(f"Dashboard: http://localhost:{config.WEB_PORT}   Traversal: http://localhost:{config.WEB_PORT}/traversal")
    print(f"S3: {config.S3_HOST}   CAM: {config.CAM_HOST}   AUDIO: {config.AUDIO_HOST}")
    # shutdown_timeout: the camera stream never ends by itself, so without this
    # Ctrl+C / kill waited the default 60 s for it before the server exited.
    web.run_app(app, port=config.WEB_PORT, print=None, shutdown_timeout=2)


if __name__ == "__main__":
    main()
