# Laptop settings. Change the three addresses to the IPs your boards print
# on the Serial Monitor at boot (or keep the .local names if mDNS works on
# your laptop: it does on Linux with avahi and on recent Windows/macOS).

S3_HOST = "rover.local"        # ESP32-S3 main board (mDNS name, follows DHCP changes), or its IP
CAM_HOST = "rover-cam.local"     # ESP32-CAM (mDNS name from Rover_CAM.ino), or its IP
AUDIO_HOST = "rover-audio.local" # ESP32 DevKit audio board (mDNS name), or its IP

WEB_PORT = 8080                   # open http://localhost:8080 on the laptop

# Capture button (traversal page): raw camera photos are saved here, to be
# labelled later by moving each one into a class folder under data/hazmat/.
CAPTURE_DIR = "../ImageProsseingTraining/data/unlabeled"

S3_WS_PORT = 81                   # the S3's WebSocket port (Rover_S3.ino WS_PORT)
CAM_STREAM_URL = f"http://{CAM_HOST}:81/stream"
AUDIO_UDP_PORT = 5005             # must match UDP_PORT in Rover_Audio.ino

# Battery (3S LiPo on the motor bus). Only used if the S3 measures it
# (USE_BUS_SENSE true in Rover_S3.ino).
BATTERY_FULL_V = 12.6
BATTERY_EMPTY_V = 9.9

# Motion tracking: how hard the pan servo follows a moving target.
# If the camera turns AWAY from the target, change the sign of TRACK_GAIN.
TRACK_GAIN = 25.0                 # degrees per unit of error (error = -0.5 .. 0.5)
TRACK_DEADBAND = 0.08             # ignore small errors so the servo does not jitter
