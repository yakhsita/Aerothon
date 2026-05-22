# ============================================================
#   QR-based Drone Navigation — Gazebo + ArduPilot SITL
#   Fixes: axis remapping, NED origin offset, position check
# ============================================================

from gz.transport13 import Node
import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image
from pyzbar.pyzbar import decode
from pymavlink import mavutil
import time
import threading

# ---------------- MAVLINK CONNECTION ----------------
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Connected to drone")

# ---------------- GLOBALS ----------------
topic = "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"
node = Node()

visited     = set()
moving      = False
mission_ready = False
landing     = False

# Drone's Gazebo world position at arm time (update if drone doesn't start at origin)
home_gz_x = 0.0
home_gz_y = 0.0
FLIGHT_ALTITUDE = 4.0   # metres

# ---------------- COORDINATE CONVERSION ----------------
def gazebo_to_ned(gz_x, gz_y, altitude):
    """
    Gazebo world frame  →  MAV_FRAME_LOCAL_NED
      Gazebo X (East)   →  NED Y (East)
      Gazebo Y (North)  →  NED X (North)
      Altitude          →  NED Z  (negative = up)
    Origin is shifted by home position recorded at arm time.
    """
    ned_x = gz_y - home_gz_y
    ned_y = gz_x - home_gz_x
    ned_z = -altitude
    return ned_x, ned_y, ned_z

# ---------------- ARM + TAKEOFF ----------------
def arm_and_takeoff(altitude):
    global mission_ready

    print("Setting GUIDED mode...")
    master.set_mode_apm("GUIDED")
    time.sleep(2)

    print("Arming motors...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Motors armed!")
    time.sleep(3)

    print(f"Taking off to {altitude} m ...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        altitude
    )

    # Wait until altitude is reached
    print("Waiting to reach target altitude...")
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
        if msg:
            current_alt = msg.relative_alt / 1000.0  # mm to metres
            print(f"  Altitude: {current_alt:.2f} m")
            if current_alt >= altitude * 0.90:        # 90 % threshold
                break
        time.sleep(0.5)

    print("Takeoff complete — Mission Ready")
    mission_ready = True

# ---------------- GOTO ----------------
def goto_position(ned_x, ned_y, ned_z):
    print(f"  Sending NED target → X={ned_x:.2f}  Y={ned_y:.2f}  Z={ned_z:.2f}")
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,   # position only (ignore vel/accel/yaw)
        ned_x, ned_y, ned_z,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )

# ---------------- WAIT UNTIL POSITION REACHED ----------------
def wait_until_reached(ned_x, ned_y, tolerance=1.0, timeout=40):
    """
    Polls LOCAL_POSITION_NED until within `tolerance` metres
    or `timeout` seconds elapse.
    Returns True if reached, False if timed out.
    """
    print(f"  Waiting to reach target (tolerance={tolerance} m, timeout={timeout} s)...")
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=2)
        if msg:
            dist = ((msg.x - ned_x) ** 2 + (msg.y - ned_y) ** 2) ** 0.5
            print(f"  Distance to target: {dist:.2f} m")
            if dist <= tolerance:
                print("  Target reached!")
                return True
        time.sleep(0.5)

    print("  Timeout — continuing anyway")
    return False

# ---------------- LAND ----------------
def land():
    global landing
    landing = True
    print("Landing command sent.")
    master.set_mode_apm("LAND")

# ---------------- CAMERA CALLBACK ----------------
def callback(msg):
    global visited, moving, mission_ready, landing

    if not mission_ready or moving or landing:
        return

    # Decode image
    img   = np.frombuffer(msg.data, dtype=np.uint8)
    frame = img.reshape((msg.height, msg.width, 3)).copy()

    qr_codes = decode(frame)

    for qr in qr_codes:
        x, y, w, h = qr.rect
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        text = qr.data.decode("utf-8")
        cv2.putText(frame, text, (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if text in visited:
            continue

        print(f"QR Detected: {text}")
        visited.add(text)

        # ---------- LAND ----------
        if text.strip().upper() == "LAND":
            land()
            return

        # ---------- COORDINATE ----------
        try:
            parts    = text.split(",")
            gz_x     = float(parts[0].strip())
            gz_y     = float(parts[1].strip())

            ned_x, ned_y, ned_z = gazebo_to_ned(gz_x, gz_y, FLIGHT_ALTITUDE)
            print(f"Gazebo ({gz_x}, {gz_y})  →  NED ({ned_x:.2f}, {ned_y:.2f}, {ned_z:.2f})")

            # Run navigation in a separate thread so camera keeps processing
            def navigate():
                global moving
                moving = True
                goto_position(ned_x, ned_y, ned_z)
                wait_until_reached(ned_x, ned_y)
                moving = False
                print("Ready for next QR\n")

            threading.Thread(target=navigate, daemon=True).start()

        except Exception as e:
            print(f"  Invalid QR format — {e}")

    cv2.imshow("Drone Camera", frame)
    cv2.waitKey(1)

# ---------------- DEBUG: print drone NED position once ----------------
def print_home_ned():
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=5)
    if msg:
        print(f"[DEBUG] Drone NED at arm time: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}")
    else:
        print("[DEBUG] Could not read LOCAL_POSITION_NED")

# ---------------- MAIN ----------------
print_home_ned()                         # sanity check before takeoff

node.subscribe(Image, topic, callback)   # start camera subscription

arm_and_takeoff(FLIGHT_ALTITUDE)         # arm → takeoff → set mission_ready

print("Mission Started — scanning for QR codes...\n")

while True:
    pass



