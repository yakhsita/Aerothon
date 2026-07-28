# ============================================================
#         AEROTHON FULL MISSION LOGIC NAVIGATION
#   Fixes: change code acc. to aerothon track-1 rulebook
# ============================================================

from gz.transport13 import Node
import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image
from pyzbar.pyzbar import decode
from pymavlink import mavutil
import time
import threading
import subprocess
import queue
from enum import Enum

# ---------------- MAVLINK CONNECTION ----------------
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Connected to drone")

# ---------------- GLOBALS ----------------
topic = "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"
node = Node()

# Mission ------------
payload_dropped = False
target_delivery = None
current_state = MissionState.TAKEOFF
home_altitude = None

# Navigation -----------
home_position = None
delivery_geofence = None
corridor_entry = None

# SEARCH --------------
visited = set()
search_started = False
banner_detected = False

# CORRIDOR ----------------
corridor_detected = False
corridor_aligned = False
corridor_completed = False
corridor_last_seen = 0

# WORLD CONFIGURATIONS ------------ Drone's Gazebo world position at arm time (update if drone doesn't start at origin)
home_gz_x = 0.0
home_gz_y = 0.0

# FLIGHT CONFIGURATIONS --------------
CRUISE_ALTITUDE = 1.0   # metres
CORRIDOR_SPEED = 0.6
SEARCH_SPEED = 1.2
LAND_SPEED = 0.3
CORRIDOR_TOLERANCE = 40      # pixels
TARGET_REACHED = 0.7         # metres
SEARCH_RADIUS = 25           # metres

# Two separate queues
raw_queue      = queue.Queue(maxsize=2)   # callback  → QR worker
display_queue  = queue.Queue(maxsize=2)   # QR worker → main thread

mav_msgs       = {}
mav_msgs_lock  = threading.Lock()

# MISSION STATES ----------------
MISSION_TAKEOFF = 0
MISSION_SCAN_QR = 1
MISSION_FIND_CORRIDOR = 2
MISSION_FLY_CORRIDOR = 3
MISSION_NAVIGATE = 4
MISSION_DELIVER = 5
MISSION_RTL = 6
MISSION_LAND = 7
MISSION_COMPLETE = 8
MISSION_EMERGENCY = 9

mission_state = MISSION_TAKEOFF

class MissionState(Enum):
    TAKEOFF = 0
    SCAN_START_QR = 1
    FIND_GREEN_BANNER = 2
    ALIGN_CORRIDOR = 3
    FOLLOW_CORRIDOR = 4
    GO_TO_DELIVERY_ZONE = 5
    SEARCH_TARGET_QR = 6
    PAYLOAD_DROP = 7
    RETURN_TO_CORRIDOR = 8
    FOLLOW_RETURN_CORRIDOR = 9
    RETURN_HOME = 10
    LAND = 11
    COMPLETE = 12

# GIMBAL STATES ----------------
current_gimbal = None
GIMBAL_FORWARD = 0.0
GIMBAL_DOWN = 1.57

# ---------------- MAVLINK READER THREAD ----------------
def mavlink_reader():
    while True:
        msg = master.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        msg_type = msg.get_type()
        with mav_msgs_lock:
            mav_msgs[msg_type] = msg

threading.Thread(target=mavlink_reader, daemon=True).start()

def get_msg(msg_type, timeout=5):
    start = time.time()
    while time.time() - start < timeout:
        with mav_msgs_lock:
            msg = mav_msgs.get(msg_type)
        if msg:
            return msg
        time.sleep(0.05)
    return None

# ----------- COORDINATE CONVERSION (for Geofencing) -----------
def gazebo_to_ned(gz_x, gz_y, altitude):
    # Gazebo: X=east, Y=North --> MAVLink NED: X=North, Y=East, Z=Down
    ned_x = gz_y - home_gz_y
    ned_y = gz_x - home_gz_x
    ned_z = -altitude
    return ned_x, ned_y, ned_z

# ---------------- GIMBAL ----------------
def set_gimbal(angle):
    subprocess.run([
        "gz", "topic",
        "-t", "/gimbal/cmd_pitch",
        "-m", "gz.msgs.Double",
        "-p", f"data: {angle}"
    ])

def change_state(new_state):
    global current_state
    current_state = new_state
    update_gimbal()

def update_gimbal():
    global current_gimbal

    if current_state in (
        MissionState.SCAN_START_QR,
        MissionState.GO_TO_DELIVERY_ZONE,
        MissionState.SEARCH_TARGET_QR,
        MissionState.PAYLOAD,
        MissionState.RETURN_HOME,
    ):
        desired = GIMBAL_DOWN

    elif current_state in (
        MissionState.FIND_BANNER,
        MissionState.ALIGN_CORRIDOR,
        MissionState.FOLLOW_CORRIDOR,
        MissionState.RETURN_TO_CORRIDOR,
        MissionState.FOLLOW_RETURN_CORRIDOR,
    ):
        desired = GIMBAL_FORWARD

    else:
        return

    if desired != current_gimbal:
        set_gimbal(desired)
        current_gimbal = desired
        
# --------- CORRIDOR DETECTION (follow b/w the banners) --------
def detect_corridor():
    found = False
    offset = 0
    return found, offset

# ---------------- ARM + TAKEOFF ----------------
def arm_and_takeoff(altitude):
    global mission_ready

    print("Mission Started")

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
        msg = get_msg("GLOBAL_POSITION_INT")
        if msg:
            current_alt = msg.relative_alt / 1000.0  # mm to metres
            print(f"  Altitude: {current_alt:.2f} m")
            if current_alt >= altitude * 0.90:        # 90 % threshold
                break
        time.sleep(0.5)
        
    msg = get_msg("LOCAL_POSITION_NED") 
    home_position = (
        msg.x,
        msg.y,
        msg.z
    ) #store it for return mission
    
    print("Takeoff complete — Mission Ready")
    change_state(MissionState.SCAN_START_QR)

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
    print(f"  Waiting to reach target (tolerance={tolerance} m, timeout={timeout} s)...")
    start = time.time()
    while time.time() - start < timeout:
        msg = get_msg("LOCAL_POSITION_NED")
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
    change_state(MissionState.LAND)
    print("Landing command sent.")
    master.set_mode_apm("LAND")
    
# ---------------- CORRIDOR DETECTION ----------------
def detect_corridor(frame):

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Green colour range (Gazebo)
    lower_green = np.array([35, 50, 50])
    upper_green = np.array([85, 255, 255])

    # Binary mask
    mask = cv2.inRange(hsv, lower_green, upper_green)

    # Remove small noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours
    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    walls = []

    # Keep only large contours
    for cnt in contours:

        area = cv2.contourArea(cnt)

        if area > 800:
            walls.append(cnt)

    # Need at least two walls
    if len(walls) < 2:

        cv2.putText(
            frame,
            "Corridor Not Found",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2
        )

        return frame, False, None

    # Sort walls from left to right
    walls = sorted(
        walls,
        key=lambda c: cv2.boundingRect(c)[0]
    )

    left_wall = walls[0]
    right_wall = walls[1]

    # Bounding boxes
    x1, y1, w1, h1 = cv2.boundingRect(left_wall)
    x2, y2, w2, h2 = cv2.boundingRect(right_wall)

    # Draw bounding boxes
    cv2.rectangle(frame,
                  (x1, y1),
                  (x1 + w1, y1 + h1),
                  (0, 255, 0),
                  2)

    cv2.rectangle(frame,
                  (x2, y2),
                  (x2 + w2, y2 + h2),
                  (0, 255, 0),
                  2)

    # Wall centres
    left_center = x1 + w1 // 2
    right_center = x2 + w2 // 2

    # Corridor centre
    corridor_center = (left_center + right_center) // 2

    # Camera centre
    frame_center = frame.shape[1] // 2

    # Offset
    offset = corridor_center - frame_center

    # Draw corridor centre (Blue)
    cv2.line(
        frame,
        (corridor_center, 0),
        (corridor_center, frame.shape[0]),
        (255, 0, 0),
        2
    )

    # Draw camera centre (Red)
    cv2.line(
        frame,
        (frame_center, 0),
        (frame_center, frame.shape[0]),
        (0, 0, 255),
        2
    )

    # Draw wall centres
    cv2.circle(
        frame,
        (left_center, y1 + h1 // 2),
        5,
        (255, 255, 0),
        -1
    )

    cv2.circle(
        frame,
        (right_center, y2 + h2 // 2),
        5,
        (255, 255, 0),
        -1
    )

    # Draw corridor centre point
    cv2.circle(
        frame,
        (corridor_center, frame.shape[0] // 2),
        7,
        (255, 0, 255),
        -1
    )

    # Display offset
    cv2.putText(
        frame,
        f"Offset: {offset}px",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2
    )
    return frame, True, offset

# ---------------- CALLBACK — ULTRA LIGHTWEIGHT ----------------
# Only job: copy raw bytes and drop into raw_queue
# No decode, no numpy reshape, no QR — nothing heavy
def callback(msg):
    try:
        raw_queue.put_nowait((msg.data, msg.width, msg.height))
    except queue.Full:
        pass   # drop frame, never block gz transport thread

# ---------------- QR WORKER THREAD ----------------
# Does all the heavy work: reshape, QR decode, draw, navigate
def qr_worker():
    while True:
        try:
            data, width, height = raw_queue.get(timeout=0.5)
        except queue.Empty:
            continue
        # Reshape here, off the callback thread
        try:
            img   = np.frombuffer(data, dtype=np.uint8)
            frame = img.reshape((height, width, 3)).copy()
            frame, corridor_found, offset = detect_corridor(frame)
            if corridor_found:
                print(f"Corridor Offset = {offset}")
        except Exception as e:
            print(f"Reshape error: {e}")
            continue

        # QR decode only when mission active
        if current_state == MissionState.SCAN_START_QR:
            try:
                qr_codes = decode(frame)
                for qr in qr_codes:
                    x, y, w, h = qr.rect
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    text = qr.data.decode("utf-8")
                    cv2.putText(frame, text, (x, y-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    if text in visited:
                        continue

                    print(f"QR Detected: {text}")
                    visited.add(text)

                    if text.strip().upper() == "LAND":
                        land()
                        break

                    try:
                        parts = text.split(",")
                        gz_x  = float(parts[0].strip())
                        gz_y  = float(parts[1].strip())

                        ned_x, ned_y, ned_z = gazebo_to_ned(gz_x, gz_y, CRUISE_ALTITUDE)
                        print(f"Gazebo ({gz_x}, {gz_y}) → NED ({ned_x:.2f}, {ned_y:.2f}, {ned_z:.2f})")

                        def navigate(nx=ned_x, ny=ned_y, nz=ned_z):
                            global moving
                            moving = True
                            goto_position(nx, ny, nz)
                            wait_until_reached(nx, ny)
                            moving = False
                            print("Ready for next QR\n")

                        target_delivery = (ned_x, ned_y)
                        change_state(MissionState.FIND_GREEN_BANNER)
                        print(f"Stored destination: {current_target}")
                        print("Searching for corridor...")

                    except Exception as e:
                        print(f"  Invalid QR format: {e}")

            except Exception as e:
                print(f"QR decode error: {e}")

        # Send annotated frame to display queue
        try:
            display_queue.put_nowait(frame)
        except queue.Full:
            pass

threading.Thread(target=qr_worker, daemon=True).start()

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

cv2.namedWindow("Drone Camera", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Drone Camera", 640, 480)

# Run takeoff in background so main thread stays free for display
takeoff_thread = threading.Thread(target=arm_and_takeoff, args=(CRUISE_ALTITUDE,), daemon=True)
takeoff_thread.start()

print("Mission Started — scanning for QR codes...\n")

# Main thread handles ALL display — no freezing
try:
    while True:
        try:
            frame = display_queue.get(timeout=0.1)
            cv2.imshow("Drone Camera", frame)
        except queue.Empty:
            pass
        key = cv2.waitKey(1) & 0xFF
        
        # ---------------- MISSION LOGIC ----------------
        if mission_state == MISSION_FIND_CORRIDOR:
            if detect_corridor():
                print("Green corridor detected!")
                mission_state = MISSION_FLY_CORRIDOR
        elif mission_state == MISSION_FLY_CORRIDOR:
            print("Flying through corridor...")
            time.sleep(3)
            print("Exited corridor.")
            mission_state = MISSION_NAVIGATE
        elif mission_state == MISSION_NAVIGATE:
            if current_target is not None and not moving:
                x, y, z = current_target
                moving = True
                print("Navigating to stored destination...")
                goto_position(x, y, z)
                wait_until_reached(x, y)
                moving = False
                current_target = None
                mission_state = MISSION_SCAN_QR
                print("Waiting for next QR...")
                
        if key == ord('q'):
            print("Q pressed — landing...")
            land()
            break

except KeyboardInterrupt:
    print("Interrupted — landing...")
    land()

finally:
    cv2.destroyAllWindows()
    print("Done.")
