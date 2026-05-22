# present working code

from gz.transport13 import Node
import cv2
import numpy as np
from gz.msgs10.image_pb2 import Image
from pyzbar.pyzbar import decode
from pymavlink import mavutil
import time

# ---------------- MAVLINK CONNECTION ----------------
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Connected to drone")

# ---------------- CAMERA TOPIC ----------------
topic = "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"
node = Node()
visited = set()
moving = False
mission_ready = False

# ---------------- ARM + TAKEOFF ----------------
def arm_and_takeoff(altitude):
    print("Setting GUIDED mode...")
    master.set_mode_apm("GUIDED")
    time.sleep(2)
    print("Arming motors...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Motors armed!")
    time.sleep(3)
    print(f"Taking off to {altitude} meters")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        altitude
    )
    time.sleep(15)
    print("Takeoff complete")
    
    global mission_ready
    mission_ready = True
    print("Mission Ready")

# ---------------- GOTO FUNCTION ----------------
def goto_position(x, y, z):
    print(f"Going to X={x}, Y={y}, Z={z}")
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,
        x,
        y,
        -z,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )

# ---------------- LAND FUNCTION ----------------
def land():
    print("Landing...")
    master.set_mode_apm("LAND")

# ---------------- CAMERA CALLBACK ----------------
def callback(msg):
    global visited
    global moving
    global mission_ready
    
    if not mission_ready:
        return
    if moving:
        return
        
    img = np.frombuffer(msg.data, dtype=np.uint8)
    frame = img.reshape((msg.height, msg.width, 3)).copy()
    qr_codes = decode(frame)

    for qr in qr_codes:
        x, y, w, h = qr.rect
        cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
        text = qr.data.decode("utf-8")
        cv2.putText(frame, text, (x, y-10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0,255,0), 2)

        if text not in visited:
            print("QR Detected:", text)
            visited.add(text)

            # -------- LAND COMMAND --------
            if text == "LAND":
                land()
                return

            # -------- COORDINATE QR --------
            try:
                coords = text.split(",")
                target_x = float(coords[0])
                target_y = float(coords[1])
                moving = True
                goto_position(target_x, target_y, 4)
                print("Travelling...")
                time.sleep(15)
                moving = False
                print("Ready for next QR")
            except:
                print("Invalid QR format")
    cv2.imshow("Drone Camera", frame)
    cv2.waitKey(1)

# ---------------- START CAMERA ----------------
node.subscribe(Image, topic, callback)

# ---------------- START MISSION ----------------
arm_and_takeoff(4)
print("Mission Started")

while True:
    pass



