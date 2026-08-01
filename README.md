# SEAINDIA_AEROTHON AUTONOMOUS FLIGHT MISSION OPERATION - SKYSCAN
## The Project Baseline:
```
Drone
│
├── Flight control (ArduPilot)
├── Simulation (Gazebo)
├── Vision (OpenCV / YOLO)
├── Decision making (FSM)
├── Navigation
├── Payload system
└── Mission logic
```

## STATE MACHINE
```
                                                   TAKEOFF
                                                      │
                                                      ▼
                                              SCAN_START_QR
                                                      │
                                                      ▼
                                            FIND_GREEN_BANNER
                                                      │
                                              Corridor Found?
                                               │           │
                                             No│           │Yes
                                               │           ▼
                                               └──── ALIGN_CORRIDOR
                                                        │
                                               Centered on corridor?
                                                   │            │
                                                 No│            │Yes
                                                   │            ▼
                                                   └── FOLLOW_CORRIDOR
                                                             │
                                              ┌──────────────┴──────────────┐
                                              │                             │
                                       Obstacle?                      End of corridor /
                                              │                      Delivery zone seen?
                                       No     │     Yes                  │
                                              │                          ▼
                                              ▼                  GO_TO_DELIVERY_ZONE
                                     Continue following                 │
                                              │                         ▼
                                              │                 SEARCH_TARGET_QR
                                              │                         │
                                              │                 Target QR found?
                                              │                         │
                                              │                         ▼
                                              │                  PAYLOAD_DROP
                                              │                         │
                                              │                         ▼
                                              │                RETURN_TO_CORRIDOR
                                              │                         │
                                              │                         ▼
                                              │            FOLLOW_RETURN_CORRIDOR
                                              │                         │
                                              │                  Home reached?
                                              │                         │
                                              ▼                         ▼
                                        AVOID_OBSTACLE            RETURN_HOME
                                              │                         │
                                      Obstacle cleared?                ▼
                                         │          │                 LAND
                                       No│          │Yes               │
                                         │          ▼                  ▼
                                         └── Continue avoiding     COMPLETE
                                                    │
                                                    ▼
                                          REACQUIRE_CORRIDOR
                                                    │
                                           Corridor visible?
                                             │            │
                                           No│            │Yes
                                             │            ▼
                                     Search / Rotate   FOLLOW_CORRIDOR
```

## 5 Phases
  PHASE 1 — Build Simulation Foundation
    
    Goal: Make a drone exist in simulation and fly properly.
    Tasks
      1. ArduPilot SITL
      2. Gazebo world
             --------------------------------------------
            |          Gazebo (physics world)            |
            |                   ⇅                        |
            |  ArduPilot SITL (flight controller brain)  |
            |                   ⇅                        |
            |           MAVLink communication            |
             --------------------------------------------
        Next:
              launch Gazebo
              spawn drone
              make drone move
      3. MAVLink communication
         Understand:
          how ArduPilot talks
          how commands are sent
      4. Basic autonomous movement
          Example:
            takeoff
            move forward
            land
      THIS is the REAL foundation.
    
  PHASE 2 — Mission State Logic (MOST IMPORTANT)
    “Mission Phase Sequencing and State Transition Logic”

    Goal:IF this happens
          → do next step

    Current Goal: 
            Mission Manager
            │
            ├── Flight Controller (MAVLink)
            ├── Vision Manager
            │   ├── QR Detection
            │   ├── Corridor Detection
            │   ├── Banner Detection    (Here i am ✅)
            │   └── Red Zone Detection
            ├── Gimbal Manager
            ├── Navigation Manager
            ├── Safety Manager
            │   ├── Geofence
            │   ├── RTL
            │   └── LiDAR Avoidance
            └── Payload Manager

    That is called:
      FSM (Finite State Machine)
      YOU SHOULD IMPLEMENT THIS EARLY
      (Not with AI first.)

  PHASE 3 — COMPUTER VISION
  (ONLY after drone simulation works.)
        
    You first need: camera feed → detect something

    Order:
      Step 1: OpenCV camera stream in Gazebo
      Step 2: Detect QR code
              Using: cv2.QRCodeDetector()
      Step 3: Move drone based on QR position
      Step 4: Simple object detection

    Only later:
        YOLO
        TensorRT optimization
        ROS2 distributed pipelines
  
  PHASE 4 — PAYLOAD SYSTEM
    (Start VERY simple.)
    
    Initially:
      servo open = payload dropped
    
    Even simulation is enough initially.
    
  PHASE 5 — REPORT WRITING
  
       https://docs.google.com/document/d/1T9HJExvlILe7wq2aILxlRqLJgfOgMIOHqeEDX1SC7wY/edit?usp=drivesdk


## NOTES: 
          1) MAVProxy = manual command interface
                we hv commands like:
                    - guided
                    - takeoff
                    - wp                # waypoint navigation
                    - mode auto         # autonomous flying
 _____________________________________________________
|           Type            |         Behavior        |
| ------------------------- | ----------------------- |
| Velocity control          | keeps moving forever    |
| Position/waypoint control | goes to point and stops |
 _____________________________________________________






  
