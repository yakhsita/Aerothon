The Project Baseline:

Drone
│
├── Flight control (ArduPilot)
├── Simulation (Gazebo)
├── Vision (OpenCV / YOLO)
├── Decision making (FSM)
├── Navigation
├── Payload system
└── Mission logic
    
  PHASE 1 — Build Simulation Foundation
    (This is where you are RIGHT NOW.)
    
    Goal: Make a drone exist in simulation and fly properly.
    Tasks
      1. ArduPilot SITL
        ✅ already done by you
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

    Example: Takeoff
            → scan QR
            → identify target
            → move
            → drop payload
            → return
            → land

    That is called:
      FSM (Finite State Machine)
      YOU SHOULD IMPLEMENT THIS EARLY
      (Not with AI first.)

  PHASE 3 — COMPUTER VISION
  (ONLY after drone simulation works.)

    Start SIMPLE
    DO NOT jump to:
        YOLOv9
        TensorRT
        DBSCAN
        ROS2 distributed pipelines
        
    You first need: camera feed → detect something

    Order:
      Step 1: OpenCV camera stream in Gazebo
      Step 2: Detect QR code
              Using: cv2.QRCodeDetector()
      Step 3: Move drone based on QR position
      Step 4: Simple object detection

    Only later:
        YOLO
        TensorRT
        optimization
  
  PHASE 4 — PAYLOAD SYSTEM
    (Start VERY simple.)
    
    Initially:
      servo open = payload dropped
    
    Even simulation is enough initially.
    
  PHASE 5 — REPORT WRITING
    (NOW the report becomes easy.)






  
