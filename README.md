# G1 Motion Control

**An integrated software package for controlling the Unitree G1 humanoid robot on a Red Hat environment.**

Marker-guided approach, box grasping, carrying while walking, and a web-based motion editor — organized as a set of small HTTP servers, each owning one resource.

- Installation : [**INSTALL.md**](./INSTALL.md)
- Internals (joint map, motion JSON schema, IK pipeline, DDS, FSM) : [**TECH.md**](./TECH.md)

---

## ⚠️ Safety Notice (Read First)

1. **Secure the stand** — Tie the supplied stand firmly to the robot's shoulders so it does not wobble.
2. **`./start_fsm.sh stand`** immediately energizes the motors. **A person must physically hold the robot** while executing this command.
3. **`./start_fsm.sh sit`** slowly lowers the robot. Bring the arms to the sides first and **continue to support the robot** during the descent.
4. Keep the workspace clear of people and obstacles, and verify the **EMERGENCY STOP** location in advance.

---

## Architecture

Each hardware resource has exactly one owner process. Everything else talks HTTP.

```
rs_stream      :50001   RealSense camera (single owner) → MJPEG / frame API
detect_marker  :50011   ArUco marker pose (reads rs_stream)
detect_box     :50010   Box detection, grip points (reads rs_stream)
arm_server     :50022   rt/arm_sdk single owner — arms/waist, IK, hold/release
robot_server   :50000   Orchestrator — grab sequence, marker following, web UI
dashboard      :50003   3D URDF viewer / joint states
simulator      :8000    Motion editor (Joint + IK), talks to arm_server
```

Rules:
- **Arms/waist**: every process must go through `arm_server` (rt/arm_sdk allows only one publisher).
- **Locomotion**: `LocoClient` is a multi-client RPC — direct use is fine, but only one source may send move commands at a time.

## Quick Start

```bash
# 1. Posture (hold the robot!)
./start_fsm.sh stand        # or: sit

# 2. Full robot stack (6 servers, in dependency order)
./start_robot.sh
#   → control UI : http://<robot-ip>:50000/
#   → 3D viewer  : http://<robot-ip>:50003/dashboard

# 3. Motion editor (standalone, or alongside start_robot.sh)
./start_simulator.sh
#   → editor     : http://<robot-ip>:8000/
```

Logs are written to `logs/<name>_<date>.log` with timestamps. Stop with `Ctrl+C`.

## Typical Scenario

1. (External SLAM, optional) navigate near the table, **stop sending move commands**, then hand over.
2. `robot_server` follows the floor ArUco marker — approaches via a waypoint on the marker normal so it always arrives **facing the marker front** (stop at 0.25 m, lateral within 4 cm).
3. Grab the box (Box mode), **hold** (weight = 1, waist pitch −3° compensation), carry while walking.
4. **Release** returns arms/waist to the locomotion controller for natural arm-swing walking.

## Repository Layout

```
g1-motion-control/
├── start_fsm.sh            # Posture switch (stand / sit)
├── start_robot.sh          # Full stack (camera, detect, arm, robot, dashboard)
├── start_simulator.sh      # Motion editor (+ arm_server if not running)
├── activate_tv.sh          # venv activation
└── high/
    ├── robot_server.py     # Orchestrator + web UI (robot_web.html)
    ├── arm_server.py       # Arm/waist HTTP server (arm_sdk owner)
    ├── simulator.py        # Motion editor backend (simulator.html)
    ├── dashboard.py        # 3D URDF viewer
    ├── rs_stream.py        # Camera server
    ├── ctrl/               # Wrappers, IK, detect_marker/box, hand, TTS
    ├── motions/            # Motion JSON files
    ├── models/             # Detection models
    └── assets/             # URDF, meshes
```
