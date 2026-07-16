# Technical Documentation (TECH)

A deep dive into the internals of **G1 Motion Control** — architecture, joint mapping, motion JSON schema, IK pipeline, DDS topics, hand-controller protocol, and the FSM state machine.

For installation see [`INSTALL.md`](./INSTALL.md). For user-level usage see [`README.md`](./README.md).

---

## 1. High-Level Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                        Browser / Operator                        │
│         (web UI · keyboard · REST · ArUco trigger)               │
└────────────────────────────────┬─────────────────────────────────┘
                                 │ HTTP/SSE
┌────────────────────────────────▼─────────────────────────────────┐
│                       high/  (FastAPI layer)                      │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐    │
│  │ simulator.py │  │ run_motion.py│  │ ik_box.py · dashboard│    │
│  │  (editor)    │  │  (player)    │  │  (grasp · live view) │    │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────────────┘    │
│         │                 │                 │                    │
│  ┌──────▼─────────────────▼─────────────────▼───────────────┐    │
│  │     ctrl/  Wrappers (ArmControllerWrapper, LocoClient)   │    │
│  │     - IK (Pinocchio + CasADi)                            │    │
│  │     - Smoothstep joint interpolation @ 100 Hz             │    │
│  │     - Waist 3-DoF coordination                            │    │
│  │     - HandController (mandro3 serial)                     │    │
│  └──────────────────────────────────────────────────────────┘    │
└────────────────────────────────┬─────────────────────────────────┘
                                 │ Unitree DDS (CycloneDDS)
                                 │   rt/arm_sdk  (motion)
                                 │   rt/lowcmd   (debug)
                                 │   rt/lowstate (feedback)
                                 │   sport_*     (LocoClient)
┌────────────────────────────────▼─────────────────────────────────┐
│                  Unitree G1 (29 DOF) + Mandro Hand                │
└──────────────────────────────────────────────────────────────────┘
```

The PC runs three layers:

1. **Web / API** — FastAPI servers (`simulator.py`, `run_motion.py`, `ik_box.py`, `dashboard.py`, `rs_stream.py`).
2. **Controller wrappers** — `high/ctrl/arm_controller_wrapper.py` glues the raw Unitree SDK (`G1_29_ArmController`, `LocoClient`) and the IK solver into a single API used by every server.
3. **Transport** — CycloneDDS topics carry low-level commands and state to/from the robot at 250 Hz.

---

## 2. Component Inventory

| Path | Role |
| --- | --- |
| `start_fsm.sh` → `utils/init_fsm.py` | FSM ID transitions via `LocoClient.SetFsmId()` |
| `start_box.sh` | Launches 3 servers (rs_stream / ik_box / dashboard) |
| `start_motion.sh` | Launches `run_motion.py` only |
| `high/simulator.py` | Motion authoring (joint-angle editor, port `8000` by default) |
| `high/simulator_ik.py` | Motion authoring (IK XYZ+RPY editor) |
| `high/run_motion.py` | Plays both joint-format and IK-format motions, REST + 3D viewer (`:50003`) |
| `high/ik_box.py` | ArUco detection + IK box grasping FSM (`:50000`) |
| `high/rs_stream.py` | RealSense MJPEG color + depth streams (`:50001`) |
| `high/dashboard.py` | Aggregated 3D viewer + camera/depth proxy (`:50003`) |
| `high/ctrl/arm_controller_wrapper.py` | High-level wrapper, smoothstep interpolation, waist control |
| `high/ctrl/robot_arm.py` | `G1_29_ArmController` — DDS publisher/subscriber, motor command builder, CRC |
| `high/ctrl/robot_arm_ik.py` | `G1_29_ArmIK` — Pinocchio-CasADi dual-arm inverse kinematics |
| `high/ctrl/mandro3.py` | `HandController` — Mandro Mark-7 hand serial protocol |
| `high/ctrl/text_to_speech.py` | TTS used by the gift / grasping sequence |
| `high/assets/g1/g1_29dof_rev_1_0.urdf` + `meshes/` | URDF + STL meshes (consumed by IK + 3D viewer) |
| `high/motions/*.json` | Motion library (see §5) |
| `low/` | Direct low-level motor / DDS tests (`g1_motor_low.py`, `g1_motor_control.py`) |
| `vision/` | Camera utilities (RealSense, ArUco, calibration, OpenVINO NPU demos) |

---

## 3. Joint and Motor Index Map

The robot has **29 controllable DOF + 6 wheel/finger slots = 35 motor slots** on the DDS bus. There are **three different index spaces** used across the codebase — keep them straight.

### 3.1 Global motor index (used in motion JSON and REST APIs)

| Index | Joint | Group |
| ---: | --- | --- |
| 0 | waist_yaw | Waist |
| 1 | waist_roll | Waist |
| 2 | waist_pitch | Waist |
| 3–11 | hip / knee / ankle (left then right) | Legs (locked by IK) |
| 12 | waist_yaw (DDS) | (mirrored by 0) |
| 13 | waist_roll (DDS) | (mirrored by 1) |
| 14 | waist_pitch (DDS) | (mirrored by 2) |
| 15 | left_shoulder_pitch | L arm |
| 16 | left_shoulder_roll | L arm |
| 17 | left_shoulder_yaw | L arm |
| 18 | left_elbow | L arm |
| 19 | left_wrist_roll | L arm |
| 20 | left_wrist_pitch | L arm |
| 21 | left_wrist_yaw | L arm |
| 22 | right_shoulder_pitch | R arm |
| 23 | right_shoulder_roll | R arm |
| 24 | right_shoulder_yaw | R arm |
| 25 | right_elbow | R arm |
| 26 | right_wrist_roll | R arm |
| 27 | right_wrist_pitch | R arm |
| 28 | right_wrist_yaw | R arm |

> This is the index that `simulator.py` / `simulator_ik.py` write into JSON files (`motor_index` field). It is also what `run_motion.py` reads. The web UI and 3D viewer use the same numbering.

### 3.2 Internal arm index (used inside `ArmControllerWrapper`)

The wrapper exposes the 14 arm joints as a flat 0–13 vector. `GLOBAL_TO_INTERNAL` (in `arm_controller_wrapper.py`) translates global 15–28 → internal 0–13.

```
global 15..21 → internal 0..6   (left arm)
global 22..28 → internal 7..13  (right arm)
```

### 3.3 DDS motor slot (the `LowCmd`/`LowState` arrays of length 35)

Internally consumed by `G1_29_ArmController.ctrl_dual_arm()`. You normally never touch this — the wrapper does the mapping.

---

## 4. Control Pipeline (smooth motion)

Every joint or task-space target goes through a **smoothstep interpolator** before being published to DDS.

```python
# arm_controller_wrapper.py
for i in range(steps + 1):
    t = i / steps
    t_smooth = t * t * (3 - 2 * t)           # smoothstep
    interp   = start + t_smooth * (target - start)
    arm_ctrl.ctrl_dual_arm(interp, np.zeros(14))
    time.sleep(dt)                            # 100 Hz outer loop
```

- **Frequency:** 100 Hz interpolation step on the outer loop; the low-level controller publishes DDS at **250 Hz**.
- **Steps:** `int(duration * 100)`.
- **Stop:** any new call sets `_stop_interpolation`, waits 20 ms, clears it — guarantees old loops exit before a new one starts.
- **Waist:** `move_waist_smooth(yaw, roll, pitch, duration)` runs the same loop on the 3-DoF waist target and calls `arm_ctrl.ctrl_waist()`.
- **Dual-arm IK:** `move_hands(left_pos, right_pos, left_rot, right_rot, duration)` interpolates start→target poses in SE(3) (linear translation, **slerp** rotation) and runs `arm_ik.solve_ik()` at each step.

### Gains (from `G1_29_ArmController`)

| Group | kp | kd |
| --- | ---: | ---: |
| Shoulder/elbow (high) | 300 | 3.0 |
| Default (low) | 80 | 3.0 |
| Wrist | 40 | 1.5 |
| Waist | 150 | 3.0 |

### DDS Topics

| Direction | Topic | Type | Used when |
| --- | --- | --- | --- |
| → robot | `rt/arm_sdk` | `unitree_hg.msg.dds_.LowCmd_` | `motion_mode=True` (default, normal use) |
| → robot | `rt/lowcmd` | `unitree_hg.msg.dds_.LowCmd_` | `motion_mode=False` (debug/raw) |
| ← robot | `rt/lowstate` | `unitree_hg.msg.dds_.LowState_` | always (subscriber thread) |
| ↔ robot | `sport_*` | LocoClient RPC | walking, FSM |

Every outgoing `LowCmd_` is CRC-stamped via `unitree_sdk2py.utils.crc.CRC`.

---

## 5. Motion JSON Schema

`run_motion.py` reads **two formats** and auto-detects which one a file uses (it checks whether the first frame contains `left_xyz`/`right_xyz`).

### 5.1 Joint-angle format (produced by `simulator.py`)

```json
[
  {
    "duration": 2.0,
    "pose": {
      "targets": [
        { "motor_index": 0,  "target_degree": 0 },
        { "motor_index": 15, "target_degree": 10 },
        { "motor_index": 22, "target_degree": 10 }
      ]
    },
    "hand_motion": { "hand": "both", "motion": "fold_a" },
    "locomotion":  { "direction": "forward" }
  }
]
```

Per frame, at most one of `pose` / `locomotion` should be set; `hand_motion` runs in parallel. `motor_index` follows §3.1 (waist 0–2, arms 15–28). `target_degree` is in **degrees**.

### 5.2 IK XYZ+RPY format (produced by `simulator_ik.py`)

```json
[
  {
    "duration": 2.0,
    "left_xyz":  [0.20,  0.20, 0.15],
    "right_xyz": [0.20, -0.20, 0.15],
    "left_rpy":  [0, 0, 0],
    "right_rpy": [0, 0, 0],
    "hand_motion": { "hand": "left", "motion": "fold_a" }
  }
]
```

- XYZ is in **meters**, relative to the pelvis frame. The wrapper exposes `GROUND_TO_PELVIS = 0.782 m` to convert to floor height.
- RPY is in **degrees**, converted to a quaternion via `rpy_to_quaternion()`.
- Workspace bounds enforced by `validate_position()`: `x∈[0.1, 0.6]`, `y∈[0.0, 0.4]` (mirrored for right hand), `z∈[-0.3, 0.5]`.

### 5.3 Per-frame fields (both formats)

| Field | Type | Notes |
| --- | --- | --- |
| `duration` | float (s) | Interpolation time for this frame |
| `pose.targets[]` | list | Joint-format only |
| `left_xyz`, `right_xyz` | `[x,y,z]` | IK-format only |
| `left_rpy`, `right_rpy` | `[r,p,y]` | IK-format, optional, degrees |
| `locomotion.direction` | str | `forward` / `backward` / `left` / `right` / `turn_left` / `turn_right` |
| `hand_motion.hand` | str | `left` / `right` / `both` |
| `hand_motion.motion` | str | Key from `mandro3.motions` (e.g. `fold_a`, `unfold_a`, `point`, `handshake`) |

While a `locomotion.direction` is active, the wrapper re-issues `loco.move()` every 20 ms for `duration` seconds, then sends `loco.stop()` so the gait controller doesn't latch.

---

## 6. Inverse Kinematics (G1_29_ArmIK)

Implementation: **Pinocchio + CasADi**, solved with IPOPT.

- **URDF**: `high/assets/g1/g1_29dof_rev_1_0.urdf`.
- **Locked joints** (legs + waist): `left/right_hip_*`, `*_knee`, `*_ankle_*`, `waist_yaw/roll/pitch` — the IK only moves the 14 arm joints.
- **End-effectors**: virtual frames `L_ee`, `R_ee` placed 5 cm forward of the wrist-yaw link.
- **Cost terms**: SE(3) translation + orientation error on both hands, regularization on `q - q_prev`, smoothness on `dq`.
- **Cache**: the reduced robot model is pickled to `high/ctrl/g1_29_model_cache.pkl` on first load (URDF parse is slow — ~5 s without cache, ~50 ms with).
- **Smoothing**: solutions are passed through `WeightedMovingFilter` (5-tap weighted moving average on the joint vector) to suppress IK chatter.

`solve_and_verify_ik()` calls FK after IK to report per-hand position error in mm — the wrapper considers a result "accurate" when both errors are below **1 mm**.

---

## 7. FSM (Stand / Sit)

`utils/init_fsm.py` is the only place that drives the robot's posture machine. It uses `LocoClient.SetFsmId(n)`.

| FSM ID | Meaning |
| --- | --- |
| 0 | (zero torque / Damping) |
| 1 | Ready to stand |
| 3 | Sit down (slow descent → Damping) |
| 4 | Standing |
| 501 | Stand + motion mode unlocked (arm SDK enabled) |

Sequences:

```python
# stand
SetFsmId(1)   # ready
sleep(5)
SetFsmId(4)   # stand
sleep(10)
SetFsmId(501) # enable arm SDK (rt/arm_sdk active)

# sit
sleep(3)
SetFsmId(3)
```

`start_fsm.sh` wraps this with `sudo` because `rt/arm_sdk` requires elevated privileges to publish in motion mode.

> ⚠️ The robot energizes/de-energizes its motors at each transition. Always have a person physically supporting the robot during `stand` and `sit`.

---

## 8. LocoClient (walking)

Wrapped by `LocoClientWrapper`. The underlying `unitree_sdk2py.g1.loco.g1_loco_client.LocoClient` exposes:

| Method | Effect |
| --- | --- |
| `Move(vx, vy, vyaw, continous_move=False)` | Velocity command in body frame |
| `Damp()` | Cut torque (passive) |
| `SetStandHeight(h)` | Adjust pelvis height |
| `SetFsmId(n)` | Switch FSM state (used by `init_fsm.py`) |
| `SetTimeout(t)` | RPC timeout |

The wrapper exposes shorthand: `forward()`, `backward()`, `left()`, `right()`, `turn_left()`, `turn_right()` — all at default speed `0.3 m/s` (or `0.3 rad/s` for turn).

Because `Move` is not "continuous", every locomotion frame in `run_motion.py` loops at 50 Hz for the duration of the frame and explicitly sends `stop()` at the end.

---

## 9. Hand Controller (Mandro Mark-7)

A single USB serial dongle (`/dev/ttyACM0`) controls **both hands**. The first byte of every packet selects which hand:

| First byte | Target |
| --- | --- |
| `0xFD` | Left hand only |
| `0xFE` | Right hand only |
| `0xFF` | Both hands |

Packet layout (11 bytes):

```
B0  : selector (FD/FE/FF)
B1  : finger select bitmask  (bit0=Thumb, bit1=Index, bit2=Middle,
                              bit3=Ring,  bit4=Little, bit5=ThumbAbduction)
B2  : speed   (0x80 = 144 RPM/200; spec 30–150)
B3  : current (0xA0 = 1200 mA)
B4..B9 : per-finger position byte (Thumb, Idx, Mid, Ring, Lit, ThumbAbd)
B10 : direction (0=Idle, 1=Forward, 2=Reverse, 3=Reset)
```

`mandro3.motions` is the dictionary of named gestures: `fold_a` / `unfold_a` (close/open), `point`, `handshake`, etc. `HandController.send_motion(name, selector='both')` walks the gesture's command list with the correct inter-step delay.

> ⚠️ A data byte of `0x00` is interpreted by the hand as **"skip"** (keep previous position). `_to_data()` clamps any non-zero ratio to a minimum of `1` so a full-release command (`ratio=0.01`) actually moves.

---

## 10. ArUco Box Grasping (`ik_box.py`)

End-to-end pipeline of the grasp demo:

1. `rs_stream.py` publishes RealSense color+depth at 30 fps on `:50001/video_feed`.
2. `ik_box.py` consumes that stream in a 30 Hz detection thread, detecting 4×4_50 ArUco markers of edge **45 mm**.
3. The marker pose is stabilized over a **1 s median window** (≥3 samples) to reject jitter.
4. Camera→pelvis transform: `CAMERA_X/Y/Z` + pitch `0.8308 rad (≈47.6°)` (mounted on the head).
5. Box model: half-width 14 cm × half-depth 4.5 cm × height 9 cm; grasp points placed on the left/right faces with `GRIP_EXTRA=-0.04 m` (fingers wrap inside).
6. A 5-state FSM drives the action: `IDLE → APPROACH → GRASP → LIFT → HANDOVER → RELEASE → HOME`, calling `move_hands_with_rotation()` and the hand controller at each step.
7. TTS narration runs in parallel via `ctrl/text_to_speech.py`.

The web stream from `ik_box.py` itself (`:50000`) is throttled to **5 fps / 320×240** while the detection loop stays at 30 fps — this keeps demo CPU usage low.

---

## 11. REST API Cheat Sheet

### `simulator.py` (port 8000 by default; bound to all interfaces)

| Method | Path | Purpose |
| --- | --- | --- |
| GET | `/` | Web editor (`simulator.html`) |
| GET | `/joint_info` | Joint table (global/internal/name) |
| GET | `/hand_motions` | Available hand-motion names, dongle status |
| POST | `/set_motor` | Single-joint smooth move (`motor_index`, `target_degree`, `duration`) |
| POST | `/set_waist` | Waist 3-axis smooth move |
| POST | `/set_all_motors` | 14-arm-joint vector smooth move |
| POST | `/set_loco_motion` | Walk in a direction (de-duplicated for 100 ms) |
| POST | `/set_hand` | Hand motion (`hand`, `motion`, `release`) |
| POST | `/set_motion` | Play an in-memory motion sequence |
| POST | `/stop_motion` | Cancel current sequence + return to home |

### `run_motion.py` (port 50003)

| Method | Path | Purpose |
| --- | --- | --- |
| GET | `/` | Integrated UI (3D viewer + buttons) |
| GET | `/robot-only` | 3D viewer only |
| GET | `/docs` | OpenAPI docs |
| GET | `/status` | Subsystem readiness (`arm/loco/hand/tts`) |
| GET | `/motions` | List `*.json` under `high/motions/` |
| POST | `/motions/run/{filename}` | Run a saved motion (joint or IK; auto-detected) |
| POST | `/run` | Play inline joint-format frames |
| POST | `/run_file` | Upload + play a joint-format file |
| POST | `/run_ik` | Play inline IK frames |
| POST | `/run_ik_file` | Upload + play an IK file |
| POST | `/send_gift` | TTS + `right_send.json` sequence |
| POST | `/stop` | Stop, return to home |
| POST | `/home` | Go to home pose |
| POST | `/loco/move` | One-shot velocity command (`vx, vy, vyaw`) |
| POST | `/loco/stop` | Stop walking |
| GET | `/api/urdf` | URDF for the viewer |
| GET | `/api/meshes`, `/api/mesh/{f}` | STL list / fetch |
| GET | `/api/joint_states` | SSE stream of live joint angles + IMU |

### `ik_box.py` (port 50000)

| Method | Path | Purpose |
| --- | --- | --- |
| GET | `/` | Demo UI |
| GET | `/video_feed` | Annotated MJPEG (5 fps web throttle) |
| GET | `/status` | Detection / FSM state |
| GET | `/start_grab` | Trigger grasp manually |
| GET | `/release`, `/go_home`, `/set_wrist` | Manual overrides |
| GET | `/auto_mode`, `/set_auto_mode` | Toggle autonomous loop |

### `rs_stream.py` (port 50001) · `dashboard.py` (port 50003)

`rs_stream.py` exposes `/video_feed` (color, 30 fps, q80) and `/depth_feed` (depth, 320×240, q60).
`dashboard.py` proxies both streams under `/stream/...` and serves the 3D viewer at `/dashboard` so the box demo can be watched from a single page.

---

## 12. Adding a New Motion

1. `source activate_tv.sh && cd high && python simulator.py` (joint-angle editor) **or** `python simulator_ik.py` (IK editor).
2. Open `http://localhost:8000/` in a browser.
3. Drag sliders to pose the robot frame by frame; press the timeline button to record a keyframe and set its `duration`.
4. Click **Save** — the browser downloads a JSON in the format described in §5.
5. Drop the file into `high/motions/`.
6. Restart `start_motion.sh` (or just call `GET /motions` to verify it shows up) and run it from the web UI or:
   ```bash
   curl -X POST http://localhost:50003/motions/run/my_motion.json
   ```

`run_motion.py` auto-detects which format you used, so joint-angle and IK files coexist in the same folder.

---

## 13. Logs and Process Hygiene

Every launcher writes to `logs/<server>.log` and uses a `sweep_zombies` pre-cleanup so stale instances of the same script are `pkill -9`'d before launch. Useful one-liners:

```bash
tail -f logs/*.log                       # everything
pgrep -af 'rs_stream|ik_box|dashboard'   # what's running
pkill -9 -f run_motion.py                # nuke a hung server
```

Graceful shutdown is `SIGTERM` first; the launcher waits **8 seconds** (the IK servers need ~5.5 s to retract to home and disable motion mode) before escalating to `SIGKILL`. Never `kill -9` mid-motion if you can avoid it — leaving `rt/arm_sdk` in motion mode while the robot is unsupported can lead to a fall.

---

---

# 기술 문서 (한국어)

**G1 Motion Control** 내부 구조 — 아키텍처, 관절 인덱스, 모션 JSON 스키마, IK 파이프라인, DDS 토픽, 핸드 컨트롤러 프로토콜, FSM 등을 정리한 문서입니다.

설치는 [`INSTALL.md`](./INSTALL.md), 사용법은 [`README.md`](./README.md) 를 참고하세요.

---

## 1. 전체 아키텍처

```
┌──────────────────────────────────────────────────────────────────┐
│                       브라우저 / 운영자                          │
│   (웹 UI · 키보드 · REST · ArUco 트리거)                          │
└────────────────────────────────┬─────────────────────────────────┘
                                 │ HTTP / SSE
┌────────────────────────────────▼─────────────────────────────────┐
│                       high/  (FastAPI 계층)                       │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐    │
│  │ simulator.py │  │ run_motion.py│  │ ik_box.py · dashboard│    │
│  │  (에디터)    │  │  (재생)      │  │  (파지 · 라이브뷰)    │    │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────────────┘    │
│         │                 │                 │                    │
│  ┌──────▼─────────────────▼─────────────────▼───────────────┐    │
│  │  ctrl/  래퍼 (ArmControllerWrapper, LocoClientWrapper)    │    │
│  │  - IK (Pinocchio + CasADi)                                │    │
│  │  - 100Hz Smoothstep 보간                                  │    │
│  │  - 허리 3축 협조 제어                                     │    │
│  │  - HandController (mandro3 시리얼)                        │    │
│  └──────────────────────────────────────────────────────────┘    │
└────────────────────────────────┬─────────────────────────────────┘
                                 │ Unitree DDS (CycloneDDS)
                                 │   rt/arm_sdk · rt/lowcmd · rt/lowstate
                                 │   sport_* (LocoClient)
┌────────────────────────────────▼─────────────────────────────────┐
│                  Unitree G1 (29 DOF) + Mandro 핸드                │
└──────────────────────────────────────────────────────────────────┘
```

PC 측 소프트웨어는 3개 계층입니다.

1. **웹/API** — FastAPI 서버 5종 (`simulator.py`, `run_motion.py`, `ik_box.py`, `dashboard.py`, `rs_stream.py`).
2. **컨트롤러 래퍼** — `high/ctrl/arm_controller_wrapper.py` 가 Unitree SDK (`G1_29_ArmController`, `LocoClient`) 와 IK 솔버를 단일 API 로 묶어 모든 서버가 동일하게 사용합니다.
3. **전송 계층** — CycloneDDS 토픽이 PC 와 로봇 사이에서 250Hz 로 저수준 명령/상태를 주고받습니다.

---

## 2. 구성 파일 목록

| 경로 | 역할 |
| --- | --- |
| `start_fsm.sh` → `utils/init_fsm.py` | `LocoClient.SetFsmId()` 로 자세 FSM 전환 |
| `start_box.sh` | 3개 서버 (rs_stream / ik_box / dashboard) 일괄 실행 |
| `start_motion.sh` | `run_motion.py` 단독 실행 |
| `high/simulator.py` | 관절각 기반 모션 에디터 (기본 포트 `8000`) |
| `high/simulator_ik.py` | IK (XYZ + RPY) 기반 모션 에디터 |
| `high/run_motion.py` | 두 포맷 모두 자동 감지하여 재생, REST + 3D 뷰어 (`:50003`) |
| `high/ik_box.py` | ArUco 인식 + IK 박스 파지 FSM (`:50000`) |
| `high/rs_stream.py` | RealSense 컬러/깊이 MJPEG (`:50001`) |
| `high/dashboard.py` | 3D 뷰어 + 카메라 프록시 통합 (`:50003`) |
| `high/ctrl/arm_controller_wrapper.py` | 고수준 래퍼, Smoothstep 보간, 허리 제어 |
| `high/ctrl/robot_arm.py` | `G1_29_ArmController` — DDS Pub/Sub, LowCmd, CRC |
| `high/ctrl/robot_arm_ik.py` | `G1_29_ArmIK` — Pinocchio-CasADi 양팔 IK |
| `high/ctrl/mandro3.py` | `HandController` — Mandro Mark-7 시리얼 프로토콜 |
| `high/ctrl/text_to_speech.py` | 박스 파지/선물 시퀀스의 TTS |
| `high/assets/g1/g1_29dof_rev_1_0.urdf` + `meshes/` | URDF + STL (IK + 3D 뷰어 공용) |
| `high/motions/*.json` | 사전 정의 모션 라이브러리 (§5) |
| `low/` | 저수준 모터/DDS 테스트 |
| `vision/` | RealSense, ArUco, 캘리브레이션, OpenVINO NPU 도구 |

---

## 3. 관절 인덱스 매핑

G1 은 **29 개의 제어 DOF + 6 슬롯 = 총 35 슬롯** 의 DDS 모터 배열을 가집니다. 코드 곳곳에서 **세 종류의 인덱스 공간**이 등장하므로 구분이 중요합니다.

### 3.1 글로벌 인덱스 (모션 JSON · REST API 가 사용하는 번호)

| 인덱스 | 관절 |
| ---: | --- |
| 0 | waist_yaw |
| 1 | waist_roll |
| 2 | waist_pitch |
| 3–11 | 다리 (IK 에서는 잠금) |
| 12–14 | 허리 DDS 슬롯 (0–2 와 동일 관절을 가리킴) |
| 15–21 | 왼팔 (shoulder_pitch, roll, yaw, elbow, wrist_roll, pitch, yaw) |
| 22–28 | 오른팔 (동일 순서) |

> `simulator.py` 가 JSON 의 `motor_index` 필드로 저장하는 번호이며, `run_motion.py` 가 동일하게 읽어 들입니다. 웹 UI · 3D 뷰어도 같은 번호를 씁니다.

### 3.2 내부 팔 인덱스 (래퍼 내부)

`ArmControllerWrapper` 는 팔 14 축을 `[0..13]` 평탄 벡터로 노출합니다. `GLOBAL_TO_INTERNAL` 매핑으로 글로벌 15–28 → 내부 0–13 으로 변환됩니다 (왼팔 0–6, 오른팔 7–13).

### 3.3 DDS 모터 슬롯 (LowCmd/LowState 35칸 배열)

`G1_29_ArmController.ctrl_dual_arm()` 내부에서 사용. 사용자는 래퍼만 호출하면 되며 직접 다룰 일은 없습니다.

---

## 4. 제어 파이프라인 (스무스 모션)

모든 관절/태스크 공간 타겟은 DDS 로 나가기 전 **Smoothstep 보간기**를 통과합니다.

```python
# arm_controller_wrapper.py
for i in range(steps + 1):
    t = i / steps
    t_smooth = t * t * (3 - 2 * t)           # smoothstep
    interp   = start + t_smooth * (target - start)
    arm_ctrl.ctrl_dual_arm(interp, np.zeros(14))
    time.sleep(dt)                            # 100Hz 외부 루프
```

- **주파수:** 외부 보간 루프 100Hz, 하위 컨트롤러 DDS 송신 250Hz.
- **단계 수:** `int(duration * 100)`.
- **중단:** 새 명령은 `_stop_interpolation` 플래그를 set 후 20ms 대기, 다시 clear — 이전 루프가 종료된 뒤 새 루프 시작.
- **허리:** `move_waist_smooth(yaw, roll, pitch, duration)` 가 동일한 보간 루프로 `arm_ctrl.ctrl_waist()` 호출.
- **양팔 IK:** `move_hands(...)` 가 SE(3) 시작→타겟 자세를 선형 보간 (위치) + slerp (회전) 한 뒤 각 step 마다 `solve_ik()` 호출.

### 게인

| 그룹 | kp | kd |
| --- | ---: | ---: |
| 어깨/팔꿈치 (high) | 300 | 3.0 |
| 기본 (low) | 80 | 3.0 |
| 손목 | 40 | 1.5 |
| 허리 | 150 | 3.0 |

### DDS 토픽

| 방향 | 토픽 | 타입 | 사용 시점 |
| --- | --- | --- | --- |
| → 로봇 | `rt/arm_sdk` | `LowCmd_` | `motion_mode=True` (기본, FSM 501 상태) |
| → 로봇 | `rt/lowcmd` | `LowCmd_` | `motion_mode=False` (디버그) |
| ← 로봇 | `rt/lowstate` | `LowState_` | 항상 (수신 스레드) |
| ↔ | `sport_*` | LocoClient RPC | 보행 / FSM |

송신 패킷은 모두 `unitree_sdk2py.utils.crc.CRC` 로 CRC 계산 후 전송됩니다.

---

## 5. 모션 JSON 스키마

`run_motion.py` 는 두 가지 포맷을 모두 읽고, 첫 프레임에 `left_xyz`/`right_xyz` 가 있는지로 자동 감지합니다.

### 5.1 관절각 포맷 (`simulator.py` 생성)

```json
[
  {
    "duration": 2.0,
    "pose": {
      "targets": [
        { "motor_index": 0,  "target_degree": 0 },
        { "motor_index": 15, "target_degree": 10 },
        { "motor_index": 22, "target_degree": 10 }
      ]
    },
    "hand_motion": { "hand": "both", "motion": "fold_a" },
    "locomotion":  { "direction": "forward" }
  }
]
```

프레임 당 `pose` 또는 `locomotion` 중 하나만 사용하고, `hand_motion` 은 병렬로 실행됩니다. `motor_index` 는 §3.1 (허리 0–2, 팔 15–28), `target_degree` 는 **도(°)** 단위.

### 5.2 IK XYZ + RPY 포맷 (`simulator_ik.py` 생성)

```json
[
  {
    "duration": 2.0,
    "left_xyz":  [0.20,  0.20, 0.15],
    "right_xyz": [0.20, -0.20, 0.15],
    "left_rpy":  [0, 0, 0],
    "right_rpy": [0, 0, 0],
    "hand_motion": { "hand": "left", "motion": "fold_a" }
  }
]
```

- XYZ 는 **m** 단위, 골반 프레임 기준. `GROUND_TO_PELVIS = 0.782 m` 을 더하면 바닥 기준 높이로 변환.
- RPY 는 **도(°)** 단위, `rpy_to_quaternion()` 으로 쿼터니언 변환 후 IK 입력.
- 작업 영역 검증 `validate_position()`: `x∈[0.1, 0.6]`, `y∈[0.0, 0.4]` (오른손은 좌우 반전), `z∈[-0.3, 0.5]`.

### 5.3 공통 필드

| 필드 | 타입 | 비고 |
| --- | --- | --- |
| `duration` | float (s) | 해당 프레임 보간 시간 |
| `locomotion.direction` | str | `forward`, `backward`, `left`, `right`, `turn_left`, `turn_right` |
| `hand_motion.hand` | str | `left` / `right` / `both` |
| `hand_motion.motion` | str | `mandro3.motions` 키 (`fold_a`, `unfold_a`, `point`, `handshake` 등) |

`locomotion.direction` 이 지정된 프레임은 20ms 마다 `loco.move()` 를 재호출하고 `duration` 종료 시 `loco.stop()` 을 보냅니다 — 보행 컨트롤러가 latch 되지 않도록 하기 위함.

---

## 6. 역기구학 (`G1_29_ArmIK`)

- **구현:** Pinocchio + CasADi, IPOPT 솔버.
- **URDF:** `high/assets/g1/g1_29dof_rev_1_0.urdf`.
- **잠금 관절:** 다리 12 개 + 허리 3 개 → 팔 14 축만 자유롭게 풀이.
- **엔드 이펙터:** wrist_yaw 링크 기준 5cm 앞쪽의 가상 프레임 `L_ee` / `R_ee`.
- **비용 함수:** 양손 위치+자세 SE(3) 오차, `q - q_prev` 정규화, `dq` 평활.
- **모델 캐시:** 최초 로드 시 `high/ctrl/g1_29_model_cache.pkl` 로 피클링 (URDF 파싱 5초 → 캐시 50ms).
- **출력 평활:** `WeightedMovingFilter` (5탭 가중 이동 평균) 으로 IK chatter 억제.

`solve_and_verify_ik()` 는 IK 직후 FK 로 손 위치를 재계산하여 **양손 모두 1mm 이내** 인지 확인합니다.

---

## 7. FSM (Stand / Sit)

`utils/init_fsm.py` 한 군데서 `LocoClient.SetFsmId(n)` 로 자세 머신을 제어합니다.

| FSM ID | 의미 |
| --- | --- |
| 0 | Damping (제로 토크) |
| 1 | Stand 준비 |
| 3 | 앉기 (천천히 하강 → Damping) |
| 4 | 서 있기 |
| 501 | 서 있기 + arm SDK 활성화 (`rt/arm_sdk` 송신 허용) |

호출 순서:

```python
# stand
SetFsmId(1); sleep(5)
SetFsmId(4); sleep(10)
SetFsmId(501)

# sit
sleep(3); SetFsmId(3)
```

`start_fsm.sh` 가 `sudo` 로 호출하는 이유는 motion mode 의 `rt/arm_sdk` publish 권한 때문입니다.

> ⚠️ 각 전환 시점에 모터에 토크가 들어가거나 빠집니다. **stand / sit 실행 중 반드시 사람이 로봇을 지지**해 주세요.

---

## 8. 보행 (`LocoClient`)

`LocoClientWrapper` 가 감싸는 `LocoClient` 의 주요 메서드:

| 메서드 | 동작 |
| --- | --- |
| `Move(vx, vy, vyaw, continous_move=False)` | 바디 프레임 속도 명령 |
| `Damp()` | 토크 차단 |
| `SetStandHeight(h)` | 골반 높이 조절 |
| `SetFsmId(n)` | FSM 전환 (init_fsm.py 가 사용) |
| `SetTimeout(t)` | RPC 타임아웃 |

래퍼는 `forward / backward / left / right / turn_left / turn_right` 단축 메서드를 기본 속도 `0.3 m/s` (회전은 `0.3 rad/s`) 로 제공합니다.

`Move` 는 latch 가 아니므로 `run_motion.py` 의 보행 프레임은 50Hz 로 재호출하고 종료 시 `stop()` 을 명시적으로 보냅니다.

---

## 9. 핸드 컨트롤러 (Mandro Mark-7)

USB 시리얼 동글 **하나(`/dev/ttyACM0`)** 가 양손을 모두 제어합니다. 패킷 첫 바이트가 대상 선택자.

| 첫 바이트 | 대상 |
| --- | --- |
| `0xFD` | 왼손 |
| `0xFE` | 오른손 |
| `0xFF` | 양손 |

11바이트 패킷 구조:

```
B0  : selector (FD/FE/FF)
B1  : 손가락 선택 비트마스크
       (bit0=엄지, 1=검지, 2=중지, 3=약지, 4=새끼, 5=엄지 외전)
B2  : speed   (0x80 = 144 RPM/200; spec 30–150)
B3  : current (0xA0 = 1200 mA)
B4..B9 : 손가락별 위치값 (Thumb, Idx, Mid, Ring, Lit, ThumbAbd)
B10 : 방향 (0=Idle, 1=Forward, 2=Reverse, 3=Reset)
```

`mandro3.motions` 딕셔너리에 `fold_a` / `unfold_a` (쥐기/펴기), `point`, `handshake` 등 사전 정의 제스처가 등록되어 있고, `HandController.send_motion(name, selector='both')` 가 각 단계 사이의 딜레이까지 처리해 줍니다.

> ⚠️ 데이터 바이트 `0x00` 은 핸드 펌웨어에서 **"skip"** (이전 위치 유지) 으로 처리됩니다. `_to_data()` 는 0 이 아닌 ratio 에 대해 최소 `1` 을 보장하여 풀-릴리스(`ratio=0.01`) 명령이 실제 움직이게 합니다.

---

## 10. ArUco 박스 파지 (`ik_box.py`) 파이프라인

1. `rs_stream.py` 가 RealSense 컬러/깊이를 30fps 로 `:50001/video_feed` 에 게시.
2. `ik_box.py` 는 30Hz 검출 스레드에서 가져와 **4×4_50 ArUco** (마커 한 변 45mm) 를 탐지.
3. 마커 자세는 **1초 윈도우 중앙값** (≥ 3 샘플) 으로 안정화하여 튀는 값 제거.
4. 카메라 → 골반 변환: `CAMERA_X/Y/Z` + pitch `0.8308 rad (≈ 47.6°)` (헤드 장착 보정값).
5. 박스 모델: 반폭 14cm, 반깊이 4.5cm, 높이 9cm. 좌우 면을 잡되 `GRIP_EXTRA = -0.04 m` 만큼 살짝 안쪽으로 파고들도록 보정.
6. 5단계 FSM: `IDLE → APPROACH → GRASP → LIFT → HANDOVER → RELEASE → HOME`. 각 단계마다 `move_hands_with_rotation()` + 핸드 동작 + TTS 멘트.
7. 웹 송신은 **5fps / 320×240** 으로 throttling (검출 루프는 30fps 유지).

---

## 11. REST API 한눈에 보기

### `simulator.py` (기본 포트 8000)

| 메서드 | 경로 | 설명 |
| --- | --- | --- |
| GET | `/` | 웹 에디터 (`simulator.html`) |
| GET | `/joint_info` | 관절 매핑 테이블 |
| GET | `/hand_motions` | 사용 가능한 핸드 모션 + 동글 상태 |
| POST | `/set_motor` | 단일 관절 부드러운 이동 |
| POST | `/set_waist` | 허리 3축 동시 이동 |
| POST | `/set_all_motors` | 팔 14축 일괄 이동 |
| POST | `/set_loco_motion` | 방향 보행 (100ms 중복 제거) |
| POST | `/set_hand` | 핸드 동작 |
| POST | `/set_motion` | 인-메모리 모션 시퀀스 실행 |
| POST | `/stop_motion` | 긴급 정지 + 홈 복귀 |

### `run_motion.py` (포트 50003)

| 메서드 | 경로 | 설명 |
| --- | --- | --- |
| GET | `/` | 통합 UI (3D 뷰어 + 버튼) |
| GET | `/robot-only` | 3D 뷰어만 |
| GET | `/docs` | OpenAPI 문서 |
| GET | `/status` | 서브시스템 상태 |
| GET | `/motions` | `high/motions/*.json` 목록 |
| POST | `/motions/run/{filename}` | 저장된 파일 실행 (포맷 자동 감지) |
| POST | `/run`, `/run_file` | 관절 포맷 실행 |
| POST | `/run_ik`, `/run_ik_file` | IK 포맷 실행 |
| POST | `/send_gift` | TTS + `right_send.json` 시퀀스 |
| POST | `/stop`, `/home` | 정지 / 홈 복귀 |
| POST | `/loco/move`, `/loco/stop` | 보행 리모컨 |
| GET | `/api/joint_states` | 실시간 관절 SSE 스트림 |
| GET | `/api/urdf`, `/api/mesh/{f}` | 3D 뷰어 자원 |

### `ik_box.py` (포트 50000)

| 메서드 | 경로 | 설명 |
| --- | --- | --- |
| GET | `/`, `/video_feed`, `/status` | UI / 영상 / 상태 |
| GET | `/start_grab`, `/release`, `/go_home`, `/set_wrist` | 수동 트리거 |
| GET | `/auto_mode`, `/set_auto_mode` | 자동 모드 On/Off |

### `rs_stream.py` (포트 50001) · `dashboard.py` (포트 50003)

`rs_stream.py` 는 `/video_feed` (컬러 30fps, q80), `/depth_feed` (320×240, q60) 제공. `dashboard.py` 가 둘을 `/stream/...` 로 프록시하고 3D 뷰어와 합쳐 `/dashboard` 한 페이지에서 데모를 확인할 수 있게 합니다.

---

## 12. 새로운 모션 추가하기

1. `source activate_tv.sh && cd high && python simulator.py` (관절각 에디터) 또는 `python simulator_ik.py` (IK 에디터) 실행.
2. 브라우저에서 `http://localhost:8000/` 접속.
3. 슬라이더로 자세를 잡고 타임라인에 키프레임 기록, `duration` 지정.
4. **Save** 버튼 → §5 의 JSON 다운로드.
5. `high/motions/` 폴더에 복사.
6. `start_motion.sh` 재시작 (혹은 `GET /motions` 로 확인) 후 웹 UI 에서 실행하거나:
   ```bash
   curl -X POST http://localhost:50003/motions/run/my_motion.json
   ```

`run_motion.py` 가 포맷을 자동 판별하므로 관절 포맷과 IK 포맷이 같은 폴더에 섞여 있어도 됩니다.

---

## 13. 로그 / 프로세스 관리

각 런처는 `logs/<서버>.log` 에 출력을 남기고, 시작 전에 `sweep_zombies` 로 같은 이름의 잔존 프로세스를 `pkill -9` 합니다.

```bash
tail -f logs/*.log                       # 전체 모니터링
pgrep -af 'rs_stream|ik_box|dashboard'   # 현재 실행 중
pkill -9 -f run_motion.py                # 행걸린 서버 강제 종료
```

종료는 먼저 `SIGTERM`, **8초** 대기 후 `SIGKILL` (ik_box 의 종료 시퀀스가 ~5.5초 필요). 모션이 진행 중일 때 임의로 `kill -9` 하면 `rt/arm_sdk` 가 motion mode 로 남아 로봇이 떨어질 수 있으니 가급적 피하세요.
