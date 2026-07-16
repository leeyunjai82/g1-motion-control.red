# G1 Motion Control

**An integrated software package for controlling the Unitree G1 humanoid robot on a Red Hat environment.**

This project provides posture control (stand/sit), box grasping based on ArUco markers, keyboard-driven motion execution, and a web-based motion simulator — all in a single package.

- Installation : [**INSTALL.md**](./INSTALL.md)
- Internals (joint map, motion JSON schema, IK pipeline, DDS, FSM) : [**TECH.md**](./TECH.md)

---

## ⚠️ Safety Notice (Read First)

The robot is shipped **disassembled with a support stand**. Before operating, please observe the following:

1. **Secure the stand** — Tie the supplied stand firmly to the robot's shoulders so it does not wobble.
2. **When running `stand`** — `./start_fsm.sh stand` immediately energizes the motors. **A person must physically hold the robot** while executing this command.
3. **When running `sit`** — `./start_fsm.sh sit` slowly lowers the robot. Bring the arms to attention posture (sides of the body) first, and **continue to support the robot** during the descent.
4. Ensure there are no people or obstacles in the workspace.
5. Verify the location of the **EMERGENCY STOP** in advance for immediate power-off.

---

## Directory Structure

```
g1-motion-control/
├── start_fsm.sh         # Posture switch (stand / sit)
├── start_box.sh         # Box-grasping system (runs 3 servers)
├── start_motion.sh      # Keyboard motion controller
├── activate_tv.sh       # Conda env activation helper
├── requirements.txt     # Python package list
│
├── high/                # High-level control (FastAPI, IK, simulator)
│   ├── simulator.py     # Web-based motion editor
│   ├── rs_stream.py     # RealSense streaming server
│   ├── ik_box.py        # ArUco + IK grasping server
│   ├── dashboard.py     # Unified dashboard
│   └── run_motion.py    # Keyboard/button motion runner
│
├── low/                 # Low-level control (motors, DDS)
├── utils/               # Utilities (FSM init, etc.)
│   └── init_fsm.py
├── vision/              # Vision modules (YOLO, MediaPipe, etc.)
├── docs/                # Additional documentation
├── high.org             # Development notes (org-mode)
├── logs/                # Runtime logs (auto-generated)
│
├── README.md            # This file (usage overview)
├── INSTALL.md           # Step-by-step installation
└── TECH.md              # Technical deep-dive (architecture, JSON schema, DDS, IK, FSM)
```

---

## Main File Descriptions

### 🔵 `start_fsm.sh` — Posture Switch

Switches between the **Damping** state (right after power-on) and the **Standing** state.

```bash
./start_fsm.sh stand    # Damping → Standing
./start_fsm.sh sit      # Standing → Sit → Damping
```

Internally invokes `utils/init_fsm.py` with `sudo`.
**Always hold the robot** while motors energize or de-energize.

---

### 🟢 `start_box.sh` — Box-Grasping System

A demo that detects an ArUco-marked box via the RealSense camera and computes dual-arm grasping poses using IK (Inverse Kinematics). Launches three FastAPI servers simultaneously.

| Server | Port | Role |
| --- | --- | --- |
| `rs_stream.py` | 50001 | RealSense color/depth MJPEG streaming |
| `ik_box.py` | 50000 | ArUco detection + IK + arm control |
| `dashboard.py` | 50003 | Unified web dashboard |

Browser endpoints after launch:
- Dashboard : `http://localhost:50003/dashboard`
- Robot viewer : `http://localhost:50003/robot-only`
- Camera only : `http://localhost:50001/video_feed`

Press `Ctrl+C` to terminate (SIGTERM, escalated to SIGKILL after 8 seconds).

---

### 🟡 `start_motion.sh` — Keyboard Motion Controller

Runs predefined G1 motions (walk, wave, etc.) using arrow keys and shortcuts. Launches `high/run_motion.py`, which serves both a web UI and a REST API.

Endpoints:
- Integrated UI : `http://localhost:50003/`
- API docs : `http://localhost:50003/docs`

Key mappings and available motions are listed inside the web UI.

---

### 🟣 `high/simulator.py` — Web Motion Simulator

A **web-based motion editor** for authoring, saving, and replaying motions. Manipulate joints via sliders in a browser, record keyframes on a timeline, and replay saved motions on the actual robot.

Features:
- Waist 3-axis control (yaw / pitch / roll)
- IK-based dual-arm positioning (X/Y/Z + RPY sliders)
- Hand (Mandro Mark 7) open/close control
- Timeline-based motion recording and JSON export
- Toggle between simulation mode and real-robot mode

Run:
```bash
source activate_tv.sh
cd high
python simulator.py
```

**Workflow** — design a motion in the simulator, export the JSON, drop it into `high/motions/`, then play it from `start_motion.sh`:

```bash
# 1) author the motion → downloads my_motion.json
python high/simulator.py        # http://localhost:8000/

# 2) save it next to the bundled motions
mv ~/Downloads/my_motion.json   high/motions/

# 3) run it
./start_motion.sh               # http://localhost:50003/  (web UI)
#   or directly via REST:
curl -X POST http://localhost:50003/motions/run/my_motion.json
```

Both joint-angle (`simulator.py`) and IK (`simulator_ik.py`) JSON formats coexist in `high/motions/`; `run_motion.py` auto-detects which one each file uses. See [`TECH.md §5`](./TECH.md) for the schema.

---

### 🔧 `activate_tv.sh` — Environment Activation

A helper script that activates the Miniconda `tv` environment. All other scripts call it internally, so you rarely need to invoke it manually.

```bash
source activate_tv.sh
```

---

## Standard Operating Flow

```
[1] Secure stand and verify safety
        ↓
[2] Power ON the robot  (Damping state)
        ↓
[3] ./start_fsm.sh stand        ← Hold the robot while executing
        ↓
[4] Pick task:
    - Box grasping demo : ./start_box.sh
    - Keyboard control  : ./start_motion.sh
    - Motion authoring  : python high/simulator.py
        ↓
[5] Finish work
        ↓
[6] Align arms to attention posture
        ↓
[7] ./start_fsm.sh sit          ← Hold the robot while executing
        ↓
[8] Power OFF the robot
```

---

## Log Monitoring

All scripts write runtime logs to the `logs/` directory.

```bash
tail -f logs/*.log              # Monitor everything in real time
tail -f logs/ik_box.log         # Box-grasping server only
tail -f logs/run_motion.log     # Motion runner only
```

---

## License and Contact

- This software was developed and delivered by Circulus Inc.
- Technical support : Circulus support team
- Unitree G1 SDK : https://github.com/unitreerobotics/unitree_sdk2_python


---
---
---


# G1 Motion Control (한국어)

**Unitree G1 휴머노이드 로봇을 Red Hat 환경에서 제어하는 통합 소프트웨어 패키지입니다.**

본 프로젝트는 G1 로봇의 자세 제어(서기/앉기), 박스 파지(ArUco 마커 기반), 키보드 기반 동작 실행, 그리고 웹 기반 모션 시뮬레이터를 하나의 패키지로 제공합니다.

- 설치 절차 : [**INSTALL.md**](./INSTALL.md)
- 내부 구조 (관절 매핑, 모션 JSON 스키마, IK, DDS, FSM 등) : [**TECH.md**](./TECH.md)

---

## ⚠️ 안전 주의사항 (반드시 먼저 읽어주세요)

본 로봇은 **조립되지 않은 상태로 거치대와 함께 납품**됩니다. 실제 동작을 시키기 전에 다음을 반드시 지켜주십시오.

1. **거치대 고정** — 지급된 거치대를 로봇 어깨에 단단히 묶고, 거치대가 흔들리지 않도록 고정해 주세요.
2. **Stand 명령 시** — `./start_fsm.sh stand` 실행 순간 모터에 힘이 들어갑니다. **반드시 사람이 로봇을 직접 잡은 상태**에서 명령을 실행하세요.
3. **Sit 명령 시** — `./start_fsm.sh sit` 실행 시 로봇이 서서히 앉습니다. 팔을 정자세(몸 옆에 차렷 자세)로 내린 뒤 실행하고, **앉는 동안에도 잡아주세요**.
4. 작업 공간 주변에 사람, 장애물이 없는지 확인하세요.
5. 비상 시 즉시 전원을 차단할 수 있도록 **EMERGENCY STOP 위치**를 미리 확인해 두세요.

---

## 디렉토리 구조

```
g1-motion-control/
├── start_fsm.sh         # 자세 전환 (stand / sit)
├── start_box.sh         # 박스 파지 시스템 (3개 서버 동시 실행)
├── start_motion.sh      # 키보드 모션 제어 소프트웨어
├── activate_tv.sh       # conda 환경 활성화 헬퍼
├── requirements.txt     # Python 패키지 목록
│
├── high/                # 고수준 제어 (FastAPI 서버, IK, 시뮬레이터)
│   ├── simulator.py     # 웹 기반 모션 에디터
│   ├── rs_stream.py     # RealSense 영상 스트리밍 서버
│   ├── ik_box.py        # ArUco + IK 박스 파지 서버
│   ├── dashboard.py     # 통합 대시보드
│   └── run_motion.py    # 키보드/버튼 모션 실행기
│
├── low/                 # 저수준 제어 (모터, DDS)
├── utils/               # 유틸리티 (FSM 초기화 등)
│   └── init_fsm.py
├── vision/              # 비전 모듈 (YOLO, MediaPipe 등)
├── docs/                # 추가 문서
├── high.org             # 개발 노트 (org-mode)
├── logs/                # 실행 로그 (자동 생성)
│
├── README.md            # 본 문서 (사용 개요)
├── INSTALL.md           # 단계별 설치 절차
└── TECH.md              # 기술 문서 (아키텍처, JSON 스키마, DDS, IK, FSM)
```

---

## 주요 파일 설명

### 🔵 `start_fsm.sh` — 자세 전환 스크립트

로봇 전원을 켠 직후의 **Damping 모드** 와 **Standing 모드** 를 전환합니다.

```bash
./start_fsm.sh stand    # Damping → Standing (서기)
./start_fsm.sh sit      # Standing → 앉기 → Damping (앉기)
```

내부적으로 `utils/init_fsm.py` 를 `sudo` 권한으로 호출합니다. 모터에 힘이 들어가거나 빠지는 순간에는 **반드시 로봇을 잡아주세요.**

---

### 🟢 `start_box.sh` — 박스 파지 시스템

ArUco 마커가 부착된 박스를 RealSense 카메라로 인식하고, IK(역기구학) 로 양팔 위치를 계산하여 잡는 데모입니다. 세 개의 FastAPI 서버를 동시에 실행합니다.

| 서버 | 포트 | 역할 |
| --- | --- | --- |
| `rs_stream.py` | 50001 | RealSense 컬러/깊이 영상 MJPEG 스트리밍 |
| `ik_box.py` | 50000 | ArUco 마커 검출 + IK 계산 + 팔 제어 |
| `dashboard.py` | 50003 | 위 두 서버를 통합한 웹 대시보드 |

실행 후 브라우저에서 접속:
- 대시보드 : `http://localhost:50003/dashboard`
- 로봇 뷰어 : `http://localhost:50003/robot-only`
- 카메라 단독 : `http://localhost:50001/video_feed`

종료는 `Ctrl+C` (TERM 신호 후 8초 안에 종료되지 않으면 KILL 처리).

---

### 🟡 `start_motion.sh` — 키보드 모션 제어

방향키와 단축키로 G1 의 사전 정의된 동작(걷기, 손 흔들기 등)을 실행하는 제어 소프트웨어입니다. `high/run_motion.py` 를 실행하며 웹 UI 와 REST API 를 함께 제공합니다.

접속 주소:
- 통합 UI : `http://localhost:50003/`
- API 문서 : `http://localhost:50003/docs`

키 매핑 및 사용 가능한 모션 목록은 웹 UI 에서 확인할 수 있습니다.

---

### 🟣 `high/simulator.py` — 웹 모션 시뮬레이터

새로운 모션을 만들고 저장/재생할 수 있는 **웹 기반 모션 에디터**입니다. 브라우저에서 슬라이더로 관절을 조작하고, 타임라인에 키프레임을 기록한 뒤 저장된 모션을 실제 로봇에서 재생할 수 있습니다.

주요 기능:
- 허리 3축 제어 (요/피치/롤)
- IK 기반 양팔 위치 제어 (X/Y/Z + RPY 슬라이더)
- 핸드(Mandro Mark 7) 개폐 제어
- 타임라인 기반 모션 녹화 및 JSON 저장
- 시뮬레이션 모드 / 실제 로봇 모드 전환

실행:
```bash
source activate_tv.sh
cd high
python simulator.py
```

**작업 흐름** — 시뮬레이터에서 모션을 설계해 JSON으로 내보낸 뒤, `high/motions/` 폴더에 넣으면 `start_motion.sh` 가 그대로 실행할 수 있습니다.

```bash
# 1) 모션 제작 → my_motion.json 다운로드
python high/simulator.py        # http://localhost:8000/

# 2) motions 폴더로 이동
mv ~/Downloads/my_motion.json   high/motions/

# 3) 실행
./start_motion.sh               # http://localhost:50003/ (웹 UI)
#   또는 REST 로 직접 호출:
curl -X POST http://localhost:50003/motions/run/my_motion.json
```

`simulator.py` (관절각 포맷) 와 `simulator_ik.py` (IK 포맷) 가 만든 JSON 은 같은 폴더에 섞여 있어도 됩니다. `run_motion.py` 가 포맷을 자동 판별합니다. 자세한 스키마는 [`TECH.md §5`](./TECH.md) 참고.

---

### 🔧 `activate_tv.sh` — 환경 활성화

Miniconda 의 `tv` 환경을 활성화하는 헬퍼 스크립트입니다. 다른 모든 스크립트가 내부에서 호출하므로 사용자가 직접 호출할 일은 거의 없습니다.

```bash
source activate_tv.sh
```

---

## 표준 사용 흐름

```
[1] 거치대 고정 및 안전 확인
        ↓
[2] 로봇 전원 ON  (Damping 모드)
        ↓
[3] ./start_fsm.sh stand        ← 사람이 잡은 상태에서 실행
        ↓
[4] 용도에 따라 선택:
    - 박스 파지 데모 :  ./start_box.sh
    - 키보드 동작 제어 : ./start_motion.sh
    - 모션 제작        : python high/simulator.py
        ↓
[5] 작업 종료
        ↓
[6] 팔을 차렷 자세로 정렬
        ↓
[7] ./start_fsm.sh sit          ← 사람이 잡은 상태에서 실행
        ↓
[8] 로봇 전원 OFF
```

---

## 로그 확인

모든 스크립트는 실행 로그를 `logs/` 디렉토리에 저장합니다.

```bash
tail -f logs/*.log              # 전체 실시간 모니터링
tail -f logs/ik_box.log         # 박스 파지 서버만
tail -f logs/run_motion.log     # 모션 실행기만
```

---

## 라이선스 및 문의

- 본 소프트웨어는 Circulus Inc. 에서 개발 및 납품한 패키지입니다.
- 기술 문의 : Circulus 기술지원팀
- Unitree G1 SDK 관련 : https://github.com/unitreerobotics/unitree_sdk2_python
