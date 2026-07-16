"""
G1 Motion Runner Server (Integrated)
- 관절값 포맷 (simulator.py)  → /run, /run_file
- IK 포맷     (simulator_ik.py) → /run_ik, /run_ik_file
- motions/ 폴더 모션 실행    → /motions, /motions/run/{filename}
- 선물 시퀀스 (TTS + 모션)   → /send_gift
- Loco 리모컨               → /loco/move, /loco/stop
- 통합 웹 UI                 → /            (3D viewer + 모션 + 리모컨)
- 로봇 단독 뷰              → /robot-only  (3D viewer + Joint States)
- URDF / Mesh / SSE         → /api/urdf, /api/meshes, /api/mesh/{f}, /api/joint_states, /api/status
- 로컬 Three.js (오프라인)  → /vendor/three.min.js

실행: python run_motion.py
웹:  http://로봇IP:50003/
docs: http://로봇IP:50003/docs
"""

import os
import sys
import json
import time
import asyncio
import numpy as np
from pathlib import Path
from contextlib import asynccontextmanager
from typing import List, Optional

import uvicorn
import pinocchio as pin
from fastapi import FastAPI, UploadFile, File, HTTPException
from fastapi.responses import FileResponse, HTMLResponse, StreamingResponse, Response
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from fastapi import Request
#from starlette.middleware.base import BaseHTTPMiddleware

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

# ===== 디렉토리 / 파일 경로 =====
MOTIONS_DIR = Path(current_dir) / "motions"
MOTIONS_DIR.mkdir(exist_ok=True)

ASSETS_DIR  = os.path.join(current_dir, 'assets', 'g1')
URDF_PATH   = os.path.join(ASSETS_DIR, 'g1_29dof_rev_1_0.urdf')
MESH_DIR    = os.path.join(ASSETS_DIR, 'meshes')
VENDOR_DIR  = os.path.join(current_dir, 'assets', 'vendor')

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from ctrl.arm_controller_wrapper import ArmControllerWrapper, LocoClientWrapper, GLOBAL_TO_INTERNAL

# 손 제어
try:
    from ctrl.mandro3 import HandController, motions as hand_motions
    HAND_AVAILABLE = True
except ImportError:
    HAND_AVAILABLE = False

# TTS
try:
    from ctrl.text_to_speech import TextToSpeech
    TTS_AVAILABLE = True
except ImportError:
    TTS_AVAILABLE = False


# ==========================================
# URDF 조인트 ↔ 모터 인덱스 매핑
# ==========================================
JOINT_TO_MOTOR = {
    'left_hip_pitch_joint':      0,
    'left_hip_roll_joint':       1,
    'left_hip_yaw_joint':        2,
    'left_knee_joint':           3,
    'left_ankle_pitch_joint':    4,
    'left_ankle_roll_joint':     5,
    'right_hip_pitch_joint':     6,
    'right_hip_roll_joint':      7,
    'right_hip_yaw_joint':       8,
    'right_knee_joint':          9,
    'right_ankle_pitch_joint':  10,
    'right_ankle_roll_joint':   11,
    'waist_yaw_joint':          12,
    'waist_roll_joint':         13,
    'waist_pitch_joint':        14,
    'left_shoulder_pitch_joint':  15,
    'left_shoulder_roll_joint':   16,
    'left_shoulder_yaw_joint':    17,
    'left_elbow_joint':           18,
    'left_wrist_roll_joint':      19,
    'left_wrist_pitch_joint':     20,
    'left_wrist_yaw_joint':       21,
    'right_shoulder_pitch_joint': 22,
    'right_shoulder_roll_joint':  23,
    'right_shoulder_yaw_joint':   24,
    'right_elbow_joint':          25,
    'right_wrist_roll_joint':     26,
    'right_wrist_pitch_joint':    27,
    'right_wrist_yaw_joint':      28,
}


# ==========================================
# 전역 상태
# ==========================================
arm:  Optional[ArmControllerWrapper] = None
loco: Optional[LocoClientWrapper]    = None
hand: Optional[object]               = None
tts:  Optional[object]               = None

is_running = False
STOP_FLAG  = False


# ==========================================
# Pydantic 모델 - 관절값 포맷
# ==========================================
class MotorTarget(BaseModel):
    motor_index:   int
    target_degree: float

class PoseData(BaseModel):
    targets: List[MotorTarget]

class LocomotionData(BaseModel):
    direction: str

class HandMotionData(BaseModel):
    hand:   str
    motion: str

class MotionFrame(BaseModel):
    duration:    float
    pose:        Optional[PoseData]              = None
    locomotion:  Optional[LocomotionData]        = None
    hand_motion: Optional[HandMotionData]        = None


# ==========================================
# Pydantic 모델 - IK 포맷
# ==========================================
class IKMotionFrame(BaseModel):
    duration:    float
    left_xyz:    Optional[List[float]]           = None
    right_xyz:   Optional[List[float]]           = None
    left_rpy:    Optional[List[float]]           = None
    right_rpy:   Optional[List[float]]           = None
    locomotion:  Optional[LocomotionData]        = None
    hand_motion: Optional[HandMotionData]        = None


# ==========================================
# Loco 리모컨 모델
# ==========================================
class LocoMoveRequest(BaseModel):
    vx:   float = 0.0
    vy:   float = 0.0
    vyaw: float = 0.0


# ==========================================
# 헬퍼
# ==========================================
def rpy_to_quaternion(roll_deg, pitch_deg, yaw_deg):
    roll  = np.radians(roll_deg)
    pitch = np.radians(pitch_deg)
    yaw   = np.radians(yaw_deg)
    cr, sr = np.cos(roll/2),  np.sin(roll/2)
    cp, sp = np.cos(pitch/2), np.sin(pitch/2)
    cy, sy = np.cos(yaw/2),   np.sin(yaw/2)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return pin.Quaternion(w, x, y, z).normalized()


def move_hands_with_rotation(left_xyz, right_xyz, left_rpy, right_rpy, duration, frequency=100):
    left_rot  = rpy_to_quaternion(*left_rpy)  if left_rpy  and any(v != 0 for v in left_rpy)  else None
    right_rot = rpy_to_quaternion(*right_rpy) if right_rpy and any(v != 0 for v in right_rpy) else None
    arm.move_hands(left_xyz, right_xyz, left_rot, right_rot, duration, frequency)


def execute_hand_motion_sync(h: str, motion: str):
    if hand:
        hand.send_motion(motion, selector=h)


def _read_motor_state():
    """arm.arm_ctrl 에서 현재 모터값과 IMU를 읽음. 실패 시 기본값."""
    try:
        if arm is not None and getattr(arm, "arm_ctrl", None) is not None:
            q = arm.arm_ctrl.get_current_motor_q()
            imu = arm.arm_ctrl.get_imu_rpy().tolist()
            return q, imu, True
    except Exception:
        pass
    return np.zeros(35), [0.0, 0.0, 0.0], False


# ==========================================
# Lifespan
# ==========================================
@asynccontextmanager
async def lifespan(app: FastAPI):
    global arm, loco, hand, tts

    print("[Motion Runner] 시작")
    ChannelFactoryInitialize(0)

    try:
        loco = LocoClientWrapper()
        print("✅ Loco 초기화 성공")
    except Exception as e:
        print(f"⚠️ Loco 초기화 실패: {e}")

    try:
        arm = ArmControllerWrapper(motion_mode=True, simulation_mode=False)
        arm.start()
        print("✅ Arm 초기화 성공")
    except Exception as e:
        print(f"⚠️ Arm 초기화 실패: {e}")

    if HAND_AVAILABLE:
        try:
            hand = HandController('/dev/ttyACM0')
            print("✅ 손 초기화 성공")
        except Exception as e:
            print(f"⚠️ 손 초기화 실패: {e}")

    if TTS_AVAILABLE:
        try:
            tts = TextToSpeech(verbose=False)
            print("✅ TTS 초기화 성공")
        except Exception as e:
            print(f"⚠️ TTS 초기화 실패: {e}")

    print("[Motion Runner] 준비 완료")
    print(f"  웹:        http://localhost:50003/")
    print(f"  로봇만:    http://localhost:50003/robot-only")
    print(f"  motions/:  {MOTIONS_DIR}")

    yield

    if arm:
        arm.go_home()
    print("[Motion Runner] 종료")


# ==========================================
# FastAPI 앱
# ==========================================
app = FastAPI(
    title="G1 Motion Runner",
    description="모션 실행 + Loco 리모컨 + 3D Viewer 통합 서버",
    version="2.1.0",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# ==========================================
# 공통 - 걷기 루프
# ==========================================
async def _run_loco(direction: str, duration: float):
    direction_map = {
        "forward":    loco.forward,
        "backward":   loco.backward,
        "left":       loco.left,
        "right":      loco.right,
        "turn_left":  loco.turn_left,
        "turn_right": loco.turn_right,
    }
    method = direction_map.get(direction)
    if method and loco:
        start = time.time()
        while time.time() - start < duration:
            if STOP_FLAG: break
            method()
            await asyncio.sleep(0.02)
        if not STOP_FLAG and loco:
            loco.stop()


# ==========================================
# 관절값 포맷 실행
# ==========================================
async def _execute_frames(frames: List[MotionFrame]):
    global is_running, STOP_FLAG
    is_running = True
    STOP_FLAG  = False
    loop = asyncio.get_running_loop()

    try:
        for i, frame in enumerate(frames):
            if STOP_FLAG:
                print(f"[Runner] 중단: 프레임 {i+1}")
                break

            print(f"[Runner] 프레임 {i+1}/{len(frames)} ({frame.duration}s)")

            hand_future = None
            if frame.hand_motion and hand:
                hand_future = loop.run_in_executor(
                    None, execute_hand_motion_sync,
                    frame.hand_motion.hand, frame.hand_motion.motion
                )

            if frame.pose and frame.pose.targets and arm:
                with arm.arm_ctrl.ctrl_lock:
                    arm_targets = np.degrees(arm.arm_ctrl.q_target.copy())
                try:
                    with arm.arm_ctrl.ctrl_lock:
                        waist_targets = np.degrees(
                            getattr(arm.arm_ctrl, 'waist_q_target', np.zeros(3)).copy()
                        )
                except:
                    waist_targets = np.zeros(3)

                has_waist = False
                for t in frame.pose.targets:
                    if 0 <= t.motor_index <= 2:
                        waist_targets[t.motor_index] = t.target_degree
                        has_waist = True
                    elif 15 <= t.motor_index <= 28:
                        arm_targets[GLOBAL_TO_INTERNAL[t.motor_index]] = t.target_degree

                tasks = [
                    loop.run_in_executor(
                        None, arm.move_joints_smooth,
                        arm_targets.tolist(), frame.duration
                    )
                ]
                if has_waist:
                    tasks.append(loop.run_in_executor(
                        None, arm.move_waist_smooth,
                        float(waist_targets[0]), float(waist_targets[1]),
                        float(waist_targets[2]), frame.duration
                    ))
                await asyncio.gather(*tasks)

            elif frame.locomotion and loco:
                await _run_loco(frame.locomotion.direction, frame.duration)
            else:
                await asyncio.sleep(frame.duration)

            if hand_future:
                await hand_future

    finally:
        is_running = False
        if loco: loco.stop()
        print("[Runner] 완료")


# ==========================================
# IK 포맷 실행
# ==========================================
async def _execute_ik_frames(frames: List[IKMotionFrame]):
    global is_running, STOP_FLAG
    is_running = True
    STOP_FLAG  = False
    loop = asyncio.get_running_loop()

    try:
        for i, frame in enumerate(frames):
            if STOP_FLAG:
                print(f"[IK Runner] 중단: 프레임 {i+1}")
                break

            print(f"[IK Runner] 프레임 {i+1}/{len(frames)} ({frame.duration}s)")

            hand_future = None
            if frame.hand_motion and hand:
                hand_future = loop.run_in_executor(
                    None, execute_hand_motion_sync,
                    frame.hand_motion.hand, frame.hand_motion.motion
                )

            if frame.left_xyz and frame.right_xyz and arm:
                left_rpy  = frame.left_rpy  or [0.0, 0.0, 0.0]
                right_rpy = frame.right_rpy or [0.0, 0.0, 0.0]
                await loop.run_in_executor(
                    None, move_hands_with_rotation,
                    frame.left_xyz, frame.right_xyz,
                    left_rpy, right_rpy, frame.duration, 100
                )

            elif frame.locomotion and loco:
                await _run_loco(frame.locomotion.direction, frame.duration)
            else:
                await asyncio.sleep(frame.duration)

            if hand_future:
                await hand_future

    finally:
        is_running = False
        if loco: loco.stop()
        print("[IK Runner] 완료")


# ==========================================
# 선물 시퀀스 (TTS + right_send.json)
# ==========================================
async def _execute_send_gift():
    loop = asyncio.get_running_loop()
    filepath = MOTIONS_DIR / "right_send.json"

    if not filepath.exists():
        print("[Send] right_send.json 없음")
        return

    try:
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception as e:
        print(f"[Send] 파싱 오류: {e}")
        return

    if not data:
        print("[Send] 빈 모션")
        return

    # 1) 시작 멘트 (재생 끝까지 대기)
    if tts:
        print("[Send] TTS: Here is your gift.")
        tts.speak("Here is your gift.")
        await loop.run_in_executor(None, tts.wait_until_done)

    # 2) 모션 실행 (포맷 자동 감지)
    first = data[0]
    is_ik = "left_xyz" in first or "right_xyz" in first
    if is_ik:
        frames = [IKMotionFrame(**f) for f in data]
        await _execute_ik_frames(frames)
    else:
        frames = [MotionFrame(**f) for f in data]
        await _execute_frames(frames)

    # 3) 종료 멘트
    if tts:
        print("[Send] TTS: Goodbye. Have a great day.")
        tts.speak("Goodbye. Have a great day.")


# ==========================================
# API: URDF / Mesh / Vendor / Joint States
# ==========================================
@app.get('/api/urdf')
def get_urdf():
    return FileResponse(URDF_PATH, media_type='text/xml')


@app.get('/api/meshes')
def list_meshes():
    files = [f for f in os.listdir(MESH_DIR) if f.lower().endswith('.stl')]
    return {'files': files}


@app.get('/api/mesh/{filename}')
def get_mesh(filename: str):
    path = os.path.join(MESH_DIR, filename)
    if not os.path.exists(path):
        for f in os.listdir(MESH_DIR):
            if f.lower() == filename.lower():
                path = os.path.join(MESH_DIR, f)
                break
    return FileResponse(path, media_type='application/octet-stream')


@app.get('/api/joint_states')
async def joint_states():
    async def gen():
        while True:
            q, imu, connected = _read_motor_state()
            data = {j: float(q[i]) for j, i in JOINT_TO_MOTOR.items()}
            data['_imu']       = imu
            data['_connected'] = connected
            yield f"data: {json.dumps(data)}\n\n"
            await asyncio.sleep(0.05)
    return StreamingResponse(
        gen(),
        media_type='text/event-stream',
        headers={'Cache-Control': 'no-cache', 'X-Accel-Buffering': 'no'}
    )


@app.get('/api/robot_status')
def robot_status():
    """3D viewer 용 - 로봇 연결 여부만 가볍게."""
    _, _, connected = _read_motor_state()
    return {'connected': connected}


@app.get('/vendor/three.min.js')
def vendor_three():
    path = os.path.join(VENDOR_DIR, 'three.min.js')
    if not os.path.exists(path):
        return Response(
            content=f"// three.min.js not found at {path}\n"
                    f"// 다음 명령으로 다운로드:\n"
                    f"//   curl -o {path} https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js",
            media_type='application/javascript',
            status_code=404,
        )
    return FileResponse(path, media_type='application/javascript')


# ==========================================
# API: 상태
# ==========================================
@app.get("/status", summary="실행 상태 확인")
async def status():
    return {
        "is_running": is_running,
        "arm_ready":  arm  is not None,
        "loco_ready": loco is not None,
        "hand_ready": hand is not None,
        "tts_ready":  tts  is not None,
    }


# ==========================================
# API: motions 폴더 모션
# ==========================================
@app.get("/motions", summary="motions 폴더 파일 목록")
async def list_motions():
    files = sorted([f.name for f in MOTIONS_DIR.glob("*.json")])
    return {"motions": files, "directory": str(MOTIONS_DIR)}


@app.post("/motions/run/{filename}", summary="motions 폴더 파일 실행")
async def run_motion_by_name(filename: str):
    if is_running:
        raise HTTPException(409, "이미 실행 중. /stop 먼저 호출하세요.")

    filepath = MOTIONS_DIR / filename
    if not filepath.exists() or not filepath.is_file():
        raise HTTPException(404, f"파일 없음: {filename}")

    try:
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception as e:
        raise HTTPException(400, f"파싱 오류: {e}")

    if not data:
        raise HTTPException(400, "빈 모션입니다.")

    # 포맷 자동 감지
    first = data[0]
    is_ik = "left_xyz" in first or "right_xyz" in first

    if is_ik:
        frames = [IKMotionFrame(**f) for f in data]
        asyncio.create_task(_execute_ik_frames(frames))
    else:
        frames = [MotionFrame(**f) for f in data]
        asyncio.create_task(_execute_frames(frames))

    return {
        "status":   "started",
        "frames":   len(frames),
        "filename": filename,
        "format":   "ik" if is_ik else "joint"
    }


# ==========================================
# API: 선물 시퀀스 (TTS + right_send.json)
# ==========================================
@app.post("/send_gift", summary="선물 모션 + TTS 시퀀스 (right_send.json)")
async def send_gift():
    if is_running:
        raise HTTPException(409, "이미 실행 중. /stop 먼저 호출하세요.")

    filepath = MOTIONS_DIR / "right_send.json"
    if not filepath.exists():
        raise HTTPException(404, "right_send.json 파일 없음")

    asyncio.create_task(_execute_send_gift())
    return {"status": "started"}


# ==========================================
# API: 관절값 포맷 실행
# ==========================================
@app.post("/run", summary="관절값 모션 실행 (JSON body)")
async def run_motion(frames: List[MotionFrame]):
    if is_running:
        raise HTTPException(409, "이미 실행 중. /stop 먼저 호출하세요.")
    if not frames:
        raise HTTPException(400, "빈 모션입니다.")
    asyncio.create_task(_execute_frames(frames))
    return {"status": "started", "frames": len(frames)}


@app.post("/run_file", summary="관절값 모션 파일 업로드 후 실행")
async def run_motion_file(file: UploadFile = File(...)):
    if is_running:
        raise HTTPException(409, "이미 실행 중. /stop 먼저 호출하세요.")
    try:
        data   = json.loads(await file.read())
        frames = [MotionFrame(**f) for f in data]
    except Exception as e:
        raise HTTPException(400, f"파일 파싱 오류: {e}")
    if not frames:
        raise HTTPException(400, "빈 모션입니다.")
    asyncio.create_task(_execute_frames(frames))
    return {"status": "started", "frames": len(frames), "filename": file.filename}


# ==========================================
# API: IK 포맷 실행
# ==========================================
@app.post("/run_ik", summary="IK 모션 실행 (JSON body)")
async def run_ik_motion(frames: List[IKMotionFrame]):
    if is_running:
        raise HTTPException(409, "이미 실행 중. /stop 먼저 호출하세요.")
    if not frames:
        raise HTTPException(400, "빈 모션입니다.")
    asyncio.create_task(_execute_ik_frames(frames))
    return {"status": "started", "frames": len(frames)}


@app.post("/run_ik_file", summary="IK 모션 파일 업로드 후 실행")
async def run_ik_motion_file(file: UploadFile = File(...)):
    if is_running:
        raise HTTPException(409, "이미 실행 중. /stop 먼저 호출하세요.")
    try:
        data   = json.loads(await file.read())
        frames = [IKMotionFrame(**f) for f in data]
    except Exception as e:
        raise HTTPException(400, f"파일 파싱 오류: {e}")
    if not frames:
        raise HTTPException(400, "빈 모션입니다.")
    asyncio.create_task(_execute_ik_frames(frames))
    return {"status": "started", "frames": len(frames), "filename": file.filename}


# ==========================================
# API: 공통 제어
# ==========================================
@app.post("/stop", summary="실행 중인 모션 정지")
async def stop_motion():
    global STOP_FLAG
    STOP_FLAG = True
    if loco: loco.stop()
    if arm:
        loop = asyncio.get_running_loop()
        await asyncio.gather(
            loop.run_in_executor(None, arm.move_joints_smooth, [0]*14, 1.0),
            loop.run_in_executor(None, arm.move_waist_smooth,  0.0, 0.0, 0.0, 1.0),
        )
    return {"status": "stopped"}


@app.post("/home", summary="홈 포지션으로 이동")
async def go_home():
    global STOP_FLAG
    STOP_FLAG = True
    await asyncio.sleep(0.1)
    if arm:
        loop = asyncio.get_running_loop()
        await asyncio.gather(
            loop.run_in_executor(None, arm.move_joints_smooth, [0]*14, 2.0),
            loop.run_in_executor(None, arm.move_waist_smooth,  0.0, 0.0, 0.0, 2.0),
        )
    return {"status": "home"}


# ==========================================
# API: Loco 리모컨
# ==========================================
@app.post("/loco/move", summary="Loco 이동 (50ms 간격 반복 호출)")
async def loco_move(req: LocoMoveRequest):
    if not loco:
        raise HTTPException(503, "Loco 미초기화")
    loco.move(req.vx, req.vy, req.vyaw)
    return {"ok": True}


@app.post("/loco/stop", summary="Loco 정지")
async def loco_stop_endpoint():
    if loco:
        loco.stop()
    return {"ok": True}


# ==========================================
# 3D Viewer JS / CSS (공통 - 두 페이지에서 같이 사용)
# ==========================================
VIEWER_CORE_CSS = r"""
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0a0a0f;color:#d0d0d8;font-family:'Segoe UI',system-ui,sans-serif;font-size:13px;overflow:hidden;height:100vh;display:flex;flex-direction:column}
header{background:#0f0f1a;border-bottom:1px solid #f9c30030;padding:0 16px;display:flex;align-items:center;gap:12px;height:44px;flex-shrink:0}
header h1{font-size:14px;color:#f9c300;font-weight:600;letter-spacing:.5px}
.badge{font-size:10px;padding:2px 8px;border-radius:10px;background:#1a1a2a;color:#555;border:1px solid #222}
.badge.ok{color:#4caf80;border-color:#4caf5044}
.badge.live{color:#f9c300;border-color:#f9c30044;animation:pulse 1.5s infinite}
.badge.err{color:#ff6666;border-color:#ff444444}
.badge.run{color:#3FB950;border-color:#3FB95044;animation:pulse 1s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.5}}
.main{display:flex;flex:1;overflow:hidden;min-height:0}

.viewport{flex:1;position:relative;overflow:hidden;background:#0a0a0f;min-width:0}
canvas#cv{display:block;width:100%!important;height:100%!important}
.hud{position:absolute;bottom:10px;left:10px;font-size:10px;color:#282838;pointer-events:none;line-height:1.9}
.load-overlay{position:absolute;inset:0;background:#0a0a0fdd;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:14px;z-index:5}
.load-title{font-size:15px;color:#f9c300;font-weight:500}
.pbar-bg{width:280px;height:5px;background:#1a1a28;border-radius:3px}
.pbar-fill{height:100%;background:#f9c300;border-radius:3px;transition:width .07s;width:0%}
.pbar-text{font-size:11px;color:#555}
.tooltip{position:absolute;background:#14141e;border:1px solid #2a2a3a;border-radius:6px;padding:5px 10px;font-size:11px;color:#aaa;pointer-events:none;z-index:100;display:none}

/* Joint States 우측 패널 */
.right{width:275px;background:#0d0d16;border-left:1px solid #1a1a28;display:flex;flex-direction:column;flex-shrink:0}
.right-head{padding:8px 12px;border-bottom:1px solid #1a1a28;display:flex;align-items:center;justify-content:space-between;gap:7px}
.right-head b{font-size:12px;font-weight:600;color:#aaa;white-space:nowrap}
.search-box{flex:1;background:#12121e;border:1px solid #1e1e2e;border-radius:5px;padding:4px 8px;color:#aaa;font-size:11px;outline:none;min-width:0}
.search-box::placeholder{color:#2a2a3a}
.search-box:focus{border-color:#f9c30044}
.jcount{font-size:10px;color:#444;white-space:nowrap}
.joints{flex:1;overflow-y:auto;padding:7px}
.no-joint{text-align:center;color:#2a2a3a;font-size:11px;padding:40px 16px;line-height:1.8}
.group-header{display:flex;align-items:center;gap:6px;padding:5px 4px;cursor:pointer;color:#444;font-size:10px;text-transform:uppercase;letter-spacing:.8px;user-select:none;margin-top:3px}
.group-header:hover{color:#666}
.garr{font-size:8px;transition:transform .15s;flex-shrink:0}
.garr.open{transform:rotate(90deg)}
.group-body{overflow:hidden}
.ji{margin-bottom:5px;background:#12121e;border-radius:7px;padding:6px 9px;border:1px solid #1a1a28}
.ji-head{display:flex;justify-content:space-between;align-items:center;margin-bottom:4px;gap:4px}
.ji-name{font-size:10px;color:#666;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;flex:1}
.ji-val{font-size:11px;color:#f9c300;font-weight:600;min-width:46px;text-align:right;flex-shrink:0;font-family:monospace}
.ji-bar{position:relative;height:4px;background:#1a1a28;border-radius:2px;overflow:hidden}
.ji-bar-fill{position:absolute;top:0;height:100%;background:#f9c300;transition:left .08s linear,width .08s linear;border-radius:2px}
.ji-bar-zero{position:absolute;top:0;left:50%;width:1px;height:100%;background:#333}
::-webkit-scrollbar{width:3px}
::-webkit-scrollbar-track{background:transparent}
::-webkit-scrollbar-thumb{background:#1e1e2e;border-radius:2px}
"""

VIEWER_CORE_JS = r"""
class Orbit {
  constructor(cam,el){
    this.cam=cam;this.el=el;this.target=new THREE.Vector3(0,.8,0);
    this.phi=1.2;this.theta=0.5;this.r=3.5;
    this._dn=false;this._btn=-1;this._lx=0;this._ly=0;
    el.addEventListener('mousedown',e=>{this._dn=true;this._btn=e.button;this._lx=e.clientX;this._ly=e.clientY;});
    window.addEventListener('mousemove',e=>{
      if(!this._dn)return;
      const dx=e.clientX-this._lx,dy=e.clientY-this._ly;this._lx=e.clientX;this._ly=e.clientY;
      if(this._btn===0){this.theta-=dx*.005;this.phi=Math.max(.04,Math.min(Math.PI-.04,this.phi-dy*.005));}
      else if(this._btn===2){const f=this.r*.0012;const rt=new THREE.Vector3().setFromMatrixColumn(cam.matrix,0);const up=new THREE.Vector3().setFromMatrixColumn(cam.matrix,1);this.target.addScaledVector(rt,-dx*f).addScaledVector(up,dy*f);}
      this.update();
    });
    window.addEventListener('mouseup',()=>this._dn=false);
    el.addEventListener('wheel',e=>{e.preventDefault();this.r=Math.max(.15,Math.min(60,this.r*(e.deltaY>0?1.1:.9)));this.update();},{passive:false});
    el.addEventListener('contextmenu',e=>e.preventDefault());
    this.update();
  }
  update(){const s=Math.sin(this.phi);this.cam.position.set(this.target.x+this.r*s*Math.sin(this.theta),this.target.y+this.r*Math.cos(this.phi),this.target.z+this.r*s*Math.cos(this.theta));this.cam.lookAt(this.target);}
  focusBox(box){const c=box.getCenter(new THREE.Vector3());const sz=box.getSize(new THREE.Vector3()).length();this.target.copy(c);this.r=sz*1.3;this.update();}
}

function parseSTL(buf){
  const dv=new DataView(buf);
  if(buf.byteLength<84)return parseASCII(new TextDecoder().decode(buf));
  const n=dv.getUint32(80,true);
  if(Math.abs(buf.byteLength-(84+n*50))<=4)return parseBin(dv,n);
  return parseASCII(new TextDecoder().decode(buf));
}
function parseBin(dv,n){
  const pos=new Float32Array(n*9),nrm=new Float32Array(n*9);let o=84;
  for(let i=0;i<n;i++){
    const nx=dv.getFloat32(o,true),ny=dv.getFloat32(o+4,true),nz=dv.getFloat32(o+8,true);o+=12;
    for(let j=0;j<3;j++){const b=i*9+j*3;pos[b]=dv.getFloat32(o,true);pos[b+1]=dv.getFloat32(o+4,true);pos[b+2]=dv.getFloat32(o+8,true);nrm[b]=nx;nrm[b+1]=ny;nrm[b+2]=nz;o+=12;}
    o+=2;
  }
  const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.BufferAttribute(pos,3));g.setAttribute('normal',new THREE.BufferAttribute(nrm,3));return g;
}
function parseASCII(txt){
  const pos=[],nrm=[];let nx=0,ny=0,nz=0;
  for(const ln of txt.split('\n')){const l=ln.trim();
    if(l.startsWith('facet normal')){const m=l.match(/normal\s+([\S]+)\s+([\S]+)\s+([\S]+)/);if(m){nx=+m[1];ny=+m[2];nz=+m[3];}}
    else if(l.startsWith('vertex')){const m=l.match(/vertex\s+([\S]+)\s+([\S]+)\s+([\S]+)/);if(m){pos.push(+m[1],+m[2],+m[3]);nrm.push(nx,ny,nz);}}
  }
  const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(pos,3));g.setAttribute('normal',new THREE.Float32BufferAttribute(nrm,3));return g;
}

function parseURDF(xml){
  const doc=new DOMParser().parseFromString(xml,'text/xml');
  const links={},joints={},materials={};

  doc.querySelectorAll('robot > material').forEach(m=>{
    const name=m.getAttribute('name');
    const c=m.querySelector('color')?.getAttribute('rgba');
    if(c){const [r,g,b]=c.split(/\s+/).map(Number);
      materials[name]=(Math.round(r*255)<<16)|(Math.round(g*255)<<8)|Math.round(b*255);}
  });
  materials['white']=0xe8e8e8;
  materials['dark']=0x1e1e1e;

  doc.querySelectorAll('link').forEach(el=>{
    const name=el.getAttribute('name');links[name]={name,visuals:[]};
    el.querySelectorAll('visual').forEach(v=>{
      const me=v.querySelector('geometry mesh');if(!me)return;
      const fn=me.getAttribute('filename')||'';
      const sc=(me.getAttribute('scale')||'1 1 1').split(/\s+/).map(Number);
      const matName=v.querySelector('material')?.getAttribute('name')||'';
      links[name].visuals.push({fn,sc,origin:parseOrig(v.querySelector('origin')),matName});
    });
  });
  doc.querySelectorAll('joint').forEach(el=>{
    const name=el.getAttribute('name'),type=el.getAttribute('type')||'fixed';
    const parent=el.querySelector('parent')?.getAttribute('link')||'';
    const child=el.querySelector('child')?.getAttribute('link')||'';
    const axEl=el.querySelector('axis');
    const axis=(axEl?.getAttribute('xyz')||'0 0 1').split(/\s+/).map(Number);
    const lim=el.querySelector('limit');
    joints[name]={name,type,parent,child,origin:parseOrig(el.querySelector('origin')),axis,
      limit:{lower:lim?+lim.getAttribute('lower'):-3.14,upper:lim?+lim.getAttribute('upper'):3.14}};
  });
  return{links,joints,materials};
}
function parseOrig(el){
  if(!el)return{xyz:[0,0,0],rpy:[0,0,0]};
  return{xyz:(el.getAttribute('xyz')||'0 0 0').split(/\s+/).map(Number),rpy:(el.getAttribute('rpy')||'0 0 0').split(/\s+/).map(Number)};
}
const basename=s=>s.split(/[/\\]/).pop().toLowerCase();
const sid=n=>n.replace(/\W/g,'_');

const REDHAT_COLOR='#EE0000';
const INTEL_COLOR='#0068B5';
const CIRCULUS_COLOR='#00AEEF';

function makeLogoTextMesh(logoGeo){
  let width=0.10, height=0.025, cx=0.005, cy=0, cz=0;
  if(logoGeo){
    logoGeo.computeBoundingBox();
    const bb=logoGeo.boundingBox;
    width  = (bb.max.y - bb.min.y);
    height = (bb.max.z - bb.min.z);
    cx     = bb.max.x + 0.0008;
    cy     = (bb.min.y + bb.max.y) * 0.5;
    cz     = (bb.min.z + bb.max.z) * 0.5;
  }
  const SCALE = 1.5;
  width  *= SCALE; height *= SCALE;
  const aspect=Math.max(0.3, width/Math.max(height,0.001));
  const canvas=document.createElement('canvas');
  canvas.height=256;
  canvas.width =Math.min(2048, Math.max(256, Math.round(256*aspect)));
  const ctx=canvas.getContext('2d');
  ctx.clearRect(0,0,canvas.width,canvas.height);
  const line1=[{text:'RedHat',color:REDHAT_COLOR},{text:' ',color:'#fff'},{text:'intel',color:INTEL_COLOR}];
  const line2=[{text:'Circulus',color:CIRCULUS_COLOR}];
  let fontSize=Math.round(canvas.height*0.42);
  const setFont=()=>{ctx.font=`900 ${fontSize}px "Segoe UI", Arial, sans-serif`;};
  setFont();
  const measure=(parts)=>parts.reduce((w,p)=>w+ctx.measureText(p.text).width,0);
  const maxW=canvas.width*0.92;
  let w1=measure(line1), w2=measure(line2);
  let widest=Math.max(w1,w2);
  if(widest>maxW){fontSize=Math.max(18,Math.floor(fontSize*maxW/widest));setFont();w1=measure(line1);w2=measure(line2);}
  ctx.textBaseline='middle';ctx.textAlign='left';
  ctx.strokeStyle='#000';ctx.lineWidth=Math.max(2,fontSize*0.04);ctx.lineJoin='round';
  function drawLine(parts,totalW,yPos){
    let x=(canvas.width-totalW)/2;
    for(const p of parts){
      ctx.strokeText(p.text,x,yPos);ctx.fillStyle=p.color;ctx.fillText(p.text,x,yPos);
      x+=ctx.measureText(p.text).width;
    }
  }
  drawLine(line1, w1, canvas.height*0.30);
  drawLine(line2, w2, canvas.height*0.72);
  const tex=new THREE.CanvasTexture(canvas);tex.anisotropy=8;tex.needsUpdate=true;
  const mat=new THREE.MeshBasicMaterial({map:tex,transparent:true,depthWrite:false,side:THREE.DoubleSide});
  const plane=new THREE.Mesh(new THREE.PlaneGeometry(width,height),mat);
  plane.rotation.x=Math.PI/2;plane.rotation.y=Math.PI/2;
  plane.position.set(cx,cy,cz);plane.renderOrder=10;
  return plane;
}

const cv=document.getElementById('cv'),vp=document.getElementById('vp');
const renderer=new THREE.WebGLRenderer({canvas:cv,antialias:true,preserveDrawingBuffer:true});
renderer.setPixelRatio(Math.min(devicePixelRatio,2));
renderer.setClearColor(0x0a0a0f);
const scene=new THREE.Scene();
const camera=new THREE.PerspectiveCamera(45,1,.001,500);
const orbit=new Orbit(camera,cv);
scene.add(new THREE.AmbientLight(0xffffff,.55));
const dl1=new THREE.DirectionalLight(0xffffff,.85);dl1.position.set(5,10,5);scene.add(dl1);
const dl2=new THREE.DirectionalLight(0x8899ff,.3);dl2.position.set(-5,5,-5);scene.add(dl2);
const grid=new THREE.GridHelper(14,28,0x181828,0x181828);scene.add(grid);
const raycaster=new THREE.Raycaster();const mouse=new THREE.Vector2();
function resize(){const w=vp.clientWidth,h=vp.clientHeight;renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();}
resize();new ResizeObserver(resize).observe(vp);

let robotRoot=null,jointObjs={},baseQs={},jointDefs={};
let allMeshes=[],allAxes=[],wireMode=false,axisMode=false;
let selectedMesh=null;
let valEls={},barEls={},jointLimits={};
let liveEvt=null;

let fps=0,fpsT=performance.now();
function renderLoop(now){
  requestAnimationFrame(renderLoop);
  renderer.render(scene,camera);
  fps++;if(now-fpsT>=1000){const el=document.getElementById('fpsEl');if(el)el.textContent='FPS: '+fps;fps=0;fpsT=now;}
}
requestAnimationFrame(renderLoop);

async function loadRobot(){
  document.getElementById('loadOv').style.display='flex';
  const pbar=document.getElementById('pbar'),ptext=document.getElementById('ptext');
  try{
    if(robotRoot){scene.remove(robotRoot);robotRoot=null;}
    jointObjs={};baseQs={};allMeshes=[];allAxes=[];valEls={};barEls={};jointLimits={};

    ptext.textContent='Loading URDF...';pbar.style.width='5%';
    const urdfTxt=await(await fetch('/api/urdf')).text();
    const{links,joints,materials}=parseURDF(urdfTxt);
    jointDefs=joints;

    ptext.textContent='Fetching mesh list...';pbar.style.width='10%';
    const meshListData=await(await fetch('/api/meshes')).json();
    const serverFiles=new Set(meshListData.files.map(f=>f.toLowerCase()));

    const needed=new Set();
    for(const l of Object.values(links))
      for(const v of l.visuals){const b=basename(v.fn);if(serverFiles.has(b))needed.add(b);}

    const logoFn=links['logo_link']?.visuals?.[0]?.fn;
    const logoBase=logoFn?basename(logoFn):null;

    const geos={};const total=needed.size;let loaded=0;
    await Promise.all([...needed].map(async b=>{
      const buf=await(await fetch('/api/mesh/'+b)).arrayBuffer();
      const geo=parseSTL(buf);geo.computeVertexNormals();geos[b]=geo;
      loaded++;pbar.style.width=`${10+(loaded/total)*85}%`;ptext.textContent=`STL ${loaded} / ${total}`;
    }));

    ptext.textContent='Building scene...';pbar.style.width='98%';
    await new Promise(r=>setTimeout(r,0));

    const childSet=new Set(Object.values(joints).map(j=>j.child));
    const rootName=Object.keys(links).find(l=>!childSet.has(l))||Object.keys(links)[0];

    function mkLink(lname){
      const link=links[lname];if(!link)return null;
      const grp=new THREE.Group();grp.name='link:'+lname;

      if(lname==='logo_link'){
        const logoGeo=logoBase?geos[logoBase]:null;
        const txt=makeLogoTextMesh(logoGeo);
        txt.userData={linkName:lname};
        allMeshes.push(txt);grp.add(txt);
      } else {
        for(const v of link.visuals){
          const b=basename(v.fn),geo=geos[b];if(!geo)continue;
          let color=(materials[v.matName]!==undefined)?materials[v.matName]:0xb0b0b0;
          if(/hand|finger|thumb|palm/i.test(lname)) color=0x1a1a1a;
          const isDark=color<0x555555;
          const mat=new THREE.MeshPhongMaterial({color,specular:isDark?0x222222:0x666666,shininess:isDark?55:25});
          const mesh=new THREE.Mesh(geo,mat);mesh.userData={linkName:lname};
          mesh.position.set(...v.origin.xyz);mesh.setRotationFromEuler(new THREE.Euler(...v.origin.rpy,'XYZ'));mesh.scale.set(...v.sc);
          allMeshes.push(mesh);grp.add(mesh);
        }
      }

      const ax=new THREE.AxesHelper(.08);ax.visible=false;allAxes.push(ax);grp.add(ax);
      for(const jt of Object.values(joints).filter(j=>j.parent===lname)){
        const jg=new THREE.Group();jg.name='joint:'+jt.name;
        jg.position.set(...jt.origin.xyz);jg.setRotationFromEuler(new THREE.Euler(...jt.origin.rpy,'XYZ'));
        jointObjs[jt.name]=jg;baseQs[jt.name]=jg.quaternion.clone();
        const child=mkLink(jt.child);if(child)jg.add(child);grp.add(jg);
      }
      return grp;
    }
    robotRoot=mkLink(rootName);
    if(robotRoot){
      robotRoot.rotation.x=-Math.PI/2;
      scene.add(robotRoot);
      const box=new THREE.Box3().setFromObject(robotRoot);
      robotRoot.position.y=-box.min.y;
      orbit.focusBox(new THREE.Box3().setFromObject(robotRoot));
    }
    buildJointDisplays(joints);
    const mv=Object.values(joints).filter(j=>j.type!=='fixed').length;
    const infoSec=document.getElementById('infoSec');if(infoSec)infoSec.style.display='block';
    const infoRows=document.getElementById('infoRows');
    if(infoRows)infoRows.innerHTML=
      [['Links',Object.keys(links).length],['Joints',Object.keys(joints).length],['Movable',mv],['Meshes',allMeshes.length]]
      .map(([k,v])=>`<div class="info-row"><span>${k}</span><span class="info-val">${v}</span></div>`).join('');
    const b=document.getElementById('statusBadge');if(b){b.textContent='Ready';b.className='badge ok';}
    pbar.style.width='100%';

    await checkConnection();
    startSSE();
    const lb=document.getElementById('liveBadge');if(lb)lb.style.display='';

  }catch(e){console.error(e);
    const b=document.getElementById('statusBadge');
    if(b){b.textContent='Error: '+e.message;b.className='badge err';}
  }
  document.getElementById('loadOv').style.display='none';
}

async function checkConnection(){
  try{
    const d=await(await fetch('/api/robot_status')).json();
    const dot=document.getElementById('connDot');const txt=document.getElementById('connText');
    if(dot&&txt){
      if(d.connected){dot.className='status-dot on';txt.textContent='Robot connected';txt.style.color='#4caf80';}
      else{dot.className='status-dot err';txt.textContent='Simulation mode';txt.style.color='#ff6666';}
    }
  }catch(e){console.error(e);}
}

function startSSE(){
  if(liveEvt)liveEvt.close();
  liveEvt=new EventSource('/api/joint_states');
  liveEvt.onmessage=e=>{
    const data=JSON.parse(e.data);
    if(data._imu){
      const [r,p,y]=data._imu;
      const imuDispEl=document.getElementById('imuDisp');
      if(imuDispEl)imuDispEl.textContent=
        `IMU  R:${(r*180/Math.PI).toFixed(1)}°  P:${(p*180/Math.PI).toFixed(1)}°  Y:${(y*180/Math.PI).toFixed(1)}°`;
      const imuSec=document.getElementById('imuSec');if(imuSec)imuSec.style.display='block';
      const imuRows=document.getElementById('imuRows');
      if(imuRows)imuRows.innerHTML=
        [['Roll',(r*180/Math.PI).toFixed(2)+'°'],['Pitch',(p*180/Math.PI).toFixed(2)+'°'],['Yaw',(y*180/Math.PI).toFixed(2)+'°']]
        .map(([k,v])=>`<div class="info-row imu-row"><span>${k}</span><span class="info-val">${v}</span></div>`).join('');
    }
    delete data._imu;delete data._connected;
    applyPose(data);
  };
  liveEvt.onerror=()=>{setTimeout(startSSE,1000);};
}

function buildJointDisplays(joints){
  const movable=Object.values(joints).filter(j=>j.type!=='fixed');
  const jcountEl=document.getElementById('jcount');if(jcountEl)jcountEl.textContent=movable.length;
  const groups={'Head':[],'Waist/Pelvis':[],'Left Arm':[],'Right Arm':[],'Left Hand':[],'Right Hand':[],'Left Leg':[],'Right Leg':[],'Other':[]};
  for(const j of movable){const n=j.name.toLowerCase();
    if(n.includes('head'))groups['Head'].push(j);
    else if(n.includes('waist'))groups['Waist/Pelvis'].push(j);
    else if(n.includes('left')&&(n.includes('shoulder')||n.includes('elbow')||n.includes('wrist')))groups['Left Arm'].push(j);
    else if(n.includes('right')&&(n.includes('shoulder')||n.includes('elbow')||n.includes('wrist')))groups['Right Arm'].push(j);
    else if(n.includes('left')&&(n.includes('hand')||n.includes('finger')||n.includes('thumb')||n.includes('index')||n.includes('middle')||n.includes('palm')))groups['Left Hand'].push(j);
    else if(n.includes('right')&&(n.includes('hand')||n.includes('finger')||n.includes('thumb')||n.includes('index')||n.includes('middle')||n.includes('palm')))groups['Right Hand'].push(j);
    else if(n.includes('left')&&(n.includes('hip')||n.includes('knee')||n.includes('ankle')))groups['Left Leg'].push(j);
    else if(n.includes('right')&&(n.includes('hip')||n.includes('knee')||n.includes('ankle')))groups['Right Leg'].push(j);
    else groups['Other'].push(j);
  }
  for(const j of movable)jointLimits[j.name]={lo:j.limit.lower,hi:j.limit.upper};

  const el=document.getElementById('jointsEl');
  if(!el)return;
  if(!movable.length){el.innerHTML='<div class="no-joint">No movable joints</div>';return;}
  let html='';
  for(const[gname,jlist]of Object.entries(groups)){
    if(!jlist.length)continue;const gid=sid(gname);
    html+=`<div class="group-header" onclick="toggleG('${gid}')"><span class="garr open" id="arr_${gid}">&#x25B6;</span><span>${gname}</span><span style="color:#2a2a3a;margin-left:4px">(${jlist.length})</span></div><div class="group-body" id="gb_${gid}">`;
    for(const j of jlist){
      html+=`<div class="ji" id="ji_${sid(j.name)}" data-joint="${j.name}">
        <div class="ji-head">
          <span class="ji-name" title="${j.name}">${j.name}</span>
          <span class="ji-val" id="v_${sid(j.name)}">0.0°</span>
        </div>
        <div class="ji-bar"><div class="ji-bar-zero"></div><div class="ji-bar-fill" id="b_${sid(j.name)}"></div></div>
      </div>`;
    }
    html+='</div>';
  }
  el.innerHTML=html;
  movable.forEach(j=>{valEls[j.name]=document.getElementById('v_'+sid(j.name));barEls[j.name]=document.getElementById('b_'+sid(j.name));});
  setTimeout(()=>document.querySelectorAll('.group-body').forEach(b=>b.style.maxHeight=b.scrollHeight+'px'),60);
}

window.toggleG=function(gid){
  const body=document.getElementById('gb_'+gid),arr=document.getElementById('arr_'+gid);
  const open=arr.classList.contains('open');
  body.style.maxHeight=open?'0px':body.scrollHeight+'px';arr.classList.toggle('open',!open);
};
const searchBox=document.getElementById('searchBox');
if(searchBox)searchBox.addEventListener('input',function(){
  const q=this.value.toLowerCase().trim();
  document.querySelectorAll('.ji').forEach(el=>el.style.display=(!q||el.dataset.joint?.toLowerCase().includes(q))?'':'none');
});

function setAngle(jname,angle){
  const jobj=jointObjs[jname],jdef=jointDefs[jname];if(!jobj||!jdef)return;
  const ax=new THREE.Vector3(...jdef.axis).normalize();
  jobj.quaternion.copy(baseQs[jname]).multiply(new THREE.Quaternion().setFromAxisAngle(ax,angle));
}

function applyPose(joints){
  for(const[jname,angle]of Object.entries(joints)){
    setAngle(jname,angle);
    if(valEls[jname])valEls[jname].textContent=(angle*180/Math.PI).toFixed(1)+'°';
    const lim=jointLimits[jname],bar=barEls[jname];
    if(bar&&lim){
      const range=Math.max(Math.abs(lim.lo),Math.abs(lim.hi),0.001);
      const ratio=Math.max(-1,Math.min(1,angle/range));
      if(ratio>=0){bar.style.left='50%';bar.style.width=(ratio*50)+'%';}
      else{bar.style.left=(50+ratio*50)+'%';bar.style.width=(-ratio*50)+'%';}
    }
  }
}

cv.addEventListener('click',e=>{
  if(!allMeshes.length)return;
  const rect=cv.getBoundingClientRect();
  mouse.x=((e.clientX-rect.left)/rect.width)*2-1;
  mouse.y=-((e.clientY-rect.top)/rect.height)*2+1;
  raycaster.setFromCamera(mouse,camera);
  const hits=raycaster.intersectObjects(allMeshes);
  if(selectedMesh){if(selectedMesh.material.color)selectedMesh.material.color.set(selectedMesh.userData.origColor||0x78909c);selectedMesh=null;}
  if(!hits.length)return;
  selectedMesh=hits[0].object;
  if(selectedMesh.material.color){
    selectedMesh.userData.origColor=selectedMesh.material.color.getHex();
    selectedMesh.material.color.set(0xf9c300);
  }
  const lname=selectedMesh.userData.linkName;
  Object.values(jointDefs).forEach(j=>{
    if(j.child===lname||j.parent===lname){const el=document.getElementById('ji_'+sid(j.name));if(el)el.scrollIntoView({block:'nearest',behavior:'smooth'});}
  });
  const tt=document.getElementById('tt');
  if(tt){tt.textContent=lname;tt.style.display='block';
    tt.style.left=(e.clientX-rect.left+10)+'px';tt.style.top=(e.clientY-rect.top+8)+'px';
    setTimeout(()=>tt.style.display='none',2000);}
});

window.addEventListener('load',()=>loadRobot());
"""


# ==========================================
# /  - 통합 페이지 (3D viewer + 모션 + 리모컨 + Joint States)
# ==========================================
INDEX_HTML = r"""<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>G1 Motion Runner</title>
<script src="/vendor/three.min.js"></script>
<style>
""" + VIEWER_CORE_CSS + r"""

/* === 좌측 컨트롤 패널 === */
.left{width:340px;background:#0d0d16;border-right:1px solid #1a1a28;display:flex;flex-direction:column;flex-shrink:0;overflow-y:auto}
.sec{padding:10px 12px;border-bottom:1px solid #1a1a28}
.sec-title{font-size:10px;text-transform:uppercase;color:#444;letter-spacing:1.2px;margin-bottom:8px;display:flex;align-items:center;justify-content:space-between}
.status-dot{width:7px;height:7px;border-radius:50%;background:#333;flex-shrink:0}
.status-dot.on{background:#4caf80;box-shadow:0 0 5px #4caf8088}
.status-dot.err{background:#ff4444}
.info-row{display:flex;justify-content:space-between;font-size:11px;padding:2px 0;color:#444}
.info-val{color:#666}
.imu-row{font-size:11px;padding:3px 0;color:#6a6a2a}

/* 모션 리스트 */
.motion-list{max-height:340px;overflow-y:auto;background:#0a0a0f;border:1px solid #1a1a28;border-radius:6px;padding:5px}
.motion-item{display:flex;justify-content:space-between;align-items:center;padding:10px 12px;margin:4px 0;background:#12121e;border:1px solid transparent;border-radius:6px;transition:all .15s}
.motion-item:hover{border-color:#f9c30055}
.motion-item .name{font-family:monospace;font-size:13px;color:#bbb;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;flex:1;margin-right:8px}
.run-btn{background:#f9c300;border:none;color:#0a0a0f;border-radius:5px;padding:8px 16px;font-size:14px;font-weight:700;cursor:pointer;flex-shrink:0;font-family:inherit;min-width:44px}
.run-btn:hover{background:#ffd23a}
.empty-msg{text-align:center;color:#2a2a3a;font-size:11px;padding:30px 8px;line-height:1.7}

/* 컨트롤 버튼 */
.ctrl-row{display:flex;gap:6px;margin-top:8px}
.ctrl-row .btn{flex:1}
.btn{background:#12121e;border:1px solid #1e1e2e;color:#bbb;border-radius:6px;padding:11px 12px;font-size:13px;font-weight:500;cursor:pointer;transition:all .15s;font-family:inherit}
.btn:hover{border-color:#f9c30055;color:#f9c300}
.btn.danger{border-color:#F8514944;color:#F85149}
.btn.danger:hover{background:#F85149;color:#0a0a0f;border-color:#F85149}
.btn.send{background:#f9c300;border-color:#f9c300;color:#0a0a0f;font-weight:700;font-size:14px;padding:13px}
.btn.send:hover{background:#ffd23a;border-color:#ffd23a;color:#0a0a0f}
.btn.full{width:100%}

/* 리모컨 */
.remote-pad{display:grid;grid-template-columns:repeat(5,1fr);grid-template-rows:repeat(3,1fr);gap:6px;aspect-ratio:5/3;margin:8px 0;min-height:180px}
.remote-pad button{background:#12121e;border:1px solid #1e1e2e;color:#bbb;border-radius:7px;cursor:pointer;font-size:26px;font-weight:600;font-family:inherit;user-select:none;-webkit-user-select:none;touch-action:none;transition:all .1s;padding:0;display:flex;align-items:center;justify-content:center}
.remote-pad button:hover{border-color:#f9c30055;color:#f9c300}
.remote-pad button.active{background:#f9c300;border-color:#f9c300;color:#0a0a0f;transform:scale(.94)}
#loco-forward    { grid-area: 1 / 3 / 2 / 4; }
#loco-backward   { grid-area: 3 / 3 / 4 / 4; }
#loco-left       { grid-area: 2 / 2 / 3 / 3; }
#loco-right      { grid-area: 2 / 4 / 3 / 5; }
#loco-stop       { grid-area: 2 / 3 / 3 / 4; font-size: 18px; color: #F85149; border-color: #F8514944; }
#loco-stop:hover { background: #F85149; color: #0a0a0f; }
#loco-turn_left  { grid-area: 2 / 1 / 3 / 2; }
#loco-turn_right { grid-area: 2 / 5 / 3 / 6; }

.speed-row{display:flex;align-items:center;gap:9px;font-size:12px;color:#888;margin-top:6px}
.speed-row input[type=range]{flex:1;-webkit-appearance:none;height:4px;background:#1a1a28;border-radius:2px;cursor:pointer}
.speed-row input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:18px;height:18px;background:#f9c300;border-radius:50%;cursor:grab}
.speed-row .val{font-family:monospace;color:#f9c300;min-width:36px;text-align:right;font-size:12px}

.hint-mini{font-size:9px;color:#333;margin-top:5px;line-height:1.5}

/* 토스트 */
#toast{position:fixed;bottom:18px;left:50%;transform:translateX(-50%);background:#1a1a28;border:1px solid #f9c30055;color:#f9c300;padding:7px 14px;border-radius:5px;font-size:12px;opacity:0;transition:opacity .2s;pointer-events:none;z-index:1000}
#toast.show{opacity:1}
#toast.error{border-color:#F8514955;color:#F85149}
</style>
</head>
<body>
<header>
  <h1>&#x1F916; G1 Motion Runner</h1>
  <span class="badge" id="statusBadge">Loading...</span>
  <span class="badge live" id="liveBadge" style="display:none">● LIVE</span>
  <span class="badge run" id="runBadge" style="display:none">▶ RUNNING</span>
  <span style="margin-left:auto;font-size:10px;color:#2a2a3a" id="imuDisp">IMU: -</span>
</header>

<div class="main">
  <!-- 좌측 컨트롤 -->
  <div class="left">
    <div class="sec">
      <div class="sec-title"><span>Connection</span></div>
      <div style="display:flex;align-items:center;gap:8px">
        <div class="status-dot" id="connDot"></div>
        <span style="font-size:11px;color:#555" id="connText">Checking...</span>
      </div>
    </div>

    <div class="sec">
      <div class="sec-title"><span>🎁 Gift Sequence</span></div>
      <button class="btn send full" onclick="sendGift()">🎁 Send</button>
      <div class="hint-mini">TTS "Here is your gift" → right_send.json → TTS "Goodbye"</div>
    </div>

    <div class="sec">
      <div class="sec-title">
        <span>📂 Motions</span>
        <span style="cursor:pointer;color:#666" onclick="refreshMotions()" title="새로고침">🔄</span>
      </div>
      <div class="motion-list" id="motionList">
        <div class="empty-msg">로딩 중...</div>
      </div>
      <div class="ctrl-row">
        <button class="btn danger" onclick="stopMotion()">⏹ Stop</button>
        <button class="btn" onclick="goHome()">🏠 Home</button>
      </div>
    </div>

    <div class="sec">
      <div class="sec-title">🎮 Loco Remote</div>
      <div class="speed-row">
        <span>속도</span>
        <input type="range" id="speed" min="0.1" max="0.5" step="0.05" value="0.3">
        <span class="val" id="speedVal">0.30</span>
      </div>
      <div class="remote-pad">
        <button id="loco-forward"    data-cmd="forward">▲</button>
        <button id="loco-turn_left"  data-cmd="turn_left">↺</button>
        <button id="loco-left"       data-cmd="left">◀</button>
        <button id="loco-stop"       onclick="locoStopBtn()">■</button>
        <button id="loco-right"      data-cmd="right">▶</button>
        <button id="loco-turn_right" data-cmd="turn_right">↻</button>
        <button id="loco-backward"   data-cmd="backward">▼</button>
      </div>
      <div class="hint-mini">↑↓←→ 이동 / Q,E 회전</div>
    </div>

    <div class="sec">
      <div class="sec-title">View</div>
      <div class="ctrl-row">
        <button class="btn" id="gridBtn">⊞ Grid</button>
        <button class="btn" id="wireBtn">▦ Wire</button>
      </div>
      <div class="ctrl-row">
        <button class="btn" id="axisBtn">↗ Axes</button>
        <button class="btn" id="ssBtn">📷 Shot</button>
      </div>
    </div>

    <div class="sec" id="infoSec" style="display:none">
      <div class="sec-title">Model Info</div>
      <div id="infoRows"></div>
    </div>

    <div class="sec" id="imuSec" style="display:none">
      <div class="sec-title">IMU (Pelvis)</div>
      <div id="imuRows"></div>
    </div>
  </div>

  <!-- 가운데 3D Viewer -->
  <div class="viewport" id="vp">
    <canvas id="cv"></canvas>
    <div class="load-overlay" id="loadOv">
      <div class="load-title">Loading model...</div>
      <div class="pbar-bg"><div class="pbar-fill" id="pbar"></div></div>
      <div class="pbar-text" id="ptext">Loading URDF...</div>
    </div>
    <div class="hud">
      <span id="fpsEl">FPS: --</span>
      <span style="color:#1e1e28">Left-click rotate / Right-click pan / Wheel zoom</span>
    </div>
    <div class="tooltip" id="tt"></div>
  </div>

  <!-- 우측 Joint States -->
  <div class="right">
    <div class="right-head">
      <b>Joint States</b>
      <input class="search-box" id="searchBox" placeholder="Search..." type="text">
      <span class="jcount" id="jcount">-</span>
    </div>
    <div class="joints" id="jointsEl">
      <div class="no-joint">Loading...</div>
    </div>
  </div>
</div>

<div id="toast"></div>

<script>
""" + VIEWER_CORE_JS + r"""

// === View 버튼 ===
const gridBtn=document.getElementById('gridBtn');
if(gridBtn){gridBtn.classList.add('on');gridBtn.addEventListener('click',function(){grid.visible=!grid.visible;this.classList.toggle('on',grid.visible);this.style.background=grid.visible?'#f9c30022':'';this.style.borderColor=grid.visible?'#f9c30055':'';this.style.color=grid.visible?'#f9c300':'';});gridBtn.style.background='#f9c30022';gridBtn.style.borderColor='#f9c30055';gridBtn.style.color='#f9c300';}
const wireBtn=document.getElementById('wireBtn');
if(wireBtn)wireBtn.addEventListener('click',function(){wireMode=!wireMode;this.style.background=wireMode?'#f9c30022':'';this.style.borderColor=wireMode?'#f9c30055':'';this.style.color=wireMode?'#f9c300':'';allMeshes.forEach(m=>{if(m.material&&m.material.wireframe!==undefined)m.material.wireframe=wireMode;});});
const axisBtn=document.getElementById('axisBtn');
if(axisBtn)axisBtn.addEventListener('click',function(){axisMode=!axisMode;this.style.background=axisMode?'#f9c30022':'';this.style.borderColor=axisMode?'#f9c30055':'';this.style.color=axisMode?'#f9c300':'';allAxes.forEach(a=>a.visible=axisMode);});
const ssBtn=document.getElementById('ssBtn');
if(ssBtn)ssBtn.addEventListener('click',()=>{renderer.render(scene,camera);const a=document.createElement('a');a.download='g1_viewer.png';a.href=cv.toDataURL('image/png');a.click();});

// === 토스트 ===
let toastTimer;
function toast(msg, isError=false){
  const el=document.getElementById('toast');
  el.textContent=msg;
  el.className='show'+(isError?' error':'');
  clearTimeout(toastTimer);
  toastTimer=setTimeout(()=>el.className='',2000);
}

// === 속도 슬라이더 ===
const speedSlider=document.getElementById('speed');
const speedVal=document.getElementById('speedVal');
speedSlider.oninput=()=>speedVal.textContent=parseFloat(speedSlider.value).toFixed(2);

// === 모션 리스트 ===
async function refreshMotions(){
  const list=document.getElementById('motionList');
  try{
    const r=await fetch('/motions');
    const d=await r.json();
    if(!d.motions.length){
      list.innerHTML='<div class="empty-msg">motions/ 폴더에<br>.json 파일을 넣으세요</div>';
      return;
    }
    list.innerHTML=d.motions.map(m=>`
      <div class="motion-item">
        <span class="name" title="${m}">${m}</span>
        <button class="run-btn" onclick="runMotion('${m.replace(/'/g,"\\'")}')">▶</button>
      </div>
    `).join('');
  }catch(e){
    list.innerHTML='<div class="empty-msg" style="color:#F85149">서버 연결 실패</div>';
  }
}

async function runMotion(name){
  try{
    const r=await fetch(`/motions/run/${encodeURIComponent(name)}`,{method:'POST'});
    const d=await r.json();
    if(!r.ok)throw new Error(d.detail||'실행 실패');
    toast(`▶ ${name} (${d.frames} 프레임)`);
  }catch(e){
    toast(`오류: ${e.message}`,true);
  }
}

async function sendGift(){
  try{
    const r=await fetch('/send_gift',{method:'POST'});
    const d=await r.json();
    if(!r.ok)throw new Error(d.detail||'실행 실패');
    toast('🎁 Send 시퀀스 시작');
  }catch(e){
    toast(`오류: ${e.message}`,true);
  }
}

async function stopMotion(){
  await fetch('/stop',{method:'POST'});
  toast('⏹ 정지');
}

async function goHome(){
  await fetch('/home',{method:'POST'});
  toast('🏠 홈 이동');
}

// === 실행 상태 폴링 ===
async function pollRunStatus(){
  try{
    const s=await(await fetch('/status')).json();
    const rb=document.getElementById('runBadge');
    if(rb)rb.style.display=s.is_running?'':'none';
  }catch{}
}
setInterval(pollRunStatus,800);

// === Loco 리모컨 ===
const cmdMap={
  forward:    ()=>({vx:+speedSlider.value,vy:0,vyaw:0}),
  backward:   ()=>({vx:-speedSlider.value,vy:0,vyaw:0}),
  left:       ()=>({vx:0,vy:+speedSlider.value,vyaw:0}),
  right:      ()=>({vx:0,vy:-speedSlider.value,vyaw:0}),
  turn_left:  ()=>({vx:0,vy:0,vyaw:+speedSlider.value}),
  turn_right: ()=>({vx:0,vy:0,vyaw:-speedSlider.value}),
};

let locoTimer=null;
let activeBtn=null;

function startLoco(cmd,btn){
  if(locoTimer)return;
  activeBtn=btn;
  if(btn)btn.classList.add('active');
  const send=()=>fetch('/loco/move',{
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify(cmdMap[cmd]())
  }).catch(()=>{});
  send();
  locoTimer=setInterval(send,50);
}

function stopLoco(){
  if(locoTimer){clearInterval(locoTimer);locoTimer=null;}
  if(activeBtn){activeBtn.classList.remove('active');activeBtn=null;}
  fetch('/loco/stop',{method:'POST'}).catch(()=>{});
}

function locoStopBtn(){
  stopLoco();
  toast('⏹ 이동 정지');
}

document.querySelectorAll('.remote-pad button[data-cmd]').forEach(btn=>{
  const cmd=btn.dataset.cmd;
  btn.addEventListener('mousedown', e=>{e.preventDefault();startLoco(cmd,btn);});
  btn.addEventListener('mouseup',   e=>{e.preventDefault();stopLoco();});
  btn.addEventListener('mouseleave',()=>stopLoco());
  btn.addEventListener('touchstart',e=>{e.preventDefault();startLoco(cmd,btn);},{passive:false});
  btn.addEventListener('touchend',  e=>{e.preventDefault();stopLoco();});
  btn.addEventListener('touchcancel',()=>stopLoco());
});

const keyMap={
  'ArrowUp':'forward','ArrowDown':'backward','ArrowLeft':'left','ArrowRight':'right',
  'q':'turn_left','Q':'turn_left','e':'turn_right','E':'turn_right',
};
document.addEventListener('keydown',e=>{
  if(e.repeat)return;
  if(['INPUT','SELECT','TEXTAREA'].includes(e.target.tagName))return;
  const cmd=keyMap[e.key];
  if(cmd){e.preventDefault();startLoco(cmd,document.querySelector(`[data-cmd="${cmd}"]`));}
});
document.addEventListener('keyup',e=>{if(keyMap[e.key]){e.preventDefault();stopLoco();}});
window.addEventListener('beforeunload',stopLoco);

// 초기화
refreshMotions();
pollRunStatus();
</script>
</body>
</html>"""


# ==========================================
# /robot-only - 로봇 + Joint States 만 (dashboard.py 스타일)
# ==========================================
ROBOT_ONLY_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>G1 Robot Viewer</title>
<script src="/vendor/three.min.js"></script>
<style>
""" + VIEWER_CORE_CSS + r"""
header{display:none}
.main{flex:1}
.right{width:240px}
</style>
</head>
<body>
<div class="main">
  <div class="viewport" id="vp">
    <canvas id="cv"></canvas>
    <div class="load-overlay" id="loadOv">
      <div class="load-title">Loading model...</div>
      <div class="pbar-bg"><div class="pbar-fill" id="pbar"></div></div>
      <div class="pbar-text" id="ptext">Loading URDF...</div>
    </div>
  </div>

  <div class="right">
    <div class="right-head">
      <b>Joint States</b>
      <input class="search-box" id="searchBox" placeholder="Search..." type="text">
      <span class="jcount" id="jcount">-</span>
    </div>
    <div class="joints" id="jointsEl">
      <div class="no-joint">Loading...</div>
    </div>
  </div>
</div>

<script>
""" + VIEWER_CORE_JS + r"""
</script>
</body>
</html>"""


# ==========================================
# 페이지 라우트
# ==========================================
@app.get("/", include_in_schema=False)
async def index():
    return HTMLResponse(INDEX_HTML)


@app.get("/robot-only", include_in_schema=False)
async def robot_only():
    return HTMLResponse(ROBOT_ONLY_HTML)


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=50003)
