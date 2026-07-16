import uvicorn
import asyncio
import os
import time
import json
from contextlib import asynccontextmanager
from typing import List, Optional

from fastapi import FastAPI
from fastapi.responses import HTMLResponse, FileResponse
from fastapi import UploadFile, File
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from g1_motor_high import Custom, LOCO_DIRECTION_MAP, LOCO_LINEAR_SPEED, LOCO_ANGULAR_SPEED

FRAME_TIME_MARGIN = 1.2

custom: Optional[Custom] = None
stop_event = asyncio.Event()


class MotorCommand(BaseModel):
    motor_index: int
    target_degree: float
    duration: float

class LocoCommand(BaseModel):
    direction: str
    linear_speed: float = LOCO_LINEAR_SPEED
    angular_speed: float = LOCO_ANGULAR_SPEED

class MotorTarget(BaseModel):
    motor_index: int
    target_degree: float

class PoseData(BaseModel):
    targets: List[MotorTarget]

class LocomotionData(BaseModel):
    direction: str
    linear_speed: float = LOCO_LINEAR_SPEED
    angular_speed: float = LOCO_ANGULAR_SPEED

class MotionFrame(BaseModel):
    duration: float
    pose: Optional[PoseData] = None
    locomotion: Optional[LocomotionData] = None


def resolve_velocity(direction: str, linear_speed: float, angular_speed: float):
    dx, dy, dyaw = LOCO_DIRECTION_MAP.get(direction, (0.0, 0.0, 0.0))
    return dx * linear_speed, dy * linear_speed, dyaw * angular_speed


async def emergency_stop():
    if not custom:
        return
    custom.execute_loco_command("Move", 0.0, 0.0, 0.0)
    loop = asyncio.get_running_loop()
    await asyncio.gather(*[
        loop.run_in_executor(None, custom.command_new_move, i, 0.0, 1.0)
        for i in custom.arm_joints
    ])


@asynccontextmanager
async def lifespan(app: FastAPI):
    global custom
    custom = Custom(interface="eth0")
    custom.Init()
    custom.Start()
    await asyncio.sleep(5)
    await emergency_stop()
    yield


app = FastAPI(lifespan=lifespan)
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_credentials=True, allow_methods=["*"], allow_headers=["*"])


@app.post("/set_motor")
async def set_motor(command: MotorCommand):
    loop = asyncio.get_running_loop()
    loop.run_in_executor(None, custom.command_new_move, command.motor_index, command.target_degree, command.duration)
    return {"status": "success"}


@app.post("/set_loco_motion")
async def set_loco_motion(command: LocoCommand):
    vx, vy, vyaw = resolve_velocity(command.direction, command.linear_speed, command.angular_speed)
    custom.execute_loco_command("Move", vx, vy, vyaw)
    return {"status": "success", "direction": command.direction}


@app.post("/set_motion")
async def set_motion(motion_sequence: List[MotionFrame]):
    stop_event.clear()
    loop = asyncio.get_running_loop()

    for i, frame in enumerate(motion_sequence):
        if stop_event.is_set():
            print(f"정지 요청 - 프레임 {i+1} 중단")
            break

        print(f"[{i+1}/{len(motion_sequence)}] duration={frame.duration}s")

        if frame.pose and frame.pose.targets:
            await asyncio.gather(*[
                loop.run_in_executor(None, custom.command_new_move, t.motor_index, t.target_degree, frame.duration)
                for t in frame.pose.targets
            ])

        if frame.locomotion:
            loco = frame.locomotion
            vx, vy, vyaw = resolve_velocity(loco.direction, loco.linear_speed, loco.angular_speed)
            start = time.time()
            while time.time() - start < frame.duration * FRAME_TIME_MARGIN:
                if stop_event.is_set():
                    break
                custom.execute_loco_command("Move", vx, vy, vyaw)
                await asyncio.sleep(0.02)
            if loco.direction != "stop" and not stop_event.is_set():
                custom.execute_loco_command("Move", 0.0, 0.0, 0.0)
        else:
            await asyncio.sleep(frame.duration * FRAME_TIME_MARGIN)

    if stop_event.is_set():
        await emergency_stop()
        stop_event.clear()
    else:
        custom.execute_loco_command("Move", 0.0, 0.0, 0.0)

    return {"status": "success"}


@app.post("/stop_motion")
async def stop_motion():
    stop_event.set()
    return {"status": "success"}

@app.post("/run_motion_file")
async def run_motion_file(file: UploadFile = File(...)):
    content = await file.read()
    data = json.loads(content)
    motion_sequence = [MotionFrame(**frame) for frame in data]
    return await set_motion(motion_sequence)

@app.get("/", response_class=HTMLResponse)
async def read_root():
    html_path = os.path.join(os.path.dirname(__file__), "simulator.html")
    if os.path.exists(html_path):
        return FileResponse(html_path)
    return HTMLResponse("<h1>Error: simulator.html not found</h1>")


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
