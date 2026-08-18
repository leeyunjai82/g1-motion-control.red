#!/usr/bin/env python3
"""
nav_server.py — G1 SLAM 내비게이션 서버 + 웹 콘솔 (포트 50030)

포트 구성
    50000  robot_server.py   (arm_sdk, 파지)
    50010  detect_box.py
    50020  loco_test.py
    50030  nav_server.py     <- 이 파일

실행 (ROS 환경 없는 쉘에서)
    ./start_nav.sh eth0
    브라우저에서 http://<PC IP>:50030

주의: 이동 전 robot_server 의 arm_sdk 를 release 하세요.
      POST http://localhost:50000/arm_release
"""

import argparse
import asyncio
import json
import math
import os
import time
from pathlib import Path

import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, JSONResponse
from pydantic import BaseModel

from unitree_sdk2py.core.channel import ChannelFactoryInitialize

from slam_client import DEFAULT_PCD, Pose, SlamClient

WAYPOINT_FILE = Path(os.environ.get("NAV_WAYPOINTS", "waypoints.json"))
ARRIVAL_LOG = Path(os.environ.get("NAV_ARRIVAL_LOG", "arrivals.jsonl"))
WEB_FILE = Path(__file__).with_name("nav_web.html")
NAV_TIMEOUT = float(os.environ.get("NAV_TIMEOUT", "180"))
SETTLE_SEC = float(os.environ.get("NAV_SETTLE_SEC", "1.5"))

app = FastAPI(title="G1 Nav Server")


def wrap_deg(rad):
    return math.degrees(math.atan2(math.sin(rad), math.cos(rad)))


class NavState:
    def __init__(self):
        self.client: SlamClient | None = None
        self.mode = "idle"          # idle | mapping | moving | error
        self.label: str | None = None
        self.started_at: float | None = None
        self.last_error: str | None = None
        self.task: asyncio.Task | None = None
        self.arrivals: list[dict] = []
        self.repeat = None
        self.relocated = False

    def snapshot(self):
        pose, _ = self.client.get_pose()
        return {
            "mode": self.mode,
            "label": self.label,
            "elapsed_sec": round(time.time() - self.started_at, 1) if self.started_at else None,
            "pose": {"x": pose.x, "y": pose.y, "yaw_deg": round(math.degrees(pose.yaw), 1)},
            "pose_fresh": self.client.pose_is_fresh(),
            "relocated": self.relocated,
            "last_error": self.last_error,
            "arrivals": self.arrivals[-10:],
            "repeat": self.repeat,
            "busy": self.mode in ("moving", "mapping"),
        }


state = NavState()


def load_waypoints() -> dict:
    if WAYPOINT_FILE.exists():
        try:
            return json.loads(WAYPOINT_FILE.read_text())
        except json.JSONDecodeError:
            return {}
    return {}


def save_waypoints(wps: dict):
    WAYPOINT_FILE.write_text(json.dumps(wps, indent=2, ensure_ascii=False))


def record_arrival(rec: dict):
    state.arrivals.append(rec)
    del state.arrivals[:-20]
    try:
        with ARRIVAL_LOG.open("a") as f:
            f.write(json.dumps(rec, ensure_ascii=False) + "\n")
    except OSError:
        pass


async def navigate(label: str, target: Pose) -> dict | None:
    """한 번 이동하고 정지 오차를 기록. 실패하면 None."""
    state.mode = "moving"
    state.label = label
    state.started_at = time.time()
    state.last_error = None

    loop = asyncio.get_running_loop()
    ok, msg = await loop.run_in_executor(
        None, lambda: state.client.goto_and_wait(target, timeout=NAV_TIMEOUT)
    )
    sec = round(time.time() - state.started_at, 1)

    if not ok:
        state.mode = "error"
        state.last_error = msg
        return None

    await asyncio.sleep(SETTLE_SEC)      # 완전히 멈춘 뒤 읽기
    actual, _ = state.client.get_pose()
    rec = {
        "t": time.strftime("%H:%M:%S"),
        "label": label,
        "target": {"x": round(target.x, 3), "y": round(target.y, 3),
                   "yaw_deg": round(math.degrees(target.yaw), 1)},
        "dx_cm": round((actual.x - target.x) * 100, 1),
        "dy_cm": round((actual.y - target.y) * 100, 1),
        "dist_cm": round(actual.distance_to(target) * 100, 1),
        "dyaw_deg": round(wrap_deg(actual.yaw - target.yaw), 1),
        "sec": sec,
    }
    record_arrival(rec)
    state.mode = "idle"
    state.label = None
    return rec


async def run_single(label: str, target: Pose):
    try:
        await navigate(label, target)
    except asyncio.CancelledError:
        state.client.pause_nav()
        state.mode = "idle"
        state.label = None
        raise
    except Exception as exc:  # noqa: BLE001
        state.mode = "error"
        state.last_error = repr(exc)


async def run_repeat(name: str, target: Pose, home: Pose, n: int):
    """홈 <-> 목표 왕복 n회. 목표 지점 정지 오차 분포를 냅니다."""
    state.repeat = {"name": name, "n": n, "done": 0, "samples": [],
                    "summary": None, "running": True}
    try:
        for i in range(n):
            rec = await navigate(f"{name} ({i+1}/{n})", target)
            if rec is None:
                break
            state.repeat["samples"].append(rec)
            state.repeat["done"] = i + 1
            if await navigate(f"홈 복귀 ({i+1}/{n})", home) is None:
                break
        state.repeat["summary"] = summarize(state.repeat["samples"])
    except asyncio.CancelledError:
        state.client.pause_nav()
        state.mode = "idle"
        raise
    except Exception as exc:  # noqa: BLE001
        state.mode = "error"
        state.last_error = repr(exc)
    finally:
        if state.repeat:
            state.repeat["running"] = False


def summarize(samples: list[dict]) -> dict | None:
    if not samples:
        return None
    out = {"n": len(samples)}
    for key in ("dx_cm", "dy_cm", "dist_cm", "dyaw_deg"):
        vals = [s[key] for s in samples]
        mean = sum(vals) / len(vals)
        var = sum((v - mean) ** 2 for v in vals) / len(vals)
        out[key] = {"mean": round(mean, 2), "sd": round(math.sqrt(var), 2),
                    "min": round(min(vals), 2), "max": round(max(vals), 2)}
    worst_dist = max(s["dist_cm"] for s in samples)
    worst_yaw = max(abs(s["dyaw_deg"]) for s in samples)
    out["worst_dist_cm"] = round(worst_dist, 1)
    out["worst_yaw_deg"] = round(worst_yaw, 1)
    out["marker_needed"] = not (worst_dist <= 5.0 and worst_yaw <= 10.0)
    return out


def ensure_free():
    if state.mode in ("moving", "mapping"):
        raise HTTPException(409, f"{state.mode} 중입니다. 먼저 중단하세요.")


def ensure_located():
    if not state.client.pose_is_fresh():
        raise HTTPException(503, "측위가 안 된 상태입니다. 측위 초기화를 먼저 하세요.")


class PoseIn(BaseModel):
    x: float
    y: float
    yaw_deg: float = 0.0

    def to_pose(self) -> Pose:
        return Pose.from_yaw(self.x, self.y, math.radians(self.yaw_deg))


class RelocIn(PoseIn):
    address: str = DEFAULT_PCD


class MappingEnd(BaseModel):
    address: str = DEFAULT_PCD


class RepeatIn(BaseModel):
    name: str
    n: int = 5


@app.get("/", response_class=HTMLResponse)
async def index():
    if WEB_FILE.exists():
        return WEB_FILE.read_text()
    return "<h1>nav_server</h1><p>nav_web.html 이 없습니다.</p>"


@app.get("/status")
async def status():
    return state.snapshot()


@app.get("/waypoints")
async def list_waypoints():
    return load_waypoints()


@app.post("/waypoints/{name}")
async def put_waypoint(name: str, p: PoseIn):
    wps = load_waypoints()
    wps[name] = p.to_pose().to_dict()
    save_waypoints(wps)
    return {"ok": True, "name": name, "pose": wps[name]}


@app.post("/waypoints/{name}/capture")
async def capture_waypoint(name: str):
    """로봇을 원하는 자리에 세워두고 호출 — 현재 위치를 그 이름으로 저장."""
    ensure_located()
    pose, _ = state.client.get_pose()
    wps = load_waypoints()
    wps[name] = pose.to_dict()
    save_waypoints(wps)
    return {"ok": True, "name": name, "pose": wps[name]}


@app.delete("/waypoints/{name}")
async def delete_waypoint(name: str):
    wps = load_waypoints()
    if name not in wps:
        raise HTTPException(404, f"지점 '{name}' 없음")
    wps.pop(name)
    save_waypoints(wps)
    return {"ok": True}


@app.post("/goto/{name}")
async def goto(name: str):
    ensure_free()
    ensure_located()
    wps = load_waypoints()
    if name not in wps:
        raise HTTPException(404, f"지점 '{name}' 없음")
    state.task = asyncio.create_task(run_single(name, Pose.from_dict(wps[name])))
    return {"ok": True, "goal": name}


@app.post("/goto_pose")
async def goto_pose(p: PoseIn):
    """지도에서 직접 찍은 좌표로 이동."""
    ensure_free()
    ensure_located()
    state.task = asyncio.create_task(run_single("지도 지정", p.to_pose()))
    return {"ok": True}


@app.post("/repeat")
async def repeat(r: RepeatIn):
    """현재 위치를 홈으로 삼아 지정 지점까지 n회 왕복하며 정지 오차를 측정."""
    ensure_free()
    ensure_located()
    wps = load_waypoints()
    if r.name not in wps:
        raise HTTPException(404, f"지점 '{r.name}' 없음")
    if not 1 <= r.n <= 20:
        raise HTTPException(400, "반복 횟수는 1~20 사이여야 합니다.")
    home, _ = state.client.get_pose()
    state.task = asyncio.create_task(
        run_repeat(r.name, Pose.from_dict(wps[r.name]), home, r.n))
    return {"ok": True}


@app.post("/relocation")
async def relocation(r: RelocIn):
    """pcd 맵을 불러오고 현재 위치를 알려줍니다. 이동 전 필수."""
    ensure_free()
    code, data = state.client.start_relocation(r.to_pose(), r.address)
    state.relocated = code == 0
    return {"ok": code == 0, "code": code, "data": str(data)}


@app.post("/mapping/start")
async def mapping_start():
    ensure_free()
    code, _ = state.client.start_mapping("indoor")
    if code == 0:
        state.mode = "mapping"
    return {"ok": code == 0, "code": code}


@app.post("/mapping/end")
async def mapping_end(m: MappingEnd):
    code, _ = state.client.end_mapping(m.address)
    if code == 0:
        state.mode = "idle"
    return {"ok": code == 0, "code": code, "address": m.address}


@app.post("/pause")
async def pause():
    code, _ = state.client.pause_nav()
    return {"ok": code == 0, "code": code}


@app.post("/resume")
async def resume():
    code, _ = state.client.resume_nav()
    return {"ok": code == 0, "code": code}


@app.post("/cancel")
async def cancel():
    """주행 중단. 비상정지는 robot_server(50000) 의 정지를 쓰세요."""
    state.client.pause_nav()
    if state.task and not state.task.done():
        state.task.cancel()
    if state.repeat:
        state.repeat["running"] = False
    state.mode = "idle"
    state.label = None
    return {"ok": True}


@app.post("/stop_node")
async def stop_node():
    """SLAM 노드 종료."""
    state.client.pause_nav()
    if state.task and not state.task.done():
        state.task.cancel()
    code, _ = state.client.stop_node()
    state.mode = "idle"
    state.relocated = False
    return {"ok": code == 0, "code": code}


@app.exception_handler(HTTPException)
async def http_exc(request, exc):
    return JSONResponse({"ok": False, "detail": exc.detail}, status_code=exc.status_code)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--iface", default="eth0")
    p.add_argument("--port", type=int, default=50030)
    p.add_argument("--pcd", default=DEFAULT_PCD)
    args = p.parse_args()

    ChannelFactoryInitialize(0, args.iface)
    state.client = SlamClient(pcd_path=args.pcd)
    state.client.Init()
    print(f"[nav_server] slam_operate 연결. pcd={args.pcd}")

    uvicorn.run(app, host="0.0.0.0", port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
