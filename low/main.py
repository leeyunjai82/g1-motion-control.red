import uvicorn
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse, FileResponse
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware
import os, time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from g1_motor_low import Custom

# --- Pydantic 모델 정의 ---
# API를 통해 받아들일 데이터의 형식을 지정합니다.
class MotorCommand(BaseModel):
    motor_index: int
    target_degree: float
    duration: float

# --- FastAPI 앱 생성 ---
app = FastAPI()

# --- CORS 미셔들웨어 설정 ---
# 다른 출처(Origin)의 웹페이지에서 API를 호출할 수 있도록 허용합니다.
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.on_event("startup")
async def startup_event():
    global custom
    print("--- FastAPI 서버 시작 ---")
    # 이 곳에 로봇 초기화 등의 시작 코드를 넣을 수 있습니다.
    ChannelFactoryInitialize(0, "eth0")

    custom = Custom()
    custom.Init()
    custom.Start()
    time.sleep(5)
    for i in range(29):
        custom.command_new_move(i, 0, 2.0)
    
# --- API 엔드포인트: /set_motor ---
@app.post("/set_motor")
async def set_motor(command: MotorCommand):
    """
    웹 시뮬레이터로부터 모터 제어 명령을 받는 API 엔드포인트입니다.
    이곳에 실제 로봇을 제어하는 코드를 연결할 수 있습니다.
    """
    print("="*30)
    print("API 요청 수신:")
    print(f"  - 모터 인덱스: {command.motor_index}")
    print(f"  - 목표 각도: {command.target_degree:.2f}°")
    print(f"  - 동작 시간: {command.duration:.2f}초")
    print("="*30)
    
    custom.command_new_move(command.motor_index, command.target_degree, command.duration)
    
    return {"status": "success", "received_command": command}

# --- 루트 엔드포인트: 웹 페이지 제공 ---
@app.get("/", response_class=HTMLResponse)
async def read_root():
    """
    서버의 기본 주소로 접속했을 때 index.html 파일을 반환합니다.
    """
    # main.py와 같은 위치에 index.html 파일이 있는지 확인합니다.
    html_file_path = os.path.join(os.path.dirname(__file__), "index.html")
    if os.path.exists(html_file_path):
        return FileResponse(html_file_path)
    return {"error": "index.html not found"}

# --- 서버 실행 ---
if __name__ == "__main__":
    # 터미널에서 'uvicorn main:app --reload' 명령으로 실행합니다.
    uvicorn.run(app, host="127.0.0.1", port=8000)
