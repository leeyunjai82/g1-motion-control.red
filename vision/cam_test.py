import cv2
import uvicorn
from fastapi import FastAPI
from fastapi.responses import StreamingResponse

app = FastAPI()

# 웹캠 설정 (0번은 기본 내장 카메라)
cap = cv2.VideoCapture(0)

def generate_frames():
    while True:
        # 프레임 읽기
        success, frame = cap.read()
        
        if not success:
            print("카메라를 찾을 수 없거나 프레임을 읽을 수 없습니다.")
            break
        else:
            # 테스트용: 화면이 정상인지 확인하기 위해 현재 시간을 프레임에 기록 (옵션)
            cv2.putText(frame, "Streaming...", (20, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # 프레임을 JPEG 포맷으로 인코딩
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                continue
                
            frame_bytes = buffer.tobytes()

            # MJPEG 스트리밍 규격에 맞춰 전송
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.get('/')
def index():
    return {"status": "ok", "message": "Go to /video_feed to see the camera"}

@app.get('/video_feed')
def video_feed():
    # StreamingResponse로 실시간 프레임 전달
    return StreamingResponse(generate_frames(), 
                             media_type='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    # 8000번 포트로 서버 실행
    uvicorn.run(app, host="0.0.0.0", port=8000)
