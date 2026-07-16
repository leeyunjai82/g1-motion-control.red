import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI
from fastapi.responses import StreamingResponse
import openvino as ov
import pyrealsense2 as rs  # RealSense 패키지 추가

app = FastAPI()

# 1. OpenVINO 및 모델 초기화
core = ov.Core()
DETECTION_MODEL = "ov_models/detection/face-detection-retail-0004.xml"
AGE_GENDER_MODEL = "ov_models/age-gender/age-gender-recognition-retail-0013.xml"
EMOTION_MODEL = "ov_models/emotion/emotions-recognition-retail-0003.xml"
EMOTION_LABELS = ['neutral', 'happy', 'sad', 'surprise', 'anger']

try:
    # 모델 로드 (Face, Age/Gender, Emotion)
    compiled_det = core.compile_model(core.read_model(DETECTION_MODEL), "CPU")
    input_det = compiled_det.input(0)
    output_det = compiled_det.output(0)
    _, _, H_det, W_det = input_det.shape

    compiled_ag = core.compile_model(core.read_model(AGE_GENDER_MODEL), "CPU")
    out_age, out_gender = compiled_ag.output("age_conv3"), compiled_ag.output("prob")
    _, _, H_ag, W_ag = compiled_ag.input(0).shape

    compiled_emo = core.compile_model(core.read_model(EMOTION_MODEL), "CPU")
    output_emo = compiled_emo.output(0)
    _, _, H_emo, W_emo = compiled_emo.input(0).shape
except Exception as e:
    print(f"모델 로드 오류: {e}"); exit(1)

# --- 2. RealSense 파이프라인 설정 ---
pipeline = rs.pipeline()
config = rs.config()
# 컬러 스트림 활성화 (640x480, 30fps)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 스트리밍 시작
pipeline.start(config)

def generate_frames():
    try:
        while True:
            # RealSense 프레임 대기 및 수신
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # RealSense 프레임을 numpy array로 변환
            frame = np.asanyarray(color_frame.get_data())
            h, w, _ = frame.shape

            # --- OpenVINO 추론 (이전과 동일) ---
            resized_det = cv2.resize(frame, (W_det, H_det))
            input_data_det = np.expand_dims(resized_det.transpose(2, 0, 1), 0)
            det_results = compiled_det([input_data_det.astype(np.float32)])[output_det]
            
            for detection in det_results[0][0]:
                confidence = detection[2]
                if confidence > 0.5:
                    xmin, ymin = max(0, int(detection[3] * w)), max(0, int(detection[4] * h))
                    xmax, ymax = min(w, int(detection[5] * w)), min(h, int(detection[6] * h))
                    
                    face_roi = frame[ymin:ymax, xmin:xmax]
                    if face_roi.size == 0: continue

                    # Age-Gender
                    resized_ag = cv2.resize(face_roi, (W_ag, H_ag))
                    ag_results = compiled_ag([np.expand_dims(resized_ag.transpose(2, 0, 1), 0).astype(np.float32)])
                    age = int(ag_results[out_age][0][0][0][0] * 100)
                    gender = "Female" if ag_results[out_gender][0][0] > ag_results[out_gender][0][1] else "Male"

                    # Emotion
                    resized_emo = cv2.resize(face_roi, (W_emo, H_emo))
                    emo_results = compiled_emo([np.expand_dims(resized_emo.transpose(2, 0, 1), 0).astype(np.float32)])[output_emo]
                    emotion = EMOTION_LABELS[np.argmax(emo_results[0])]

                    # 그리기
                    label = f"{gender}, {age}s, {emotion}"
                    cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                    cv2.putText(frame, label, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # 스트리밍 인코딩
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret: continue
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
    finally:
        # 프로그램 종료 시 파이프라인 정지
        pipeline.stop()

@app.get('/')
def index():
    return {"message": "RealSense Multi-Model Inference Running"}

@app.get('/video_feed')
def video_feed():
    return StreamingResponse(generate_frames(), media_type='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
