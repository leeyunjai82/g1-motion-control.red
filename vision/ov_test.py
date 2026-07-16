import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI
from fastapi.responses import StreamingResponse
import openvino as ov

app = FastAPI()

# 1. OpenVINO Core 초기화
core = ov.Core()

# 모델 경로 설정
DETECTION_MODEL = "ov_models/detection/face-detection-retail-0004.xml"
AGE_GENDER_MODEL = "ov_models/age-gender/age-gender-recognition-retail-0013.xml"
EMOTION_MODEL = "ov_models/emotion/emotions-recognition-retail-0003.xml"

# 감정 레이블 정의 (Retail-0003 모델 기준)
EMOTION_LABELS = ['neutral', 'happy', 'sad', 'surprise', 'anger']

try:
    # --- 모델 로드 및 컴파일 ---
    # 1. Face Detection
    model_det = core.read_model(DETECTION_MODEL)
    compiled_det = core.compile_model(model_det, "CPU")
    input_det = compiled_det.input(0)
    output_det = compiled_det.output(0)
    _, _, H_det, W_det = input_det.shape

    # 2. Age-Gender Recognition
    model_ag = core.read_model(AGE_GENDER_MODEL)
    compiled_ag = core.compile_model(model_ag, "CPU")
    input_ag = compiled_ag.input(0)
    # 출력 레이어 2개: age_conv3, prob (gender)
    out_age = compiled_ag.output("age_conv3")
    out_gender = compiled_ag.output("prob")
    _, _, H_ag, W_ag = input_ag.shape

    # 3. Emotion Recognition
    model_emo = core.read_model(EMOTION_MODEL)
    compiled_emo = core.compile_model(model_emo, "CPU")
    input_emo = compiled_emo.input(0)
    output_emo = compiled_emo.output(0)
    _, _, H_emo, W_emo = input_emo.shape

except Exception as e:
    print(f"모델 로드 오류: {e}")
    exit(1)

cap = cv2.VideoCapture(0)

def generate_frames():
    while True:
        success, frame = cap.read()
        if not success: break

        h, w, _ = frame.shape

        # --- [Step 1] Face Detection ---
        resized_det = cv2.resize(frame, (W_det, H_det))
        input_data_det = np.expand_dims(resized_det.transpose(2, 0, 1), 0)
        det_results = compiled_det([input_data_det.astype(np.float32)])[output_det]
        detections = det_results[0][0]

        for detection in detections:
            confidence = detection[2]
            if confidence > 0.5:
                # 좌표 계산 (이미지 범위를 벗어나지 않도록 clip)
                xmin = max(0, int(detection[3] * w))
                ymin = max(0, int(detection[4] * h))
                xmax = min(w, int(detection[5] * w))
                ymax = min(h, int(detection[6] * h))

                # 얼굴 영역 크롭 (ROI)
                face_roi = frame[ymin:ymax, xmin:xmax]
                if face_roi.size == 0: continue

                # --- [Step 2] Age & Gender Inference ---
                resized_ag = cv2.resize(face_roi, (W_ag, H_ag))
                input_ag_data = np.expand_dims(resized_ag.transpose(2, 0, 1), 0)
                ag_results = compiled_ag([input_ag_data.astype(np.float32)])
                
                # 나이: age_conv3의 값에 100을 곱함
                age = int(ag_results[out_age][0][0][0][0] * 100)
                # 성별: [0, 1] -> 0: Female, 1: Male (softmax 결과값)
                gender_prob = ag_results[out_gender][0]
                gender = "Female" if gender_prob[0] > gender_prob[1] else "Male"

                # --- [Step 3] Emotion Inference ---
                resized_emo = cv2.resize(face_roi, (W_emo, H_emo))
                input_emo_data = np.expand_dims(resized_emo.transpose(2, 0, 1), 0)
                emo_results = compiled_emo([input_emo_data.astype(np.float32)])[output_emo]
                emotion = EMOTION_LABELS[np.argmax(emo_results[0])]

                # --- 시각화 ---
                label = f"{gender}, {age}s, {emotion}"
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                cv2.putText(frame, label, (xmin, ymin - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # 스트리밍 전송
        ret, buffer = cv2.imencode('.jpg', frame)
        if not ret: continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.get('/')
def index():
    return {"status": "Multi-Model Inference Running", "models": ["Detection", "Age-Gender", "Emotion"]}

@app.get('/video_feed')
def video_feed():
    return StreamingResponse(generate_frames(),
                             media_type='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
