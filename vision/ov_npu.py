import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI
from fastapi.responses import StreamingResponse
import openvino as ov
import pyrealsense2 as rs

app = FastAPI()

# 1. OpenVINO 및 모델 초기화
core = ov.Core()
print(f"Available devices: {core.available_devices}")

# NPU 컴파일 캐시 (첫 로드 시간 단축)
core.set_property({"CACHE_DIR": "./ov_cache"})

DEVICE = "NPU" if "NPU" in core.available_devices else "CPU"
print(f"Using device: {DEVICE}")

DETECTION_MODEL = "ov_models/detection/face-detection-retail-0004.xml"
AGE_GENDER_MODEL = "ov_models/age-gender/age-gender-recognition-retail-0013.xml"
EMOTION_MODEL = "ov_models/emotion/emotions-recognition-retail-0003.xml"
EMOTION_LABELS = ['neutral', 'happy', 'sad', 'surprise', 'anger']


def compile_with_fallback(model_path, device):
    """NPU 실패 시 CPU로 fallback"""
    model = core.read_model(model_path)
    if device == "NPU":
        try:
            return core.compile_model(model, "NPU")
        except Exception as e:
            print(f"  NPU 컴파일 실패 ({model_path}): {e}")
            print(f"  → CPU로 fallback")
            return core.compile_model(model, "CPU")
    return core.compile_model(model, device)

try:
    print("모델 컴파일 중... (NPU 첫 로드 시 시간 소요)")
    compiled_det = compile_with_fallback(DETECTION_MODEL, DEVICE)
    input_det = compiled_det.input(0)
    output_det = compiled_det.output(0)
    _, _, H_det, W_det = input_det.shape

    compiled_ag = compile_with_fallback(AGE_GENDER_MODEL, DEVICE)
    out_age, out_gender = compiled_ag.output("age_conv3"), compiled_ag.output("prob")
    _, _, H_ag, W_ag = compiled_ag.input(0).shape

    compiled_emo = compile_with_fallback(EMOTION_MODEL, DEVICE)
    output_emo = compiled_emo.output(0)
    _, _, H_emo, W_emo = compiled_emo.input(0).shape
    print("모든 모델 컴파일 완료")
except Exception as e:
    print(f"모델 로드 오류: {e}"); exit(1)

# --- 2. RealSense 파이프라인 설정 ---
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)


def generate_frames():
    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            h, w, _ = frame.shape

            # Face detection
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

                    label = f"{gender}, {age}s, {emotion}"
                    cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                    cv2.putText(frame, label, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # 디바이스 표시
            cv2.putText(frame, f"Device: {DEVICE}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret: continue
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
    finally:
        pipeline.stop()


@app.get('/')
def index():
    return {"message": f"RealSense Multi-Model Inference Running on {DEVICE}"}


@app.get('/video_feed')
def video_feed():
    return StreamingResponse(generate_frames(), media_type='multipart/x-mixed-replace; boundary=frame')


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
