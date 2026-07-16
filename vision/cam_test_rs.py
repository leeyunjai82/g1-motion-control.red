import cv2
import numpy as np
import pyrealsense2 as rs
import uvicorn
from fastapi import FastAPI
from fastapi.responses import StreamingResponse

app = FastAPI()

# RealSense D435i 파이프라인 설정
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

# depth -> color 정렬
align = rs.align(rs.stream.color)


def generate_frames():
    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)

            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # depth 컬러맵 (시각화)
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03),
                cv2.COLORMAP_JET
            )

            # color + depth 가로로 합치기
            combined = np.hstack((color_image, depth_colormap))

            cv2.putText(combined, "RealSense D435i Streaming...", (20, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            ret, buffer = cv2.imencode('.jpg', combined)
            if not ret:
                continue

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
    except GeneratorExit:
        pass


@app.get('/')
def index():
    return {"status": "ok", "message": "Go to /video_feed to see the camera"}


@app.get('/video_feed')
def video_feed():
    return StreamingResponse(generate_frames(),
                             media_type='multipart/x-mixed-replace; boundary=frame')


@app.on_event("shutdown")
def shutdown_event():
    pipeline.stop()


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
