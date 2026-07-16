import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI
from fastapi.responses import StreamingResponse
import pyrealsense2 as rs
from pyzbar.pyzbar import decode

app = FastAPI()

# ── ArUco 설정 ──────────────────────────────────────────────
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

MARKER_LENGTH = 0.035  # 3.5cm

# ── 상자 치수 ─────────────────────────────────────────────────
HALF_W = 0.27 / 2   # 13.5cm (X축, 27cm 방향)
HALF_D = 0.09 / 2   #  4.5cm (Y축, 9cm 방향)
HEIGHT  = 0.09       #  9cm   (Z축, 아래 방향)

# ── 3D 박스 꼭짓점 (마커 좌표계 기준) ───────────────────────
BOX_CORNERS_3D = np.array([
    # 윗면 (TL, TR, BR, BL)
    [-HALF_W, -HALF_D,      0],
    [ HALF_W, -HALF_D,      0],
    [ HALF_W,  HALF_D,      0],
    [-HALF_W,  HALF_D,      0],
    # 아랫면
    [-HALF_W, -HALF_D, -HEIGHT],
    [ HALF_W, -HALF_D, -HEIGHT],
    [ HALF_W,  HALF_D, -HEIGHT],
    [-HALF_W,  HALF_D, -HEIGHT],
], dtype=np.float32)

# 엣지 정의 (인덱스 쌍)
BOX_EDGES = [
    (0, 1), (1, 2), (2, 3), (3, 0),  # 윗면
    (4, 5), (5, 6), (6, 7), (7, 4),  # 아랫면
    (0, 4), (1, 5), (2, 6), (3, 7),  # 기둥
]

# solvePnP용 마커 꼭짓점 (ArUco 표준: TL, TR, BR, BL)
MARKER_OBJ_PTS = np.array([
    [-MARKER_LENGTH / 2,  MARKER_LENGTH / 2, 0],
    [ MARKER_LENGTH / 2,  MARKER_LENGTH / 2, 0],
    [ MARKER_LENGTH / 2, -MARKER_LENGTH / 2, 0],
    [-MARKER_LENGTH / 2, -MARKER_LENGTH / 2, 0],
], dtype=np.float32)

# ── RealSense 초기화 ─────────────────────────────────────────
pipeline = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

try:
    profile = pipeline.start(cfg)
except Exception as e:
    print(f"RealSense 시작 실패: {e}")
    exit(1)

# RealSense intrinsics 추출
intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
camera_matrix = np.array([
    [intr.fx,       0, intr.ppx],
    [      0, intr.fy, intr.ppy],
    [      0,       0,         1],
], dtype=np.float64)
dist_coeffs = np.array(intr.coeffs, dtype=np.float64)


# ── 헬퍼 ─────────────────────────────────────────────────────
def estimate_pose(corners_i):
    success, rvec, tvec = cv2.solvePnP(
        MARKER_OBJ_PTS, corners_i[0],
        camera_matrix, dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE
    )
    return success, rvec, tvec


def draw_box_3d(frame, rvec, tvec):
    pts, _ = cv2.projectPoints(
        BOX_CORNERS_3D, rvec, tvec, camera_matrix, dist_coeffs
    )
    pts = pts.reshape(-1, 2).astype(int)

    # 윗면 반투명 채우기
    top_face = pts[:4].reshape((-1, 1, 2))
    overlay = frame.copy()
    cv2.fillPoly(overlay, [top_face], (0, 255, 255))
    cv2.addWeighted(overlay, 0.2, frame, 0.8, 0, frame)

    # 엣지 그리기
    for i, j in BOX_EDGES:
        cv2.line(frame, tuple(pts[i]), tuple(pts[j]), (0, 255, 255), 2)

    # 꼭짓점 점
    for pt in pts[:4]:
        cv2.circle(frame, tuple(pt), 4, (0, 200, 255), -1)


# ── 스트리밍 ──────────────────────────────────────────────────
def generate_frames():
    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            frame = np.asanyarray(color_frame.get_data())

            # QR 코드 인식
            for obj in decode(frame):
                data = obj.data.decode('utf-8')
                pts = np.array(obj.polygon, np.int32).reshape((-1, 1, 2))
                cv2.polylines(frame, [pts], True, (0, 255, 0), 3)
                cv2.putText(frame, data, (obj.rect.left, obj.rect.top - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            # ArUco 인식 + Pose + 3D 박스
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco_detector.detectMarkers(gray)

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                for i, marker_id in enumerate(ids.flatten()):
                    success, rvec, tvec = estimate_pose(corners[i:i+1])
                    if not success:
                        continue

                    # 좌표축 (3cm)
                    cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs,
                                      rvec, tvec, 0.03)
                    # 3D 박스
                    draw_box_3d(frame, rvec, tvec)

                    # ID + 거리 표시
                    c = corners[i][0]
                    cx, cy = int(c[:, 0].mean()), int(c[:, 1].mean())
                    dist_m = np.linalg.norm(tvec)
                    cv2.putText(frame, f"ID:{marker_id}  {dist_m*100:.1f}cm",
                                (cx - 30, cy - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
    finally:
        pipeline.stop()


@app.get('/')
def index():
    return {"status": "running", "mode": "QR + ArUco + 3D Box"}

@app.get('/video_feed')
def video_feed():
    return StreamingResponse(generate_frames(),
                             media_type='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
