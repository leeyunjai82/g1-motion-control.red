#!/usr/bin/env python3
# Version: 3.19
# Changes:
#   3.19 - 비교용 'alt' 추정 추가 (fit_top_plane_rect): 윗면 RANSAC 평면 피팅(기울기 실측) +
#          평면 좌표(미터)에서 사각형 피팅 + 경계 1셀 깎기. 기존 결과는 그대로, out['alt'] 에만 기록
#   3.18 - VERBOSE 진단로그 켬 (up/h_top/h_table/H/윗면비율)
#   3.17 - 윗면 분리 비대칭 임계(위 넉넉/아래 빡빡)로 앞면 차단
#   3.16 - 윗면 4꼭지점 minAreaRect 우선(노이즈 강함)+approx 보조
#   3.15 - TOP_THRESH ±5mm→±20mm(기울어진 박스), 최대 연결요소만 유지
"""
박스 검출 + 윗면 분리 + 외곽선 4 꼭지점 (approxPolyDP).

v3.14 변경:
  - VERBOSE_LOG 토글 추가 (상단 상수)
  - 콘솔 [BOX] 로그 on/off 가능

v3.13: approxPolyDP로 4 꼭지점
v3.12: INSET 2cm
v3.11: 비율 → 고정 거리
v3.7: minAreaRect 제거, 외곽선 극단점 사용
"""
import numpy as np
import cv2


# ==========================================
# 로그 토글
# ==========================================
VERBOSE_LOG = True    # True = [BOX] 로그 출력 (윗면 분리 진단용)


MIN_PIXELS    = 100
MIN_DEPTH_M   = 0.10
MAX_DEPTH_M   = 3.00
TOP_THRESH_M  = 0.020   # 윗면 ±20mm (기울어진 박스 포용)

# ---- alt (평면 피팅 + 3D 사각형) 파라미터 ----
ALT_BAND_DOWN  = 0.030  # 윗면 후보: h_top 아래 3cm 까지 (기울기 몰라도 윗면 전체 포함)
ALT_BAND_UP    = 0.050  #           위 5cm 까지
ALT_RANSAC_IT  = 200
ALT_RANSAC_TH  = 0.006  # 평면 inlier 거리 6mm
ALT_MAX_TILT   = 25.0   # 고정 up 과 평면 법선 차이가 이보다 크면 실패 처리 (deg)
ALT_CELL       = 0.005  # 평면 격자 5mm
ALT_INSET_M    = 0.02   # L/R 변에서 안쪽 2cm (기존 INSET_M 과 동일)


def _mode(v, step=0.005):
    """1D 값의 최빈 구간 중앙 (기존 h_top/h_table 과 같은 방식)."""
    if len(v) == 0:
        return None
    bins = np.arange(v.min(), v.max() + step, step)
    if len(bins) < 2:
        return float(np.median(v))
    hist, edges = np.histogram(v, bins=bins)
    k = int(np.argmax(hist))
    return float((edges[k] + edges[k + 1]) / 2)


def fit_top_plane_rect(pts3d, h, h_top, up, table_pts=None, rng=None):
    """윗면을 '고정 기울기 가정' 없이 depth 로 직접 맞춘다 (비교/검증용).

    1) h_top 근처 넓은 띠의 점들로 RANSAC 평면 → 법선 n (실제 기울기)
    2) inlier 를 평면 좌표(e1=카메라 x 의 평면 투영, e2=n×e1, 미터)로 펼쳐 5mm 격자화
       → open/close → 최대 덩어리 → 경계 1셀 깎기(depth 경계 튐 제거) → minAreaRect
    3) 사각형 4 꼭지점/변 중점을 3D 로 복원, L/R = 카메라 좌/우 변 중점에서 안쪽 2cm
    반환: dict 또는 None (카메라 좌표계, m)
    """
    rng = rng or np.random.default_rng(0)
    band = (h > h_top - ALT_BAND_DOWN) & (h < h_top + ALT_BAND_UP)
    P = pts3d[band]
    if len(P) < 200:
        return None

    # --- RANSAC 평면 (벡터화) ---
    S = P if len(P) <= 4000 else P[rng.choice(len(P), 4000, replace=False)]
    idx = rng.integers(0, len(S), size=(ALT_RANSAC_IT, 3))
    a, b, c = S[idx[:, 0]], S[idx[:, 1]], S[idx[:, 2]]
    N = np.cross(b - a, c - a)
    nn = np.linalg.norm(N, axis=1)
    ok = nn > 1e-9
    if not ok.any():
        return None
    N, a = N[ok] / nn[ok, None], a[ok]
    N[(N @ up) < 0] *= -1                                    # 위쪽을 향하게
    tilt_ok = (N @ up) > np.cos(np.radians(ALT_MAX_TILT))
    if not tilt_ok.any():
        return None
    N, a = N[tilt_ok], a[tilt_ok]
    d = np.abs((S[None, :, :] - a[:, None, :]) @ N[:, :, None])[..., 0]   # (it, n)
    best = int(np.argmax((d < ALT_RANSAC_TH).sum(axis=1)))
    n0, p0 = N[best], a[best]
    inl = np.abs((P - p0) @ n0) < ALT_RANSAC_TH
    if inl.sum() < 150:
        return None
    # 최소제곱 재피팅 (SVD)
    Q = P[inl]
    cen = Q.mean(axis=0)
    _, _, vt = np.linalg.svd(Q - cen, full_matrices=False)
    n = vt[2] if vt[2] @ up > 0 else -vt[2]
    res = (Q - cen) @ n
    rms_mm = float(np.sqrt(np.mean(res ** 2)) * 1000)
    tilt_deg = float(np.degrees(np.arccos(np.clip(n @ up, -1, 1))))

    # --- 평면 좌표계 ---
    cx = np.array([1.0, 0.0, 0.0])
    e1 = cx - (cx @ n) * n
    e1 /= np.linalg.norm(e1) + 1e-12
    e2 = np.cross(n, e1)
    uv = np.stack([(Q - cen) @ e1, (Q - cen) @ e2], axis=1)

    # --- 격자화 → 정리 → 사각형 ---
    # 여백 8셀: close/erode 가 격자 경계에 닿으면(OpenCV 경계값) 가장자리가 안 줄어 크기가 커진다
    PAD = 8
    mn = uv.min(axis=0) - PAD * ALT_CELL
    ij = np.floor((uv - mn) / ALT_CELL).astype(int)
    Wg, Hg = ij[:, 0].max() + PAD + 1, ij[:, 1].max() + PAD + 1
    if Wg * Hg > 400 * 400:
        return None
    grid = np.zeros((Hg, Wg), np.uint8)
    grid[ij[:, 1], ij[:, 0]] = 255
    k3 = np.ones((3, 3), np.uint8)
    grid = cv2.morphologyEx(grid, cv2.MORPH_CLOSE, k3, iterations=2)
    grid = cv2.morphologyEx(grid, cv2.MORPH_OPEN, k3, iterations=1)
    n_lbl, lbls, stats, _ = cv2.connectedComponentsWithStats(grid, 8)
    if n_lbl < 2:
        return None
    big = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    grid = np.where(lbls == big, 255, 0).astype(np.uint8)
    grid = cv2.erode(grid, k3, iterations=1)                 # 경계 1셀(5mm) 깎기
    ys, xs = np.nonzero(grid)
    if len(xs) < 20:
        return None
    (rc_x, rc_y), (rw, rh), ang = cv2.minAreaRect(np.stack([xs, ys], 1).astype(np.float32))
    # 셀 중심 → 실제 외곽 (+1셀), 깎은 만큼 복원 (+2셀)
    rw_m, rh_m = (rw + 3) * ALT_CELL, (rh + 3) * ALT_CELL
    box = cv2.boxPoints(((rc_x, rc_y), (rw + 3, rh + 3), ang))      # 셀 단위 4점

    def to3d(pc):                                            # 셀 좌표 → 3D (평면 위)
        u = mn[0] + (pc[0] + 0.5) * ALT_CELL
        v = mn[1] + (pc[1] + 0.5) * ALT_CELL
        return cen + u * e1 + v * e2

    corners = [to3d(p) for p in box]
    center = to3d((rc_x, rc_y))
    mids = [(corners[i] + corners[(i + 1) % 4]) / 2 for i in range(4)]
    mu = [(m - cen) @ e1 for m in mids]                      # 카메라 좌(-)/우(+)
    L_edge, R_edge = mids[int(np.argmin(mu))], mids[int(np.argmax(mu))]

    def inset(e):
        dv = center - e
        dl = np.linalg.norm(dv)
        return e + dv / dl * ALT_INSET_M if dl > ALT_INSET_M else e

    # 박스 높이: 평면 법선 방향으로 테이블까지 (테이블 점 있으면)
    H_m = None
    if table_pts is not None and len(table_pts) > 50:
        ht = _mode(table_pts @ n)
        if ht is not None:
            H_m = float(cen @ n - ht)
            if not (0.01 < H_m < 0.50):
                H_m = None

    return {"top_center_3d": center, "L": inset(L_edge), "R": inset(R_edge),
            "corners_3d": corners, "W_m": float(max(rw_m, rh_m)), "D_m": float(min(rw_m, rh_m)),
            "H_m": H_m, "normal": n, "tilt_deg": tilt_deg, "rms_mm": rms_mm,
            "inliers": int(inl.sum()), "band": int(len(P))}


class BoxEstimator:
    def __init__(self, model_path, K, conf=0.4, device="cpu", imgsz=640):
        from ultralytics import YOLO
        self.model  = YOLO(model_path, task="segment")
        self.K      = np.asarray(K, dtype=np.float32)
        self.conf   = conf
        self.device = device
        self.imgsz  = imgsz

    def detect(self, color_bgr, depth_mm, gravity_cam=None):
        results = self.model(color_bgr, conf=self.conf,
                             device=self.device, imgsz=self.imgsz,
                             verbose=False)
        if len(results) == 0 or results[0].masks is None:
            return None

        confs = results[0].boxes.conf.cpu().numpy()
        if len(confs) == 0:
            return None
        idx = int(np.argmax(confs))
        conf_val = float(confs[idx])

        mask_raw = results[0].masks.data[idx].cpu().numpy()
        H_img, W_img = color_bgr.shape[:2]
        mask = cv2.resize(mask_raw, (W_img, H_img),
                          interpolation=cv2.INTER_LINEAR) > 0.5

        if mask.sum() < MIN_PIXELS:
            return None

        out = {
            'mask':         mask,
            'conf':         conf_val,
            'mask_pixels':  int(mask.sum()),
        }

        fx, fy = self.K[0, 0], self.K[1, 1]
        cx_K, cy_K = self.K[0, 2], self.K[1, 2]

        ys, xs = np.where(mask)
        z = depth_mm[ys, xs].astype(np.float32) / 1000.0
        valid = (z > MIN_DEPTH_M) & (z < MAX_DEPTH_M)
        if valid.sum() < 10:
            return out
        xs, ys, z = xs[valid], ys[valid], z[valid]

        X = (xs - cx_K) * z / fx
        Y = (ys - cy_K) * z / fy
        pts3d = np.stack([X, Y, z], axis=1)

        # mask 무게중심
        u_cen = float(xs.mean())
        v_cen = float(ys.mean())
        z_med = float(np.median(z))
        Xc = (u_cen - cx_K) * z_med / fx
        Yc = (v_cen - cy_K) * z_med / fy
        out['center_3d']  = np.array([Xc, Yc, z_med], dtype=np.float64)
        out['distance_m'] = float(np.linalg.norm([Xc, Yc, z_med]))

        if gravity_cam is None:
            return out

        # up 벡터
        gravity = gravity_cam.astype(np.float64)
        gravity /= np.linalg.norm(gravity) + 1e-9
        up = -gravity
        if up[1] > 0:
            up = -up

        # 수직 높이
        h = pts3d @ up

        # 윗면 mode
        h_max = h.max()
        top_band = h[h > h_max - 0.05]
        if len(top_band) < 30:
            return out

        bins = np.arange(top_band.min(), top_band.max() + 0.005, 0.005)
        if len(bins) >= 2:
            hist, edges = np.histogram(top_band, bins=bins)
            peak = int(np.argmax(hist))
            h_top = (edges[peak] + edges[peak + 1]) / 2
        else:
            h_top = float(np.median(top_band))

        near = h[np.abs(h - h_top) < 0.005]
        if len(near) >= 20:
            h_top = float(np.median(near))

        # === 테이블 높이 (mask 외곽 영역에서 mode) ===
        # 박스 mask 외곽 25~50px 띠 = 테이블 픽셀
        mask_u8_orig = mask.astype(np.uint8)
        outer = cv2.dilate(mask_u8_orig, np.ones((101, 101), np.uint8)) > 0
        inner = cv2.dilate(mask_u8_orig, np.ones((51, 51), np.uint8)) > 0
        table_region = outer & ~inner

        h_table = None
        pts_table = None
        if table_region.sum() > 50:
            ys_t, xs_t = np.where(table_region)
            z_t = depth_mm[ys_t, xs_t].astype(np.float32) / 1000.0
            v_t = (z_t > MIN_DEPTH_M) & (z_t < MAX_DEPTH_M)
            if v_t.sum() > 50:
                xs_t, ys_t, z_t = xs_t[v_t], ys_t[v_t], z_t[v_t]
                X_t = (xs_t - cx_K) * z_t / fx
                Y_t = (ys_t - cy_K) * z_t / fy
                pts_t = np.stack([X_t, Y_t, z_t], axis=1)
                pts_table = pts_t
                h_t = pts_t @ up
                # mode
                bins_t = np.arange(h_t.min(), h_t.max() + 0.005, 0.005)
                if len(bins_t) >= 2:
                    hist_t, edges_t = np.histogram(h_t, bins=bins_t)
                    peak_t = int(np.argmax(hist_t))
                    h_table = (edges_t[peak_t] + edges_t[peak_t + 1]) / 2
                else:
                    h_table = float(np.median(h_t))

        # 박스 H = h_top - h_table
        H_m = None
        if h_table is not None:
            H_m = float(h_top - h_table)
            if H_m < 0.01 or H_m > 0.50:
                H_m = None

        # === 비교용 alt: 평면 피팅 + 3D 사각형 (기존 결과에는 영향 없음) ===
        try:
            alt = fit_top_plane_rect(pts3d, h, h_top, up, pts_table)
        except Exception as e:
            alt = None
            if VERBOSE_LOG:
                print(f"[BOX-ALT] 실패: {e}")
        if alt is not None:
            out['alt'] = alt

        # 윗면 분리: 비대칭 임계
        #   윗면이 카메라 향해 기울면 h가 h_top보다 위로 퍼짐 → 위쪽 여유 넉넉히
        #   앞면은 h_top보다 아래로 연속해 내려감 → 아래쪽은 빡빡하게 차단
        #   (테이블까지 거리 H를 알면 그 절반을 경계로)
        top_up   = TOP_THRESH_M + 0.015          # 위쪽 +3.5cm
        if H_m is not None and H_m > 0.02:
            top_down = min(TOP_THRESH_M, H_m * 0.35)  # 박스높이 35%까지만 (앞면 차단)
        else:
            top_down = TOP_THRESH_M               # H 모르면 ±2cm
        is_top = (h < h_top + top_up) & (h > h_top - top_down)
        if is_top.sum() < 30:
            return out

        # 윗면 mask (2D)
        H_img2, W_img2 = mask.shape
        top_pixel_mask = np.zeros((H_img2, W_img2), dtype=np.uint8)
        top_pixel_mask[ys[is_top], xs[is_top]] = 255

        # 노이즈 제거 (커널 키우고 close 강하게)
        kernel = np.ones((5, 5), np.uint8)
        top_clean = cv2.morphologyEx(top_pixel_mask, cv2.MORPH_OPEN,
                                      kernel, iterations=1)
        top_clean = cv2.morphologyEx(top_clean, cv2.MORPH_CLOSE,
                                      kernel, iterations=3)

        # 가장 큰 연결 덩어리만 유지 (떨어진 얼룩 제거)
        n_lbl, lbls, stats, _ = cv2.connectedComponentsWithStats(top_clean, 8)
        if n_lbl > 1:
            largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
            top_clean = np.where(lbls == largest, 255, 0).astype(np.uint8)
            # 내부 구멍 메우기
            top_clean = cv2.morphologyEx(top_clean, cv2.MORPH_CLOSE,
                                          np.ones((9, 9), np.uint8), iterations=2)

        # 윗면 외곽선 → minAreaRect → 4 꼭지점 (2D)
        contours, _ = cv2.findContours(top_clean, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        out['top_pixel_mask'] = top_clean > 0
        out['top_pixels']     = int((top_clean > 0).sum())
        out['h_top_m']        = float(h_top)

        if len(contours) == 0:
            return out
        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < 100:
            return out

        # === 4 꼭지점 추출 ===
        # minAreaRect: 윗면 마스크를 감싸는 최소 회전 사각형 (노이즈에 강함)
        # top-down 뷰 + 마스크 얼룩에도 안정적으로 4모서리 추출
        contour_pts = contour.reshape(-1, 2).astype(np.float32)

        rect = cv2.minAreaRect(contour)       # ((cx,cy),(w,h),angle)
        box_pts = cv2.boxPoints(rect)         # 4점 (회전 사각형)
        corners_4 = box_pts.astype(np.float32)

        # 보조: approxPolyDP로 4점이 깔끔히 나오면 그게 더 정확 (사다리꼴 대응)
        perimeter = cv2.arcLength(contour, True)
        for eps_ratio in [0.02, 0.03, 0.04, 0.05]:
            approx = cv2.approxPolyDP(contour, eps_ratio * perimeter, True)
            if len(approx) == 4:
                poly = approx.reshape(4, 2).astype(np.float32)
                # approx 사각형이 minAreaRect 면적의 70% 이상이면 채택
                # (너무 작으면 노이즈로 모서리 놓친 것)
                if cv2.contourArea(poly) > 0.70 * cv2.contourArea(box_pts):
                    corners_4 = poly
                break

        # === 4 꼭지점 정렬: TL / TR / BR / BL ===
        # 4개를 무게중심 기준 시계방향 + TL부터 시작
        cen = corners_4.mean(axis=0)
        # 각 점의 각도 (무게중심 기준)
        angles = np.arctan2(corners_4[:, 1] - cen[1],
                             corners_4[:, 0] - cen[0])
        # 각도 오름차순 정렬 (-π부터 +π)
        order = np.argsort(angles)
        sorted_pts = corners_4[order]
        # 시계방향 정렬됨: 좌상 → 우상 → 우하 → 좌하
        # 가장 좌상 (x+y 최소)을 첫 점으로 회전
        sums = sorted_pts[:, 0] + sorted_pts[:, 1]
        start_idx = int(np.argmin(sums))
        rolled = np.roll(sorted_pts, -start_idx, axis=0)

        # 시계방향: TL, TR, BR, BL
        # 다만 sorted_pts가 시계방향인지 반시계방향인지 확인 필요
        # 시계방향이면 두번째 점은 첫 점에서 +x 방향 (대략)
        # 두번째 점의 x > 첫 점의 x 이면 시계방향
        if rolled[1, 0] > rolled[0, 0]:
            # 시계방향 — TL/TR/BR/BL 순서 맞음
            TL, TR, BR, BL = rolled
        else:
            # 반시계 — 뒤집기
            TL = rolled[0]
            BL = rolled[1]
            BR = rolled[2]
            TR = rolled[3]

        # 윗면 중심 (2D): 4 꼭지점 평균
        center_2d = (TL + TR + BL + BR) / 4

        # 좌/우 변 중심 (2D)
        L_edge = (TL + BL) / 2   # 좌측 변 정중앙
        R_edge = (TR + BR) / 2   # 우측 변 정중앙

        # === 6개 2D 점을 3D로 복원 ===
        # 윗면 평면: up · X = h_top
        normal = up
        plane_d = -h_top

        def pixel_to_top_plane(u, v):
            rx = (u - cx_K) / fx
            ry = (v - cy_K) / fy
            ray = np.array([rx, ry, 1.0])
            denom = np.dot(normal, ray)
            if abs(denom) < 1e-6:
                return None
            t = -plane_d / denom
            return np.array([t*rx, t*ry, t])

        TL_3d = pixel_to_top_plane(*TL)
        TR_3d = pixel_to_top_plane(*TR)
        BL_3d = pixel_to_top_plane(*BL)
        BR_3d = pixel_to_top_plane(*BR)
        L_edge_3d = pixel_to_top_plane(*L_edge)
        R_edge_3d = pixel_to_top_plane(*R_edge)
        center_3d_top = pixel_to_top_plane(*center_2d)

        # === L/R 안쪽 2cm 고정 이동 (3D 거리) ===
        INSET_M = 0.02   # 변에서 박스 중심 방향으로 2cm

        L_mid_3d, R_mid_3d = None, None
        L_mid, R_mid = L_edge, R_edge   # fallback (2D)
        if L_edge_3d is not None and center_3d_top is not None:
            dir_L = center_3d_top - L_edge_3d
            dist_L = np.linalg.norm(dir_L)
            if dist_L > INSET_M:
                L_mid_3d = L_edge_3d + (dir_L / dist_L) * INSET_M
            else:
                # 박스가 너무 작으면 변 중심 그대로
                L_mid_3d = L_edge_3d
        if R_edge_3d is not None and center_3d_top is not None:
            dir_R = center_3d_top - R_edge_3d
            dist_R = np.linalg.norm(dir_R)
            if dist_R > INSET_M:
                R_mid_3d = R_edge_3d + (dir_R / dist_R) * INSET_M
            else:
                R_mid_3d = R_edge_3d

        # 시각화용 2D: 3D L/R을 다시 픽셀로 사영
        def project_to_pixel(p3d):
            if p3d is None or p3d[2] < 1e-6:
                return None
            u = p3d[0] * fx / p3d[2] + cx_K
            v = p3d[1] * fy / p3d[2] + cy_K
            return np.array([u, v])

        L_proj = project_to_pixel(L_mid_3d)
        R_proj = project_to_pixel(R_mid_3d)
        if L_proj is not None:
            L_mid = L_proj
        if R_proj is not None:
            R_mid = R_proj

        # 윗면 중심 = 4 꼭지점 3D 평균
        if all(p is not None for p in [TL_3d, TR_3d, BL_3d, BR_3d]):
            top_center_3d = (TL_3d + TR_3d + BL_3d + BR_3d) / 4
        else:
            top_center_3d = pts3d[is_top].mean(axis=0)

        # === 박스 크기 추정 ===
        W_m, D_m = None, None
        if all(p is not None for p in [TL_3d, TR_3d, BL_3d, BR_3d]):
            # 위/아래 변 = TL-TR, BL-BR
            top_edge    = float(np.linalg.norm(TR_3d - TL_3d))
            bottom_edge = float(np.linalg.norm(BR_3d - BL_3d))
            # 좌/우 변 = TL-BL, TR-BR
            left_edge   = float(np.linalg.norm(BL_3d - TL_3d))
            right_edge  = float(np.linalg.norm(BR_3d - TR_3d))
            # 위/아래 평균 = 한 차원, 좌/우 평균 = 다른 차원
            W_m = (top_edge + bottom_edge) / 2
            D_m = (left_edge + right_edge) / 2
            # W가 큰 쪽 (가로)
            if D_m > W_m:
                W_m, D_m = D_m, W_m

        out['top_center_3d']  = top_center_3d
        out['top_corners_3d'] = {
            'TL': TL_3d, 'TR': TR_3d,
            'BL': BL_3d, 'BR': BR_3d,
        }
        out['top_corners_2d'] = {
            'TL': TL, 'TR': TR,
            'BL': BL, 'BR': BR,
        }
        out['top_mids_3d']    = {'L': L_mid_3d, 'R': R_mid_3d}
        out['top_mids_2d']    = {'L': L_mid, 'R': R_mid}
        if W_m is not None:
            out['box_W_m'] = W_m
            out['box_D_m'] = D_m
        if H_m is not None:
            out['box_H_m'] = H_m
            out['h_table_m'] = float(h_table)

        if VERBOSE_LOG:
            size_str = ""
            if W_m is not None and H_m is not None:
                size_str = f" size={W_m*100:.1f}x{D_m*100:.1f}x{H_m*100:.1f}cm"
            elif W_m is not None:
                size_str = f" WxD={W_m*100:.1f}x{D_m*100:.1f}cm"
            h_tab_str = f"{h_table*100:+.1f}" if h_table is not None else "?"
            H_str = f"{H_m*100:.1f}" if H_m is not None else "?"
            alt_str = ""
            if out.get('alt'):
                a = out['alt']
                alt_str = (f" | ALT tilt={a['tilt_deg']:.1f}° rms={a['rms_mm']:.1f}mm "
                           f"{a['W_m']*100:.1f}x{a['D_m']*100:.1f}cm")
            print(f"[BOX] up={up.round(2)} h_top={h_top*100:+.1f} "
                  f"h_table={h_tab_str} H={H_str}cm "
                  f"top_px={int(is_top.sum())}/{len(pts3d)} "
                  f"({100*is_top.sum()/len(pts3d):.0f}%){size_str}{alt_str}")

        return out


# ==============================================================
# 시각화
# ==============================================================
def _project_point_3d(pt3d, K, dist_coeffs):
    rvec0 = np.zeros(3, dtype=np.float32)
    tvec0 = np.zeros(3, dtype=np.float32)
    p3d = np.asarray(pt3d, dtype=np.float32).reshape(1, 3)
    p2d, _ = cv2.projectPoints(p3d, rvec0, tvec0, K, dist_coeffs)
    return p2d[0, 0].astype(int)


def draw_box_overlay(frame, result, K=None, dist_coeffs=None,
                    draw_mask=True, draw_obb=True, draw_axes=True):
    """seg mask + 윗면 + 4 꼭지점 + 좌/우 중심."""
    if result is None or 'mask' not in result:
        return
    if dist_coeffs is None:
        dist_coeffs = np.zeros(5, dtype=np.float32)

    mask = result['mask']

    # 1) 전체 mask = 옅은 빨강
    mask_color = np.zeros_like(frame)
    mask_color[mask] = (100, 100, 255)
    cv2.addWeighted(mask_color, 0.25, frame, 1.0, 0, frame)

    # 2) 윗면 = 파랑
    if 'top_pixel_mask' in result:
        top = result['top_pixel_mask']
        top_color = np.zeros_like(frame)
        top_color[top] = (255, 150, 0)
        cv2.addWeighted(top_color, 0.5, frame, 1.0, 0, frame)

    # 3) mask 외곽선 (빨강)
    mask_u8 = (mask.astype(np.uint8)) * 255
    contours, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(frame, contours, -1, (0, 100, 255), 2)

    if K is None:
        return

    # 4) 윗면 4 꼭지점 + 변 (초록)
    if 'top_corners_2d' in result:
        corners = result['top_corners_2d']
        TL, TR = corners['TL'].astype(int), corners['TR'].astype(int)
        BL, BR = corners['BL'].astype(int), corners['BR'].astype(int)

        # 변 그리기
        pts = np.array([TL, TR, BR, BL])
        cv2.polylines(frame, [pts], True, (0, 255, 0), 1)

        # 꼭지점 (작은 초록 점 + 라벨)
        for label, pt in [('TL', TL), ('TR', TR), ('BL', BL), ('BR', BR)]:
            cv2.circle(frame, tuple(pt), 3, (0, 255, 0), -1)
            cv2.circle(frame, tuple(pt), 4, (255, 255, 255), 1)
            cv2.putText(frame, label, (pt[0] + 6, pt[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)

    # 5) 좌/우 변 중심 (마젠타)
    if 'top_mids_2d' in result:
        mids = result['top_mids_2d']
        L_pt = mids['L'].astype(int)
        R_pt = mids['R'].astype(int)
        cv2.line(frame, tuple(L_pt), tuple(R_pt), (255, 0, 255), 1)
        for label, pt in [('L', L_pt), ('R', R_pt)]:
            cv2.circle(frame, tuple(pt), 5, (255, 0, 255), -1)
            cv2.circle(frame, tuple(pt), 6, (255, 255, 255), 1)
            offset_x = -15 if label == 'L' else 9
            cv2.putText(frame, label, (pt[0] + offset_x, pt[1] + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)

    # 6) 윗면 중심 (T)
    if 'top_center_3d' in result:
        px, py = _project_point_3d(result['top_center_3d'], K, dist_coeffs)
        cv2.circle(frame, (px, py), 5, (255, 0, 255), -1)
        cv2.circle(frame, (px, py), 6, (255, 255, 255), 1)
        cv2.putText(frame, "T", (px + 9, py - 9),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)


def draw_gravity_overlay(frame, gravity_cam, K, origin_3d=None):
    pass
