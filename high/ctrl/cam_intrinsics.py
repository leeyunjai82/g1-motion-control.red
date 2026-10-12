#!/usr/bin/env python3
# Version: 0.1
"""
cam_intrinsics.py — 카메라 내부 파라미터(K)를 rs_stream(50001) GET /intrinsics 에서 읽는다.

  · rs_stream 이 카메라에서 직접 읽은 color 스트림 intrinsics (depth 는 color 에 align 되므로 같은 K)
  · 해상도를 바꿔도 K 가 자동으로 맞는다 — 고정값(640x480 기준 606/316/258)은 더 이상 쓰지 않는다
  · 읽기 실패 시 고정값으로 대체하지 않고 예외 → 서버가 뜨지 않는다
    (틀린 K 로 3D 좌표를 내면 팔이 엉뚱한 곳으로 가므로)

  dist(왜곡) 는 지금까지와 같이 0 으로 쓴다. RealSense color 의 왜곡 모델(Inverse Brown-Conrady)은
  OpenCV 모델과 정의가 달라 계수를 그대로 넣으면 안 된다. 원본 계수는 응답의 coeffs 로만 보관.
"""
import os
import json
import time
import urllib.request

import numpy as np

INTR_URL = os.environ.get("RS_INTR_URL", "http://localhost:50001/intrinsics")


def fetch_intrinsics(url=INTR_URL, wait_sec=60.0, tag="CAM"):
    """rs_stream /intrinsics 를 읽어 (info, K) 반환. wait_sec 동안 1초 간격 재시도 후 실패면 RuntimeError."""
    t0 = time.time()
    last = None
    while True:
        try:
            d = json.loads(urllib.request.urlopen(url, timeout=1.0).read())
            for k in ("width", "height", "fx", "fy", "ppx", "ppy"):
                if d.get(k) is None:
                    raise ValueError(f"'{k}' 없음")
            K = np.array([[d["fx"], 0, d["ppx"]],
                          [0, d["fy"], d["ppy"]],
                          [0, 0, 1]], dtype=np.float32)
            print(f"[{tag}] K 로드 ({d['width']}x{d['height']}) "
                  f"fx={d['fx']:.3f} fy={d['fy']:.3f} ppx={d['ppx']:.3f} ppy={d['ppy']:.3f}")
            return d, K
        except Exception as e:
            last = e
        if time.time() - t0 > wait_sec:
            raise RuntimeError(f"카메라 K 를 읽지 못함 ({url}): {last} — rs_stream(50001) 실행 확인")
        time.sleep(1.0)


class FrameSizeGuard:
    """프레임 크기가 K 를 읽은 해상도와 다르면 그 프레임은 쓰지 않는다 (로그는 한 번만)."""

    def __init__(self, info, tag="CAM"):
        self.wh = (int(info["width"]), int(info["height"]))
        self.tag = tag
        self._warned = False

    def ok(self, img):
        h, w = img.shape[:2]
        if (w, h) == self.wh:
            self._warned = False
            return True
        if not self._warned:
            print(f"[{self.tag}] ⚠️ 프레임 {w}x{h} ≠ K 해상도 {self.wh[0]}x{self.wh[1]} — 프레임 무시 "
                  f"(rs_stream 해상도가 바뀌었으면 이 서버를 다시 시작)")
            self._warned = True
        return False
