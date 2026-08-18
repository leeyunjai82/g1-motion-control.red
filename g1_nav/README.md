# g1_nav — Unitree G1 SLAM 내비게이션 웹 콘솔

공식 SDK2 SLAM 예제(`keyDemo.cpp`)를 Python 으로 옮기고 웹 UI 를 붙인 것입니다.
C++ 빌드는 필요 없습니다.

## 구성

| 파일 | 역할 |
|---|---|
| `slam_client.py` | `slam_operate` 서비스 RPC + 토픽 구독 (핵심 로직) |
| `nav_server.py`  | FastAPI 서버 (포트 50030) |
| `nav_web.html`   | 웹 콘솔 — 지도, 지점, 정지오차 측정 |
| `nav_probe.py`   | 웹 없이 터미널로 확인하는 도구 |
| `start_nav.sh`   | 실행 스크립트 |

## 준비

```bash
pip install fastapi uvicorn
# unitree_sdk2py 는 이미 설치되어 있어야 합니다
```

**ROS/ROS2 환경을 source 하지 않은 쉘**에서 실행하세요. SLAM 서비스와 충돌합니다.
`start_nav.sh` 가 관련 환경변수를 걷어내지만, 새 터미널을 쓰는 쪽이 확실합니다.

## 로봇 준비

```
Damp → StandUp → SetFsmId(501)
```

501 은 허리 3자유도(29축) G1 의 정규 보행 모드입니다.

**1차 시험에서는 `robot_server`(50000) 를 띄우지 마세요.**
`ArmControllerWrapper` 가 250Hz 로 arm_sdk 를 쏘면서 상체를 붙잡기 때문에,
보행이 망가지고 정지오차 측정값도 오염됩니다.

## 실행

```bash
./start_nav.sh eth0          # eth0 = 123 대역 네트워크 카드
```

브라우저에서 `http://<PC IP>:50030`

## 사용 순서

1. **토픽 확인** — `python3 nav_probe.py --iface eth0 listen`
   위치가 안 들어오면 웹으로 가도 소용없습니다. 여기서 먼저 잡으세요.
2. **매핑** — 웹에서 `매핑 시작` → 로봇 수동 주행 → `매핑 저장`
3. **측위 초기화** — 로봇을 기준 자리에 세우고 그 좌표를 넣고 `측위 시작`
4. **지점 등록** — 로봇을 원하는 자리에 세우고 이름 넣고 `현재 위치로 저장`
   또는 지도를 드래그해서 좌표와 방향을 직접 찍고 `지점 저장`
5. **이동** — 지점 목록의 `이동` 버튼
6. **정지오차 측정** — 지점과 왕복 횟수를 넣고 `측정 시작`

## 정지오차 측정이 핵심입니다

홈 ↔ 목표를 왕복하면서 매번 오차를 기록하고, 평균·표준편차·최악값을 냅니다.
최악 5cm / 10° 안쪽이면 마커 없이 바로 파지로 넘어갈 수 있고,
넘으면 ArUco 정밀 보정 단계가 필요합니다.

기록은 `arrivals.jsonl` 에 계속 쌓입니다. 지점은 `waypoints.json` 에 저장됩니다.

## 환경변수

| 변수 | 기본값 | 설명 |
|---|---|---|
| `NAV_PCD` | `/home/unitree/test.pcd` | 맵 파일 경로 |
| `NAV_TIMEOUT` | `180` | 이동 한 건 최대 대기(초) |
| `NAV_SETTLE_SEC` | `1.5` | 도착 후 위치를 읽기까지 대기(초) |
| `NAV_WAYPOINTS` | `waypoints.json` | 지점 저장 파일 |

## API

| 메서드 | 경로 | 설명 |
|---|---|---|
| GET | `/status` | 현재 상태, 위치, 도착 기록 |
| GET/POST/DELETE | `/waypoints[/{name}]` | 지점 조회·저장·삭제 |
| POST | `/waypoints/{name}/capture` | 현재 위치를 지점으로 저장 |
| POST | `/goto/{name}` | 저장된 지점으로 이동 |
| POST | `/goto_pose` | 좌표 지정 이동 |
| POST | `/repeat` | 왕복 정지오차 측정 |
| POST | `/relocation` | 맵 로드 + 위치 초기화 |
| POST | `/mapping/start`, `/mapping/end` | 매핑 |
| POST | `/pause`, `/resume`, `/cancel` | 주행 제어 |
| POST | `/stop_node` | SLAM 노드 종료 |

## 주의

- 비상정지는 이 서버가 아니라 `robot_server`(50000) 의 정지 버튼입니다.
- 나중에 `robot_server` 를 함께 띄울 때는 이동 전 `POST /arm_release`,
  도착 후 `POST /arm_hold` 로 팔 제어권을 수동 전환하세요.
- 파지 후 다시 이동하기 전에 허리 yaw 를 0 으로 되돌리세요.
  라이다가 상체에 있으면 허리 각도만큼 측위 방향이 어긋납니다.

## 알려진 불확실성

`slam_client.py` 의 `String_` 임포트 경로가 `unitree_sdk2py` 버전에 따라 다를 수 있습니다.

```python
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
```

여기서 깨지면 C++ 예제를 빌드해서 쓰는 방법도 있습니다.
