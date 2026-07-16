import serial
import time
import math

# --- [Helper Functions] ---
# Note: Data=0은 프로토콜상 "skip"(이전 위치 유지) 의미이므로, 모션 명령에서는
#       최소값 1을 보장해서 full-release(-19 step) 동작이 가능하게 함.
def _to_data(base_value, ratio):
    """ratio(0.0~1.0)을 Data 바이트값으로 변환. ratio>0 이면 최소 1 보장."""
    val = math.floor(base_value * ratio)
    if ratio > 0 and val == 0:
        val = 1  # 0은 skip이므로 최소 1로 클램프 (= -19 step, full release)
    return val & 0xFF

# 엄지 손가락 위치 조정 (Thumb Flexion)
def f1_pos(ratio):
    return _to_data(0x50, ratio)

# 검지/중지/약지/새끼 손가락 위치 조정 (Index/Middle/Ring/Little Flexion)
def f24_pos(ratio):
    return _to_data(0x90, ratio)
    #return _to_data(0x80, ratio)

# 엄지 외전 위치 조정 (Thumb Abduction) - 6번째 자유도
# base 0x40 = target 107 step (실제 가동 범위 ~119 근처)
def f6_pos(ratio):
    return _to_data(0x90, ratio)


def create_command(fingers, ratio):
    """
    fingers: 6-element list [Thumb, Index, Middle, Ring, Little, ThumbAbd]
             각 원소는 0/1 (선택 비트)
    """
    position = [
        f1_pos(ratio)  if fingers[0] else 0,  # B4: Thumb Flexion
        f24_pos(ratio) if fingers[1] else 0,  # B5: Index Flexion
        f24_pos(ratio) if fingers[2] else 0,  # B6: Middle Flexion
        f24_pos(ratio) if fingers[3] else 0,  # B7: Ring Flexion
        f24_pos(ratio) if fingers[4] else 0,  # B8: Little Flexion
        f6_pos(ratio)  if fingers[5] else 0,  # B9: Thumb Abduction
    ]

    # fingers[0]=Thumb(bit 1) ... fingers[5]=ThumbAbd(bit 32)
    finger_sel = int("".join(map(str, fingers[::-1])), 2)

    command = [0xFF]                  # B0 : left(0xFD), right(0xFE), both(0xFF)
    command.append(finger_sel)        # B1 : 손가락 선택 비트마스크 (6 bits)
    command.extend([0x80, 0xA0])      # B2 speed=144 (RPM/200, spec 30~150), B3 current=1200mA
    command.extend(position)          # B4-B9 : 6개 손가락 위치값
    command.append(0x1)               # B10: 방향 (0:Idle, 1:Forward, 2:Reverse, 3:Reset)

    # 동작 시간 계산 (대략적인 값)
    return {'pos': command, 'time': sum(e == 1 for e in fingers) * 0.5}

# --- [Motion Dictionary] ---
# fingers: [Thumb, Index, Middle, Ring, Little, ThumbAbd]
# 기본 정책: Thumb이 움직이면 ThumbAbd도 함께 움직이도록 묶음
motions = {
    # --- 기본 동작 ---
    "unfold_a": [
        create_command([0, 1, 1, 1, 1, 0], 0.01),  # 4지 펴기
        create_command([1, 0, 0, 0, 0, 1], 0.01),  # 엄지 + 외전 펴기
    ],
    "fold_a": [
        create_command([1, 0, 0, 0, 0, 1], 1.0),   # 엄지 + 외전 접기
        create_command([0, 1, 1, 1, 1, 0], 1.0),   # 4지 접기
    ],
    "fold_ha": [
        create_command([1, 0, 0, 0, 0, 1], 0.5),
        create_command([0, 1, 1, 1, 1, 0], 0.5),
    ],
    # --- 조합 동작 ---
    "point": [
        create_command([0, 0, 1, 1, 1, 0], 1.0),   # 중/약/소 접기 (검지만 펴짐)
        create_command([1, 0, 0, 0, 0, 1], 1.0),   # 엄지 + 외전 접기
    ],
    "handshake": [
        create_command([0, 1, 1, 1, 1, 0], 0.4),
    ],
    "ok": [
        create_command([1, 0, 0, 0, 0, 1], 0.5),   # 엄지 살짝 + 외전
        create_command([0, 1, 0, 0, 0, 0], 0.9),   # 검지 접어 동그라미
    ],
    "thumbup": [
        create_command([0, 0, 0, 0, 0, 1], 1.0),   # 엄지만 펴고 나머지 접기
        create_command([0, 1, 1, 1, 1, 0], 1.0),   # 엄지만 펴고 나머지 접기
    ],
    "victory": [
        create_command([1, 0, 0, 0, 0, 1], 1.0),   # 엄지 + 외전 접기
        create_command([0, 0, 0, 1, 1, 0], 1.0),   # 약/소 접기 (V자)
    ],
    "rock": [
        create_command([0, 0, 1, 1, 0, 0], 1.0),   # 중/약 접기 (검지+소지 펴짐)
    ],
    "promise": [
        create_command([1, 0, 0, 0, 0, 1], 1.0),   # 엄지 + 외전 접기
        create_command([0, 1, 1, 1, 0, 0], 1.0),   # 검/중/약 접기 (소지 펴짐)
    ],
    "grab": [
        create_command([1, 0, 0, 0, 0, 1], 0.7),
        create_command([0, 1, 1, 1, 1, 0], 0.5),
    ],
}

# 완전 릴리즈 모션 목록 - 실행 후 position counter 리셋 (sensor slip 보정)
RELEASE_MOTIONS = {'unfold_a'}

# --- [Controller Class] ---
class HandController:
    def __init__(self, port,
                 tx_repeat=3,            # ACK 미수신 시 최대 재시도 횟수
                 tx_repeat_interval=0.05,   # 재시도 사이 간격(초)
                 inter_packet_delay=0.02,   # 서로 다른 명령 사이 최소 간격(초)
                 ack_timeout=0.3):          # ACK(응답) 대기 시간(초)
        # 타임아웃 설정으로 무한 대기 방지
        self.ser = serial.Serial(
            port=port,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.5,       # 읽기 타임아웃
            write_timeout=0.5  # 쓰기 타임아웃
        )
        self.tx_repeat = tx_repeat
        self.tx_repeat_interval = tx_repeat_interval
        self.inter_packet_delay = inter_packet_delay
        self.ack_timeout = ack_timeout
        # 최근 피드백 캐시 (B) 옵션 확장용 — 외부에서 조회 가능
        self.last_feedback = {'left': None, 'right': None}
        time.sleep(1)  # 포트 안정화 대기

    def reset(self, selector='both'):
        # 0x3f = 0b111111 → 6개 손가락 모두 선택, Direction 3 = Reset
        self.send_motor([0xff, 0x3f, 0, 0, 0, 0, 0, 0, 0, 0, 0x3], selector)

    def preset(self, selector='both'):
        # 0x3f = 0b111111 → 6개 손가락 모두 선택, Direction 4 = Preset (확장 명령)
        self.send_motor([0xff, 0x3f, 0, 0, 0, 0, 0, 0, 0, 0, 0x4], selector)

    def send_motor(self, data, selector='both', repeat=None):
        """
        ACK 기반 재시도:
        - 1회 송신 후 ack_timeout 동안 응답 대기
        - 응답 받으면 즉시 종료 (평소엔 1패킷)
        - 응답 없으면 최대 tx_repeat 회까지 재송신
        - position이 절대값이라 중복 송신해도 안전 (idempotent)
        """
        cmd_data = data.copy()

        # selector 정규화
        sel = (selector or 'both').strip().lower()
        if sel == 'left':
            cmd_data[0] = 0xFD
        elif sel == 'right':
            cmd_data[0] = 0xFE
        else:
            cmd_data[0] = 0xFF
            sel = 'both'

        data_bytes = bytes(cmd_data) + b'\n'
        n = self.tx_repeat if repeat is None else max(1, repeat)
        hex_str = ' '.join(f'{b:02X}' for b in cmd_data)

        for attempt in range(n):
            try:
                self.ser.reset_input_buffer()

                # TX 로그
                tag = f"[TX→{sel}]" if attempt == 0 else f"[TX→{sel} retry {attempt}]"
                print(f"{tag} {hex_str}")

                # 송신
                self.ser.write(data_bytes)
                self.ser.flush()

                # ACK 대기 (최대 ack_timeout)
                deadline = time.time() + self.ack_timeout
                response = b''
                while time.time() < deadline:
                    if self.ser.in_waiting > 0:
                        response += self.ser.read(self.ser.in_waiting)
                        if b'\n' in response:
                            break
                    time.sleep(0.005)

                if response:
                    text = response.decode(errors='ignore').strip()
                    if text:
                        print(f"[RX←{sel}] {text}")
                        self._parse_feedback(text)
                        time.sleep(self.inter_packet_delay)
                        return  # ACK 성공 → 종료

                # ACK 못 받음 → 재시도
                if attempt < n - 1:
                    time.sleep(self.tx_repeat_interval)

            except Exception as e:
                print(f"[Serial Error]: {e}")
                return

        # 모든 시도 실패
        print(f"[FAIL→{sel}] No ACK after {n} attempts")

    def _parse_feedback(self, text):
        """
        Hand→PC 피드백 파서 (spec slide 7).
        Format: 'L|R, P,C,T(Thumb), P,C,T(Index), ..., P,C,T(ThumbAbd)'
        가장 최근 값을 self.last_feedback['left'/'right']에 저장.
        """
        for line in text.splitlines():
            parts = [p.strip() for p in line.split(',') if p.strip()]
            if not parts:
                continue
            head = parts[0].upper()
            hand = 'left' if head == 'L' else 'right' if head == 'R' else None
            if hand is None or len(parts) < 19:
                continue
            try:
                fingers = []
                for i in range(6):
                    base = 1 + i * 3
                    fingers.append({
                        'pos': int(parts[base]),
                        'cur': int(parts[base + 1]),
                        'temp': int(parts[base + 2]),
                    })
                self.last_feedback[hand] = {
                    'thumb': fingers[0], 'index': fingers[1],
                    'middle': fingers[2], 'ring': fingers[3],
                    'little': fingers[4], 'thumb_abd': fingers[5],
                }
            except (ValueError, IndexError):
                pass  # 파싱 실패해도 무시

    def send_motion(self, motion_name, selector='both', auto_reset=True):
        if motion_name not in motions:
            print(f"Invalid command name: {motion_name}")
            return

        items = motions[motion_name]
        for item in items:
            self.send_motor(item["pos"], selector)
            time.sleep(item["time"])

        # 슬라이드 6 권고: full-release 후 sensor slip 보정 위해 position counter 리셋
        if auto_reset and motion_name in RELEASE_MOTIONS:
            time.sleep(0.1)  # 모션 안정화 대기
            print("[INFO] Auto-reset position counter after release")
            self.reset(selector)

    def send_release(self, release_name=None, selector='both'):
        self.send_motion('unfold_a', selector)

    def close(self):
        if self.ser.is_open:
            self.ser.close()


if __name__ == "__main__":
    import readline

    try:
        hand = HandController('/dev/ttyACM0')  # L 컨트롤러 L동글 부터 연결
        print("컨트롤러 초기화 성공")
    except Exception as e:
        print(f"컨트롤러 초기화 실패: {e}")
        exit()

    while True:
        command_name = input("Enter command name,selector(left|right|both) (or 'exit'): ").strip()
        if command_name == "exit":
            break

        if command_name == "reset":
            hand.reset()
            continue
        if command_name == "preset":
            hand.preset()
            continue
        try:
            parts = [p.strip() for p in command_name.split(',')]
            if len(parts) != 2:
                raise ValueError("형식: <motion>,<left|right|both>")
            name, selector = parts
        except Exception as e:
            print(f"[입력 오류] {e}")
            continue
        hand.send_motion(name, selector)
