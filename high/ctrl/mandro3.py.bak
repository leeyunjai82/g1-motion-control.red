import serial
import time
import math

# --- [Helper Functions] ---
# 엄지 손가락 위치 조정
def f1_pos(ratio):
    base_value = 0x50 
    result_int = math.floor(base_value * ratio)
    return result_int & 0xFF

# 엄지 제외 손가락 위치 조정
def f24_pos(ratio):
    base_value = 0x80 
    result_int = math.floor(base_value * ratio)
    return result_int & 0xFF

def create_command(fingers, ratio):

    position = [f1_pos(ratio), f24_pos(ratio), f24_pos(ratio), f24_pos(ratio), f24_pos(ratio), 0x0 ] 

    finger_sel = int("".join(map(str, fingers[::-1])), 2)

    command = [0xFF]                 # Bytes 0 : left(0xFD), right(0xFE), both(0xFF)
    command.append(finger_sel)          # Bytes 1: 손가락 활성화 상태
    command.extend([0xB0, 0xA0]) # Bytes 2 speed, Bytes 3 current
    command.extend(position)         # Bytes 4-8: 위치값 (-5 ~ 400) / Bytes 9: reserved
    command.append(0x1)        # Byte 10: 방향 (forward)
    
    # 동작 시간 계산 (대략적인 값)
    return {'pos': command, 'time': sum(e==1 for e in fingers) * 0.5}

# --- [Motion Dictionary] ---
motions = {
    # --- 기본 동작 ---
    "unfold_a": [ 
        create_command([0, 1, 1, 1, 1], 0.01),
        create_command([1, 0, 0, 0, 0], 0.01),
    ],
    "fold_a": [ 
        create_command([1, 0, 0, 0, 0], 1.0),
        create_command([0, 1, 1, 1, 1], 1.0),
    ],
    "test": [ 
        create_command([1, 0, 0, 0, 0], 1.0),
        create_command([0, 1, 1, 1, 1], 0.3),
    ],
    "fold_ha": [ 
        create_command([1, 0, 0, 0, 0], 0.5),
        create_command([0, 1, 1, 1, 1], 0.5),
    ],
    # --- 조합 동작 ---
    "point": [
        create_command([0, 0, 1, 1, 1], 1.0),
        create_command([1, 0, 0, 0, 0], 1.0),
    ],
    "handshake": [
        create_command([0, 1, 1, 1, 1], 0.4),
    ],
    "ok": [
        create_command([1, 0, 0, 0, 0], 0.5),
        create_command([0, 1, 0, 0, 0], 0.9),
    ],
    "thumbup": [
        create_command([0, 1, 1, 1, 1], 1.0),
    ],
    "victory": [
        create_command([1, 0, 0, 0, 0], 1.0),
        create_command([0, 0, 0, 1, 1], 1.0),
    ],
    "rock": [
        create_command([0, 0, 1, 1, 0], 1.0),
    ],
    "promise": [
        create_command([1, 0, 0, 0, 0], 1.0),
        create_command([0, 1, 1, 1, 0], 1.0),
    ],
    "grab": [
        create_command([1, 0, 0, 0, 0], 0.7),
        create_command([0, 1, 1, 1, 1], 0.5),
    ],
}

# --- [Controller Class] ---
class HandController:
    def __init__(self, port):
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
        time.sleep(1) # 포트 안정화 대기

    def reset (self):
        self.send_motor([0xff, 0x1f, 0, 0, 0, 0, 0, 0, 0, 0, 0x3])
    def preset (self):
        self.send_motor([0xff, 0x1f, 0, 0, 0, 0, 0, 0, 0, 0, 0x4])

    def send_motor(self, data, selector='both'):
        cmd_data = data.copy()

        # ID 헤더 변경
        if selector == 'left':
            cmd_data[0] = 0xFD
        elif selector == 'right':
            cmd_data[0] = 0xFE
        else:
            cmd_data[0] = 0xFF # both
        
        print(cmd_data)
        # [중요] \n (줄바꿈)을 포함하여 바이트 변환
        data_bytes = bytes(cmd_data) + b'\n'

        try:
            # 1. 입력 버퍼 비우기 (이전 응답 찌꺼기 제거)
            self.ser.reset_input_buffer()
            
            # 2. 데이터 전송
            self.ser.write(data_bytes)
            self.ser.flush() # 전송 완료 대기
            
            # 3. 처리 시간 대기
            time.sleep(0.1)

            # 4. 안전한 데이터 읽기 (Blocking 방지)
            # readline() 대신 in_waiting 체크 후 read() 사용
            if self.ser.in_waiting > 0:
                response = self.ser.read(self.ser.in_waiting)
                # 디코딩 에러 무시하고 출력 (필요 시 주석 처리 가능)
                try:
                    print(f"[RX]: {response.decode(errors='ignore').strip()}")
                except:
                    pass

        except Exception as e:
            print(f"[Serial Error]: {e}")
            # 연결이 끊겼을 경우 재연결 로직이 필요하다면 여기에 추가

    def send_motion(self, motion_name, selector='both'):
        if motion_name not in motions:
            print(f"Invalid command name: {motion_name}")
            return

        items = motions[motion_name]
        for item in items:
            self.send_motor(item["pos"], selector)
            time.sleep(item["time"])

    def send_release(self, release_name=None, selector='both'):
        self.send_motion('unfold_a', selector)

    def close(self):
        if self.ser.is_open:
            self.ser.close()


if __name__ == "__main__":
  import readline

  try:
    hand = HandController('/dev/ttyACM0') # L 컨트롤러 L동글 부터 연결
    print("컨트롤러 초기화 성공")
  except Exception as e:
    print(f"컨트롤러 초기화 실패: {e}")
    exit()
 
  while True:
    # Example usage
    command_name = input("Enter command name,selector(left|right|both) (or 'exit'): ")
    if command_name == "exit":
      break

    if command_name == "reset":
      hand.reset()
      continue
    if command_name == "preset":
      hand.preset()
      continue
    try:
      name, selector = command_name.split(',')
    except Exception as e:
      print(e)
      continue
    hand.send_motion(name, selector)

  while False:
    hand.send_motion("fold_a")
    time.sleep(1.8)
    hand.send_motion("unfold_a")
    time.sleep(1.8)
