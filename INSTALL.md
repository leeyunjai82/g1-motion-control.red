# Installation Guide (INSTALL)

A step-by-step procedure for installing the Unitree G1 Motion Control project on a Red Hat–family OS. Follow each step in order.

- Usage : [`README.md`](./README.md)
- Internals (architecture, motion JSON schema, IK, DDS, FSM) : [`TECH.md`](./TECH.md)

---

## 1. System Requirements

| Item | Recommended |
| --- | --- |
| OS | RHEL 9 / RHEL 10 (or equivalent Rocky / AlmaLinux) |
| Python | 3.10 (via Miniconda) |
| Memory | 8 GB or more |
| Disk | 10 GB free space or more |
| Network | Same LAN as the G1 robot (wired recommended) |
| Camera | Intel RealSense D435i (USB 3.0) |
| Privileges | An account with sudo access |

Default install path: `/home/circulus/project/g1-motion-control/`

---

## 2. Base OS Packages

Install build tools, USB/camera libraries, and OpenGL components first.

```bash
sudo dnf groupinstall -y "Development Tools"
sudo dnf install -y \
    git wget curl \
    cmake \
    libusb1-devel \
    mesa-libGL mesa-libGLU \
    libglvnd-glx \
    glfw glfw-devel \
    python3-devel
```

---

## 3. Intel RealSense SDK

System libraries required by the RealSense D435i camera.

```bash
sudo dnf config-manager --add-repo \
    https://librealsense.intel.com/Debian/librealsense.repo
sudo dnf install -y librealsense2 librealsense2-utils librealsense2-devel
```

After installation, connect the camera to a USB 3.0 port and verify:

```bash
realsense-viewer
```

---

## 4. Miniconda Installation

Use a Miniconda environment to avoid affecting the system Python.

```bash
cd ~
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh -b -p $HOME/miniconda3
source $HOME/miniconda3/bin/activate
conda init bash
```

Open a new terminal and confirm with `conda --version`.

---

## 5. Create the `tv` Environment

This project uses a conda environment named `tv`.
(The scripts call `conda activate tv` internally, so keep the name.)

```bash
conda create -n tv python=3.10 -y
conda activate tv
```

---

## 6. Clone the Project

```bash
mkdir -p /home/circulus/project
cd /home/circulus/project
git clone https://github.com/leeyunjai82/g1-motion-control.git
cd g1-motion-control
```

---

## 7. Install Python Packages

`requirements.txt` defines roughly 100 packages, including FastAPI, OpenCV, PyTorch, MuJoCo, Ultralytics (YOLO), pyrealsense2, CycloneDDS, and the Unitree SDK.

```bash
conda activate tv
pip install --upgrade pip
pip install -r requirements.txt
```

> ⚠️ `unitree_sdk2py` is installed directly from GitHub (`-e git+...`).
> If the network is restricted, you will need an internal mirror or an offline wheel package.

If `cyclonedds` fails to build, install these first:

```bash
sudo dnf install -y openssl-devel bison flex
```

---

## 8. Sudo Configuration (for FSM Script)

`start_fsm.sh` runs Python with `sudo` because motor control requires elevated privileges. To avoid typing a password each time, add the following line via `visudo`:

```bash
sudo visudo
```
Add:
```
circulus ALL=(ALL) NOPASSWD: /home/circulus/miniconda3/envs/tv/bin/python
```

---

## 9. Network Setup

The G1 robot's default IP is `192.168.123.161`. Set the PC's wired LAN to the same subnet.

```bash
# Example: assign 192.168.123.222/24 to interface enp3s0
sudo nmcli connection modify "<connection-name>" ipv4.addresses 192.168.123.222/24
sudo nmcli connection modify "<connection-name>" ipv4.method manual
sudo nmcli connection up "<connection-name>"
```

Verify:
```bash
ping 192.168.123.161
```

---

## 10. Grant Execute Permissions

```bash
cd /home/circulus/project/g1-motion-control
chmod +x start_fsm.sh start_box.sh start_motion.sh activate_tv.sh
```

---

## 11. Verify the Installation

If the following all succeed, installation is complete.

```bash
# (1) Activate environment
source activate_tv.sh

# (2) Check Python packages
python -c "import cv2, torch, pyrealsense2, unitree_sdk2py; print('OK')"

# (3) Check RealSense
python -c "import pyrealsense2 as rs; print(rs.context().devices[0].get_info(rs.camera_info.name))"
```

If everything is OK, continue with [`README.md`](./README.md) for usage instructions, or [`TECH.md`](./TECH.md) for the internal architecture.

---

## Troubleshooting

| Symptom | Cause / Fix |
| --- | --- |
| `cyclonedds` build fails | Install `openssl-devel`, `bison`, `flex` and retry |
| RealSense not detected | Use a USB 3.0 port; verify with `realsense-viewer` first |
| `Permission denied` (FSM) | Check sudoers entry (step 8) |
| Cannot reach the robot | Check IP subnet and `firewalld` rules |
| Zombie processes remain | Clean up with `pkill -9 -f rs_stream.py` (etc.) and restart |


---
---
---


# 설치 가이드 (한국어)

Unitree G1 Motion Control 프로젝트를 Red Hat 계열 OS에 설치하는 절차입니다. 처음부터 끝까지 순서대로 따라가시면 됩니다.

- 사용법 : [`README.md`](./README.md)
- 내부 구조 (아키텍처, 모션 JSON 스키마, IK, DDS, FSM) : [`TECH.md`](./TECH.md)

---

## 1. 시스템 요구사항

| 항목 | 권장 사양 |
| --- | --- |
| OS | RHEL 9 / RHEL 10 (또는 Rocky / AlmaLinux 동등 버전) |
| Python | 3.10 (Miniconda 환경 사용) |
| 메모리 | 8 GB 이상 |
| 디스크 | 10 GB 이상 여유 공간 |
| 네트워크 | G1 로봇과 동일한 LAN (유선 권장) |
| 카메라 | Intel RealSense D435i (USB 3.0) |
| 권한 | sudo 권한이 있는 계정 |

설치 기본 경로는 `/home/circulus/project/g1-motion-control/` 입니다.

---

## 2. OS 기본 패키지 설치

빌드 도구, USB/카메라 라이브러리, OpenGL 등을 먼저 설치합니다.

```bash
sudo dnf groupinstall -y "Development Tools"
sudo dnf install -y \
    git wget curl \
    cmake \
    libusb1-devel \
    mesa-libGL mesa-libGLU \
    libglvnd-glx \
    glfw glfw-devel \
    python3-devel
```

---

## 3. Intel RealSense SDK 설치

RealSense D435i 카메라를 사용하기 위한 시스템 라이브러리입니다.

```bash
sudo dnf config-manager --add-repo \
    https://librealsense.intel.com/Debian/librealsense.repo
sudo dnf install -y librealsense2 librealsense2-utils librealsense2-devel
```

설치 후 카메라를 USB 3.0 포트에 연결하고 다음 명령으로 동작을 확인합니다.

```bash
realsense-viewer
```

---

## 4. Miniconda 설치

기본 시스템 Python을 건드리지 않기 위해 Miniconda 가상 환경을 사용합니다.

```bash
cd ~
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh -b -p $HOME/miniconda3
source $HOME/miniconda3/bin/activate
conda init bash
```

새 터미널을 열고 `conda --version` 으로 정상 설치를 확인합니다.

---

## 5. 가상환경 `tv` 생성

본 프로젝트는 `tv` 라는 이름의 conda 환경을 사용합니다.
(스크립트 내부에서 `conda activate tv` 로 호출되므로 이름을 그대로 사용하세요.)

```bash
conda create -n tv python=3.10 -y
conda activate tv
```

---

## 6. 프로젝트 클론

```bash
mkdir -p /home/circulus/project
cd /home/circulus/project
git clone https://github.com/leeyunjai82/g1-motion-control.git
cd g1-motion-control
```

---

## 7. Python 패키지 설치

`requirements.txt` 에는 약 100여 개의 패키지가 정의되어 있습니다. 주요 항목: FastAPI, OpenCV, PyTorch, MuJoCo, Ultralytics(YOLO), pyrealsense2, CycloneDDS, Unitree SDK 등.

```bash
conda activate tv
pip install --upgrade pip
pip install -r requirements.txt
```

> ⚠️ `unitree_sdk2py` 는 GitHub에서 직접 설치됩니다(`-e git+...`).
> 네트워크가 차단된 환경이라면 사내 미러 또는 오프라인 wheel 패키지가 별도로 필요합니다.

설치 중 `cyclonedds` 빌드 오류가 발생하면 다음을 먼저 실행하세요.

```bash
sudo dnf install -y openssl-devel bison flex
```

---

## 8. sudo 권한 설정 (FSM 스크립트용)

`start_fsm.sh` 는 모터 권한 때문에 `sudo` 로 Python 을 실행합니다. 매번 비밀번호 입력을 피하려면 sudoers 에 다음 한 줄을 추가합니다.

```bash
sudo visudo
```
추가 내용:
```
circulus ALL=(ALL) NOPASSWD: /home/circulus/miniconda3/envs/tv/bin/python
```

---

## 9. 네트워크 설정

G1 로봇의 기본 IP 는 `192.168.123.161` 입니다. PC 의 유선 LAN 을 동일 대역으로 설정합니다.

```bash
# 예시: enp3s0 인터페이스를 192.168.123.222/24 로 설정
sudo nmcli connection modify "<연결이름>" ipv4.addresses 192.168.123.222/24
sudo nmcli connection modify "<연결이름>" ipv4.method manual
sudo nmcli connection up "<연결이름>"
```

연결 확인:
```bash
ping 192.168.123.161
```

---

## 10. 실행 권한 부여

```bash
cd /home/circulus/project/g1-motion-control
chmod +x start_fsm.sh start_box.sh start_motion.sh activate_tv.sh
```

---

## 11. 설치 검증

다음이 정상 동작하면 설치 완료입니다.

```bash
# (1) 환경 활성화
source activate_tv.sh

# (2) Python 패키지 확인
python -c "import cv2, torch, pyrealsense2, unitree_sdk2py; print('OK')"

# (3) RealSense 확인
python -c "import pyrealsense2 as rs; print(rs.context().devices[0].get_info(rs.camera_info.name))"
```

이상이 없으면 [`README.md`](./README.md) 의 사용 방법으로 이동하거나, 내부 구조가 궁금하다면 [`TECH.md`](./TECH.md) 를 참고하세요.

---

## 문제 해결 (Troubleshooting)

| 증상 | 원인 / 해결 |
| --- | --- |
| `cyclonedds` 빌드 실패 | `openssl-devel`, `bison`, `flex` 설치 후 재시도 |
| RealSense 인식 안 됨 | USB 3.0 포트 사용, `realsense-viewer` 로 먼저 확인 |
| `Permission denied` (FSM) | sudoers 설정 확인 (8번 단계) |
| 로봇과 통신 불가 | IP 대역 및 방화벽(`firewalld`) 확인 |
| 좀비 프로세스 잔존 | `pkill -9 -f rs_stream.py` 등으로 정리 후 재시작 |
