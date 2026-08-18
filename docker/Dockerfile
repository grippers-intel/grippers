# syntax=docker/dockerfile:1
#
# IntelPi — grippers 프로젝트 개발/실행 컨테이너
#
# ─────────────────────────────────────────────────────────────────────────────
# 배경
# ─────────────────────────────────────────────────────────────────────────────
# 기존 IntelPi 는 MentorPi 출고 이미지를 docker export 한 `ros:humble-export` 였고,
# 재현 가능한 정의가 없었습니다 (hld.md 미결 #1). 이 파일이 그 정의입니다.
#
# ⚠️ 베이스 이미지 제약
#   `ros:humble-export` 는 공개 레지스트리에 없고 Pi 로컬에만 존재합니다.
#   Hiwonder MentorPi 벤더 드라이버(ros_robot_controller 등)가 Humble 로 컴파일된
#   상태로 들어있고 그 소스를 보유하고 있지 않아, 밑바닥부터 재구성할 수 없습니다.
#   따라서 이 Dockerfile 은 **Pi 로컬 재현용**이며 CI 에서는 빌드할 수 없습니다.
#
# ─────────────────────────────────────────────────────────────────────────────
# 빌드 · 실행
# ─────────────────────────────────────────────────────────────────────────────
#   docker build -f docker/Dockerfile -t intelpi:latest .
#   ./ros_start.sh      # 볼륨 마운트 · 장치 패스스루는 이 스크립트가 담당
#   ./exec_shell.sh
#
FROM ros:humble-export

USER root
SHELL ["/bin/bash", "-c"]

# ─────────────────────────────────────────────────────────────────────────────
# 1. ROS2 APT GPG 키 갱신
#
#    Open Robotics 키(F42ED6FBAB17C654)가 만료되어 `apt update` 가 실패합니다:
#      EXPKEYSIG F42ED6FBAB17C654 Open Robotics <info@osrfoundation.org>
#    이 단계가 없으면 이후 모든 apt install 이 막힙니다.
# ─────────────────────────────────────────────────────────────────────────────
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && apt-get update

# ─────────────────────────────────────────────────────────────────────────────
# 2. 시스템 의존성
# ─────────────────────────────────────────────────────────────────────────────
RUN apt-get install -y --no-install-recommends \
      python3-pip \
      udev \
      curl \
      ca-certificates \
 && rm -rf /var/lib/apt/lists/*

# ─────────────────────────────────────────────────────────────────────────────
# 3. Python 의존성
#
#    pyserial : SO-ARM101 (Feetech STS3215) 시리얼 통신
#    mujoco   : soarm_lab 의 down=True 파지 IK 가 방향 자코비안 계산에 사용합니다.
#               시뮬 전용이 아니라 **실물 경로에서도 필요**합니다 — 빼면 arm_driver 가
#               파지 모드에서 죽습니다.
#    numpy    : soarm_lab 전반
#    pytest   : 컨테이너 안에서 domain 테스트를 돌릴 때 사용
# ─────────────────────────────────────────────────────────────────────────────
RUN pip3 install --no-cache-dir \
      pyserial \
      mujoco \
      numpy \
      pytest

# ─────────────────────────────────────────────────────────────────────────────
# 4. 환경변수
#
#    MACHINE_TYPE   : controller/odom_publisher_node 가 이 값으로 mecanum 역기구학
#                     경로를 선택합니다 (os.environ['MACHINE_TYPE'] 직접 참조).
#                     빠지면 KeyError 로 노드가 죽습니다.
#    SOARM_HEADLESS : soarm_lab 의 SimBackend 가 MuJoCo 뷰어를 띄우지 않게 합니다.
#                     없으면 디스플레이 없는 환경에서 xvfb-run 이 필요해집니다.
#    ROS_DOMAIN_ID  : 호스트(Debian trixie + ROS2 Jazzy)와 DDS 도메인을 분리합니다.
#                     ros_start.sh 가 --network=host 를 쓰므로 이 격리가 필요합니다.
#                     팀 규칙: ros2 명령은 컨테이너 안에서만 실행 (미결 #13 해소).
#    PYTHONPATH     : /grippers                    — domain 계층 (ROS2 비의존)
#                     /third_party/soarm_provided_d — soarm_lab
# ─────────────────────────────────────────────────────────────────────────────
ENV MACHINE_TYPE=MentorPi_Mecanum \
    SOARM_HEADLESS=1 \
    ROS_DOMAIN_ID=42 \
    PYTHONPATH=/grippers:/third_party/soarm_provided_d

# ─────────────────────────────────────────────────────────────────────────────
# 5. Hailo-10H 런타임 — 보류 (미결 #14)
#
#    2026-08-18 검증 결과:
#      ✅ libhailort.so / hailortcli — 호스트(Debian trixie)용 .deb 를 그대로 풀어도
#         이 컨테이너(Ubuntu 22.04 jammy)에서 정상 동작. HAILO10H 인식 확인.
#         의존성이 libstdc++/libm/libgcc_s/libc 뿐이라 glibc 차이를 타지 않습니다.
#      ❌ Python 바인딩 — _pyhailort.cpython-**313**-aarch64-linux-gnu.so 로
#         Python 3.13 전용 빌드. 이 컨테이너는 Python 3.10 이라 import 불가.
#         PyPI 에도 hailort 패키지가 없습니다 (No matching distribution found).
#
#    필요한 것: Hailo Developer Zone 에서 HailoRT **5.1.1** + **cp310** + **aarch64**
#              Python 휠. 드라이버(h10-hailort-pcie-driver 5.1.1)와 런타임 버전이
#              정확히 일치해야 하므로 버전을 임의로 올리면 안 됩니다.
#
#    휠 확보 후 추가할 블록:
#      COPY docker/vendor/hailort-5.1.1-cp310-cp310-linux_aarch64.whl /tmp/
#      RUN pip3 install --no-cache-dir /tmp/hailort-*.whl && rm /tmp/hailort-*.whl
#
#    커널 드라이버는 호스트 것을 그대로 사용합니다 — 컨테이너에는 유저스페이스
#    라이브러리와 Python 바인딩만 필요하고, 장치 접근은 ros_start.sh 의
#    `-v /dev:/dev` 로 이미 확보되어 있습니다.
# ─────────────────────────────────────────────────────────────────────────────

# ─────────────────────────────────────────────────────────────────────────────
# 6. 사용자
#
#    exec_shell.sh 가 `-u ubuntu` 로 진입합니다.
#    dialout — 시리얼 장치 접근 (/dev/ttyACM0=베이스보드, /dev/ttyACM1=SO-ARM101)
#    video   — 카메라 장치 접근
# ─────────────────────────────────────────────────────────────────────────────
RUN usermod -aG dialout,video ubuntu || true

USER ubuntu
WORKDIR /ros2_ws
CMD ["/bin/zsh"]
