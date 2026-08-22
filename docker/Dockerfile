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
# 4. 카메라 이미지 처리 (cv2)
#
#    depth_cam_rotate_node · gripper_cam_publisher_node · perception_node의
#    confirm_grasp · tools/hailo/live_yolo_demo.py가 전부 opencv를 쓴다.
#
#    ⚠️ 2026-08-21: PyTorch 기반 YOLO CPU 추론 대비용으로 ultralytics/torch/
#    torchvision도 잠깐 여기 있었다. Hailo-10H 경로가 실기에서 230 FPS로
#    검증되고 나서(tools/hailo/README_LIVE_DEMO.md 참고) 안 쓰는 게 확정돼
#    지웠다 — CPU YOLO로 되돌아갈 일이 생기면 그때 다시 추가할 것.
# ─────────────────────────────────────────────────────────────────────────────
RUN pip3 install --no-cache-dir \
      opencv-python-headless==4.10.0.84

# ─────────────────────────────────────────────────────────────────────────────
# 5. 환경변수
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
# 6. Hailo-10H 런타임
# ─────────────────────────────────────────────────────────────────────────────

# Vendor binaries are not tracked by Git.
# If both HailoRT 5.1.1 files exist in docker/vendor/, install Hailo support.
# Otherwise the image remains buildable without HailoRT.
#
# BuildKit bind mount keeps the .deb/.whl out of the final image layers.
# Exact filenames and download instructions: docker/vendor/README.md
RUN --mount=type=bind,source=docker/vendor,target=/vendor \
    if [ -f /vendor/h10-hailort_5.1.1_arm64.deb ] \
       && [ -f /vendor/hailort-5.1.1-cp310-cp310-linux_aarch64.whl ]; then \
        mkdir -p /tmp/hailort-root \
        && dpkg-deb -x /vendor/h10-hailort_5.1.1_arm64.deb /tmp/hailort-root \
        && cp /tmp/hailort-root/usr/lib/libhailort.so.5.1.1 /usr/lib/ \
        && ln -sf /usr/lib/libhailort.so.5.1.1 /usr/lib/libhailort.so \
        && cp /tmp/hailort-root/usr/bin/hailortcli /usr/bin/ \
        && ldconfig \
        && pip3 install --no-cache-dir \
            /vendor/hailort-5.1.1-cp310-cp310-linux_aarch64.whl \
        && rm -rf /tmp/hailort-root \
        && echo "[hailo] HailoRT 5.1.1 installed"; \
    else \
        echo "[hailo] vendor .deb/.whl not found; building without HailoRT"; \
    fi

# ─────────────────────────────────────────────────────────────────────────────
# 7. 사용자
#
#    exec_shell.sh 가 `-u ubuntu` 로 진입합니다.
#    dialout — 시리얼 장치 접근 (/dev/ttyACM0=베이스보드, /dev/ttyACM1=SO-ARM101)
#    video   — 카메라 장치 접근
# ─────────────────────────────────────────────────────────────────────────────
RUN usermod -aG dialout,video ubuntu || true

USER ubuntu
WORKDIR /ros2_ws
CMD ["/bin/zsh"]
