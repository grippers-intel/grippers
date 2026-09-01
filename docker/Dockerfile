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
# 4. YOLO (CPU 추론) — 2026-08-21 임시 추가
#
#    Hailo HEF 파이프라인(§6)이 준비되기 전까지 PyTorch 기반 YOLO를 CPU로 돌리기
#    위해 컨테이너에 직접 pip install 했던 패키지들을 재현 가능하게 옮겨온 것.
#    버전은 그때 라이브 컨테이너에 실제로 설치되어 검증된 값과 맞춤.
#    Hailo 경로가 자리잡으면 이 블록은 제거될 수 있다 — 지금은 유지만 한다.
# ─────────────────────────────────────────────────────────────────────────────
RUN pip3 install --no-cache-dir \
      ultralytics==8.2.38 \
      torch==2.3.1 \
      torchvision==0.18.1 \
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
#    need_compile   : odom_publisher.launch.py · depth_camera.launch.py 가
#                     os.environ['need_compile']을 기본값 없이 직접 읽습니다
#                     — 빠지면 KeyError 로 launch 전체가 죽습니다. False 는
#                     "설치된 공유 경로 대신 벤더 소스트리 경로를 그대로
#                     쓴다"는 뜻으로, 이 이미지의 실제 마운트 구조와
#                     맞습니다(bringup.launch.py 의 기본 분기와 동일).
#    DEPTH_CAMERA_TYPE : depth_camera.launch.py 가 같은 방식으로 직접
#                     읽습니다. 이 로봇의 실측 카메라 종류(ascamera)로 고정.
#                     (2026-09-01: 실기 확인 결과 매번 손으로 export 하던
#                     값이라 이미지에 박아 컨테이너를 새로 만들어도 남게 함)
# ─────────────────────────────────────────────────────────────────────────────
ENV MACHINE_TYPE=MentorPi_Mecanum \
    SOARM_HEADLESS=1 \
    ROS_DOMAIN_ID=42 \
    PYTHONPATH=/grippers:/third_party/soarm_provided_d \
    need_compile=False \
    DEPTH_CAMERA_TYPE=ascamera

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

# ─────────────────────────────────────────────────────────────────────────────
# 8. exec_shell.sh 진입 자동화 (2026-09-01, 사용자 지시)
#
#    /home/ubuntu/ros2_ws/.zshrc(베이스 이미지 소유, .zshrc → .robotrc 체인의
#    끝단)는 벤더 워크스페이스(/home/ubuntu/ros2_ws)와 third_party_ws는
#    자동으로 source 하지만, 이 프로젝트의 워크스페이스(/ros2_ws == 컨테이너
#    기동 시 ${HOME}/docker/shared/grippers/ros2_ws 를 마운트한 자리, 이름만
#    같고 다른 디렉터리)는 안 건드립니다 — 그래서 매번 손으로
#    `source /ros2_ws/install/setup.zsh` 해야 했습니다.
#
#    ⚠️ /ros2_ws/install/setup.zsh 는 런타임에 마운트되는 파일이라 이미지
#    빌드 시점엔 없습니다 — 그래서 ENV로 못 박고, 셸 시작 시점에 source 하는
#    코드만 .zshrc 에 심어 둡니다. GRIPPERS_ENV_LOADED 가드는 같은 셸에서
#    .zshrc 가 중첩 로드될 때 중복 source 를 막습니다.
# ─────────────────────────────────────────────────────────────────────────────
RUN cat >> /home/ubuntu/ros2_ws/.zshrc << 'EOF'

# ── grippers 실기 bringup 자동화 (2026-09-01, 사용자 지시) ──────────────
# ros2 launch grippers_bringup bringup.launch.py 전에 매번 손으로 source
# 하던 프로젝트 워크스페이스를 exec_shell.sh 진입 시 자동으로 잡아 둔다.
# need_compile/DEPTH_CAMERA_TYPE 은 이 Dockerfile의 ENV로 이미 고정돼 있어
# 여기서 다시 export 하지 않는다.
if [ -z "$GRIPPERS_ENV_LOADED" ]; then
  source /ros2_ws/install/setup.zsh
  export GRIPPERS_ENV_LOADED=1
fi
EOF

CMD ["/bin/zsh"]
