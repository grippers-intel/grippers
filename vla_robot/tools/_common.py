"""도구 공용: 저장소 경로를 잡고 설정·버스를 연다.

⚠️ 이 도구들은 시리얼 버스를 **직접** 연다. arm_driver_node 가 떠 있으면 포트 독점
때문에 열리지 않는다(의도된 동작). 먼저 노드를 멈출 것:
    ros2 launch ... use_arm:=false   또는 해당 프로세스 종료
"""
from __future__ import annotations

import sys
from pathlib import Path


def configure_console() -> None:
    """Windows 한글 콘솔(cp949)은 '—', '⚠️' 를 인코딩하지 못해 print 에서 죽는다.
    인코딩은 그대로 두고 못 그리는 글자만 '?' 로 바꾼다. import 시점에 한 번 건다.

    도구들은 Pi(UTF-8)뿐 아니라 **노트북에 팔을 직접 꽂고** 도 쓴다(녹화 때 COM8).
    그쪽에서 --help 조차 UnicodeEncodeError 로 죽던 것을 막는다."""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="replace")
        except (AttributeError, ValueError):
            pass


configure_console()

ROOT = Path(__file__).resolve().parents[1]
for _p in (ROOT / "ros2_ws" / "src" / "vla_common", ROOT / "ros2_ws" / "src" / "vla_robot_arm"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from vla_common.arm_units import ArmCalibration  # noqa: E402
from vla_common.config import load_robot_config  # noqa: E402
from vla_robot_arm.feetech_bus import BusError, FeetechBus  # noqa: E402

DEFAULT_CONFIG = ROOT / "ros2_ws" / "src" / "vla_robot_bringup" / "config" / "robot.yaml"


def add_common_args(parser) -> None:
    parser.add_argument("--config", default=str(DEFAULT_CONFIG), help="robot.yaml 경로")
    parser.add_argument("--port", default="", help="arm.port 덮어쓰기 (예: /dev/ttyACM0, COM8)")


def open_from_args(args):
    cfg = load_robot_config(args.config)
    calib = ArmCalibration.load(cfg.arm.calibration_file)
    port = args.port or cfg.arm.port
    try:
        bus = FeetechBus(port, cfg.arm.baudrate, retries=cfg.arm.read_retries).open()
    except BusError as exc:
        print(f"버스를 열지 못했다: {exc}\n  arm_driver_node 가 떠 있으면 먼저 멈출 것.")
        raise SystemExit(2)
    return cfg, calib, bus


def confirm(message: str, assume_yes: bool) -> bool:
    if assume_yes:
        return True
    try:
        return input(f"{message} 계속하려면 yes 입력: ").strip().lower() == "yes"
    except EOFError:
        return False
