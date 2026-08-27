"""실측 수평 파지 프로필의 폭 계산.

⚠️ 2026-08-27: 여기 있던 bbox 폭 휴리스틱(select_horizontal_grasp_plan·
approach_target_key)을 지웠다. YOLO subtype이 없던 시절 물체 폭만으로
프로필을 추측하던 경로인데, 지금 Pi YOLO(train-9)는 클래스 이름을 직접
주므로 baseline_mission.plan_for_label()이 라벨로 바로 고른다 — 이
휴리스틱은 저장소 어디서도 안 불리는 죽은 코드였다(코드 리뷰로 발견,
사용자 확인 후 삭제).
"""

from dataclasses import dataclass

# ros2_ws/src/grippers_arm/grippers_arm/gripper_calibration.py의 GRIPPER_OPEN_MM
# 실측값과 같은 수다. domain 계층은 그 ROS 패키지를 import하지 않으므로(계층
# 분리) 값만 그대로 복제해 둔다 — 그리퍼 기구가 바뀌어 GRIPPER_OPEN_MM이
# 바뀌면 여기도 같이 바꿔야 한다. 2026-08-24: 사용자 지시로 80.0(임의로 절반쯤
# 열기)에서 이 안전 최대치로 올림.
GRIPPER_MAX_SAFE_OPEN_MM = 168.0

# 투하 시 물체 폭에 더할 여유 (mm). ros2_ws 쪽 GRIPPER_RELEASE_MM과 같은 수를
# 계층 분리 때문에 복제해 둔다 — 한쪽을 바꾸면 다른 쪽도 바꿔야 한다.
#
# 2026-08-25 사용자 지시: "물체를 놓을 때 완전히 벌리지 말고 물체가 그리퍼
# 사이에서 나올 정도로만 벌려." GRIPPER_MAX_SAFE_OPEN_MM(168)까지 열면
# 손가락 판이 바구니 위로 넓게 쓸릴 뿐 얻는 것이 없다.
GRIPPER_RELEASE_MM = 15.0


# ros2_ws 쪽 GRIPPER_SQUEEZE_MM · GRIPPER_GRASP_MIN_MM와 같은 수를 계층 분리
# 때문에 복제해 둔다 — 한쪽을 바꾸면 다른 쪽도 바꿔야 한다.
#
# ⚠️ 2026-08-26: 실제로 그 동기화가 깨져 있었다. 이 파일이 파지 폭을
# 13.0/13.0/15.0/30.0/35.0으로 **하드코딩**하고 있었는데, ros2 쪽은
# _close_width(물체폭 - 15.0, 하한 7.0)로 7.0/7.0/9.5/25.0/31.0을 낸다.
# 미션이 실제로 쓰는 것은 이 파일 값이라, 08-25에 하한을 9.0 -> 7.0으로
# 내리며 "최대한 세게 잡자"고 한 사용자 지시가 도메인 경로에는 반영되지
# 않은 채였다. 계산식을 옮겨 와 다시는 어긋나지 않게 한다.
GRIPPER_SQUEEZE_MM = 15.0
GRIPPER_GRASP_MIN_MM = 7.0


def _close_width(object_width_mm: float) -> float:
    """물체 폭에서 GRIPPER_SQUEEZE_MM만큼 더 좁힌 목표 폭.

    빈 닫힘 폭이 아니라 **파지 전용** 하한으로 clamp한다 — 물체가 턱을 멈춰
    주므로 파지 때는 더 좁게 명령해 위치 오차(=힘)를 키울 수 있다."""
    return max(GRIPPER_GRASP_MIN_MM, round(object_width_mm - GRIPPER_SQUEEZE_MM, 1))


def _release_width(object_width_mm: float) -> float:
    """물체가 턱 사이에서 빠져나올 만큼만 벌린 목표 폭."""
    return min(GRIPPER_MAX_SAFE_OPEN_MM, round(object_width_mm + GRIPPER_RELEASE_MM, 1))


@dataclass(frozen=True)
class HorizontalGraspPlan:
    profile: str
    preopen_width_mm: float
    close_width_mm: float
    release_width_mm: float
