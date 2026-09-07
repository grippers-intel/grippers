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
# 손가락 판이 바구니 위로 넓게 쓸릴 뿐 얻는 것이 없다는 근거였다.
#
# ⚠️ 2026-09-04 사용자 지시로 뒤집혔다 — host+Pi 연동 실기에서 물체가
# 그리퍼에서 안 떨어지는 사고가 반복됐다("바구니에 내려놓을 때, 그리퍼
# 최대로 열어. 물체가 그리퍼에서 안 떨어져"). `_release_width()`가 이제
# 이 상수를 안 쓰고 무조건 `GRIPPER_MAX_SAFE_OPEN_MM`까지 연다 — 이
# 상수는 그 이전 결정의 기록으로만 남겨 둔다.
GRIPPER_RELEASE_MM = 15.0


# ⚠️ 2026-09-07 사용자 지시로 **파지 폭 정책을 통째로 들어냈다.**
#
#   "kica927에서 가져온 파지 정책 제거해줘. 파지는 내 역할이니까."
#
# 지운 것: GRIPPER_SQUEEZE_MM, GRIPPER_GRASP_MIN_MM, _close_width(),
# 그리고 HorizontalGraspPlan 의 preopen_width_mm / close_width_mm.
#
# 저 값들이 "얼마나 벌리고 얼마나 조일 것인가"를 정하던 자리다 —
# 라벨마다 다른 하한, box/star 예외(20mm), 백래시를 감안한 0.0mm 밀어붙임.
# 전부 팀원 브랜치에서 온 실기 튜닝이었고, 사용자가 직접 만들 부분이다.
#
# 지금 파지에서 그리퍼를 모는 것은 **정책 하나뿐**이다. 미션은 판정 직전에
# 한 번 확실히 닫으라고만 하고(baseline_mission 의 그 주석), 그때 쓰는 폭은
# 프로파일이 아니라 0.0mm 상수다. 그래서 이 값들은 이미 아무도 안 읽고
# 있었다 — 남겨 두면 "여기가 파지 폭을 정하는 곳"으로 읽힌다.
#
# 남긴 것은 투하(INSERT)에 쓰는 _release_width 뿐이다. 그것은 파지가 아니라
# 놓기다.


def _release_width(object_width_mm: float) -> float:
    """투하 시 여는 폭 — 2026-09-04부터 무조건 최대 안전폭이다.

    예전엔 물체가 턱 사이에서 빠져나올 만큼만(GRIPPER_RELEASE_MM 여유)
    벌렸는데, 실기에서 물체가 그리퍼에 계속 걸려 안 떨어지는 사고가
    나서 사용자 지시로 최대로 여는 쪽으로 바꿨다(GRIPPER_RELEASE_MM
    주석 참고). `object_width_mm` 인자는 더 안 쓰지만, 호출부
    (`HorizontalGraspPlan` 생성)를 안 건드리려고 시그니처는 그대로
    둔다."""
    return GRIPPER_MAX_SAFE_OPEN_MM


@dataclass(frozen=True)
class HorizontalGraspPlan:
    profile: str
    release_width_mm: float
