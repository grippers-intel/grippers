"""Measured floor-grasp targets for the current SO-ARM101 end effector.

Object geometry is kept separate from the named, hardware-tested arm poses so
that a low GABE pose is not accidentally reused for taller chess pieces.
"""

from dataclasses import dataclass

# 절대 import를 쓴다 — align_to_idle.py 등 tools/*.py의 grippers_arm 참조와
# 같은 방식이다. 이 파일은 tests/test_floor_grasp_profiles.py에서
# importlib.util.spec_from_file_location으로 단독 로드되기도 하는데, 그
# 경로에선 패키지 컨텍스트가 없어 상대 import(`from .gripper_calibration`)가
# "attempted relative import with no known parent package"로 깨진다.
from grippers_arm.gripper_calibration import GRIPPER_GRASP_MIN_MM, GRIPPER_OPEN_MM

# ═══════════════════════════════════════════════════════════════════════════
# 이 파일의 RAW 자세가 유효하려면 팔이 이 캘리브레이션이어야 한다
#
# 아래 자세들은 전부 RAW 서보값이다. 그런데 RAW 값이 가리키는 물리 자세는
# 서보 EEPROM 의 Homing_Offset 에 달려 있다.
#
#     Present_Position = Actual_Position - Homing_Offset
#
# 즉 오프셋이 바뀌면 **같은 숫자가 다른 자세**가 된다. 그래서 자세만 적어
# 두는 것으로는 부족하다 — 어떤 오프셋 아래에서 잰 값인지 같이 적는다.
#
# 아래 값은 2026-08-29 18:11:24 에 팔로워(COM8)에서 읽은 것으로,
# **LeRobot 캘리브레이션을 돌리기 직전**의 상태다. 원본은
# tools/arm/servo_backup/servo_COM8_20260829_181124.json 에 있다.
#
# arm_driver_node 가 기동할 때 이 값과 대조한다(calib_identity.py). 다르면
# 기동을 거부한다 — sysy009 실측으로 shoulder_pan 가동폭이 2493 -> 2087 로
# 줄어 있어(차체·라이다에 막힘), 어긋난 채 움직이면 부딪힌다.
#
# 팔을 다시 교시했다면 이 값도 같이 갱신할 것. 둘은 한 쌍이다.
# ═══════════════════════════════════════════════════════════════════════════
TAUGHT_HOMING_OFFSETS = {
    1: -1945,   # shoulder_pan
    2: -1762,   # shoulder_lift
    3: 1307,    # elbow_flex
    4: 1760,    # wrist_flex
    5: -1848,   # wrist_roll
    6: 1343,    # gripper
}

# ⚠️ 2026-09-01 실기, 이날의 가장 큰 발견: Homing_Offset만 되돌리는 것으로는
# 부족하다. 그리퍼(servo 6)가 어떤 폭을 명령해도 전혀 안 닫히는 사고가
# 났는데, `restore_taught_offsets.py --apply --yes`로 Homing_Offset은
# 이미 복구된 상태였다 — 원인은 Min/Max_Angle_Limit(EEPROM 주소 9/11,
# 서보가 실제로 움직일 수 있는 물리적 허용 범위)이었다. LeRobot/VLA
# 캘리브레이션은 이 레지스터를 Homing_Offset과 **같이** 덮어쓰는데(그날
# 실측: 1140~2090 -> 1960~2378), 그 복구 도구는 이 레지스터를 아예
# 보지도 쓰지도 않았다. "닫힘"에 필요한 목표(raw ~1150)가 서보 펌웨어의
# 허용 범위 밖이라 조용히 무시되고 있었다 — ROS 서비스는 ok=True를 그대로
# 반환했다(set_position()이 ACK는 받지만 목표 자체가 서보 안에서 버려짐).
#
# 그날은 스크래치패드 즉석 스크립트로 레지스터를 직접 복구했고 저장소에는
# 반영되지 않았다. 아래 값은 그 즉석 복구가 확인한 값이자, TAUGHT_HOMING_
# OFFSETS와 같은 백업 파일(08-29 18:11:24, LeRobot 캘리브레이션 돌리기
# **직전** 스냅샷)에서 나온 것이다 — 둘은 같은 순간의 같은 팔 상태를
# 담고 있으므로 반드시 같이 갱신해야 한다.
TAUGHT_POSITION_LIMITS = {
    1: (932, 3425),     # shoulder_pan
    2: (817, 3196),     # shoulder_lift
    3: (889, 3101),     # elbow_flex
    4: (870, 3224),     # wrist_flex
    5: (129, 3995),     # wrist_roll
    # ⚠️ 2026-09-07 에 하한을 1140 -> 1090 으로 내렸다 (사용자 지시).
    #
    # 1140 은 서보가 허용하는 하한이었고, 그래서 그리퍼가 1144~1147 에서
    # 멈췄다. 그동안 그것을 **기계적 한계**로 알고 있었다 — 2026-08-25 힘
    # 프로브 주석도 "턱은 raw 1144에서 기계적으로 멈춘다"고 적었다.
    #
    # 아니었다. 그 프로브 결과 자체가 반증이었는데 못 읽었다: "그보다 좁게
    # 명령해도 위치가 안 변하고 **부하도 안 는다**(0.0274 고정)". 진짜 기계적
    # 접촉이면 더 좁게 명령할수록 위치 오차가 커져 부하가 올라간다. 부하가
    # 고정이라는 건 서보가 아예 안 밀고 있다는 뜻이고, 그게 소프트웨어
    # 제한의 특징이다.
    #
    # 2026-09-07 실측으로 확정: 토크를 끄고 손으로 눌렀더니 **1106** 까지
    # 닫혔다. 제한값 1140 보다 34 raw 더 간다.
    #
    # 이게 파지 실패의 직접 원인이었다. VLA 정책은 닫힘을 raw 1100 으로
    # 명령하는데(학습 데이터의 grip 20 units), 서보가 1140 에서 잘라 40 raw
    # 만큼 덜 조였다. 물체가 헐겁게 물려 들어올릴 때 빠졌다.
    #
    # 1090 은 손으로 도달한 1106 보다 조금 아래다 — 정책의 1100 이 잘리지
    # 않으면서, 물리적으로 확인된 범위를 크게 벗어나지도 않는다.
    #
    # ⚠️ 2026-09-07 저녁, 하한을 다시 1090 -> 1007 로 내렸다.
    #
    # 위 1090 은 "손으로 눌러 도달한 1106 보다 조금 아래" 라는 기준으로
    # 잡은 값이다. 그 기준이 틀렸다. 사용자 관찰이 그것을 뒤집었다 —
    # "텔레옵을 했을 때 팔로암도 리더암 같은 파지력이 나왔었는데 지금은
    # 그만큼 안 나온다."
    #
    # 무는 힘은 P x (목표 - 현재) 라 **도달하지 못하는 거리**에서 나온다.
    # 턱이 물체에서 멈춘 뒤에도 서보가 계속 밀어야 하므로, 하한을 기계적
    # 스토퍼 근처로 잡으면 밀 여지 자체가 없어진다. 1106 에서 멈추는 턱에
    # 하한 1090 을 주면 남는 여지가 16 raw 뿐이다.
    #
    # 1007 은 지어낸 값이 아니라 **텔레옵이 실제로 쓰던 자리**다. lerobot
    # 캘리브레이션(host/vla/calibration/grippers_arm.json)의 그리퍼
    # range_min = 1960 을 지금 프레임으로 옮긴 값이다
    # (Present = Actual - Homing, 390 - 1343 = -953, 1960 - 953 = 1007).
    # 텔레옵이 몇 달을 그 하한으로 돌았으니 검증된 범위이기도 하다.
    #
    # 퀸(실측 1190)을 물었을 때 위치 오차가 100 -> 183 raw 로 늘어난다.
    #
    # ⚠️ Homing_Offset 은 안 건드렸다(6: 1343 그대로). 프레임이 그대로이니
    # 아래 RAW 교시 자세는 전부 유효하다 — 이 변경은 가동 범위의 **아래쪽을
    # 넓히기만** 한다. 2026-09-01 사고(범위 밖이라 목표가 조용히 버려짐)와
    # 반대 방향이다.
    #
    # ── 2026-09-08: 1007 -> 1090 으로 되돌린다 ───────────────────────────
    #
    # 실기에서 arm_driver 가 이것 때문에 죽었다:
    #
    #     ArmCalibrationMismatchError: Min_Angle_Limit이 교시 당시와 다릅니다
    #       servo 6: 교시 1007 · 현재 1090 (+83 카운트 = +7.3도)
    #
    # 하드웨어가 1090 이다. 팀원 브랜치(kica927/baseline_mission)가 2026-09-07
    # 에 그렇게 잡았고 하루 종일 그 값으로 돌았다 — 사용자도 "VLA 기반에서는
    # 물체를 딱히 떨어뜨린 적이 없다"고 했는데, 그 파지들이 전부 1090 에서
    # 난 것이다.
    #
    # **팀 공용 EEPROM 을 이 브랜치에 맞추지 않고 이 브랜치를 하드웨어에
    # 맞춘다.** 하드웨어를 1007 로 되돌리는 것은 무는 힘을 83 raw 만큼 더
    # 주는 대신, servo 6 이 과전류로 버스에서 떨어지던 바로 그 설정으로
    # 돌아가는 일이다(2026-09-07, 온도 50°C — 다른 서보는 32~35°C). 그
    # 문제는 아직 안 풀렸고, 파지는 사용자 영역이다.
    #
    # 되돌리고 싶으면(무는 힘을 다시 깊게):
    #     tools/arm/gripper_grip_gain.py --restore-teleop-grip
    # 그러면 이 값도 1007 로 같이 고쳐야 한다 — 안 고치면 arm_driver 가
    # 위 예외로 다시 죽는다. 둘은 **짝**이다.
    #
    # ⚠️ baseline_constants.GRIPPER_EMPTY_POSITION_RAW(1040) 는 하한 1007
    # 에서 잰 값이라 지금 하한과 안 맞는다. 파지 판정을 들어낸 뒤로 그 상수를
    # 읽는 생산 코드가 없어서(가짜 팔과 시험만 쓴다) 지금은 무해하지만,
    # 판정을 되살리면 다시 재야 한다.
    6: (1090, 2090),    # gripper — 2026-09-01 사고가 난 바로 그 레지스터
}


@dataclass(frozen=True)
class FloorGraspProfile:
    """Geometry and initial gripper commands for one object class."""

    object_width_mm: float
    grasp_center_height_mm: float
    preopen_width_mm: float
    close_width_mm: float
    release_width_mm: float


# 2026-08-24: preopen_width_mm을 80.0(임의로 잡았던 절반쯤 열기)에서
# GRIPPER_OPEN_MM(기구적으로 안전하다고 실측된 최대 개구, 168.0)로 올림 —
# 사용자 지시: "무리가 되지 않는 범위 내에서 최대로" 열 것. GRIPPER_OPEN_MM
# 자체가 이미 gripper_calibration.py의 안전 clamp 상한이라 별도 여유값을
# 더 두지 않는다.
#
# 파지력은 **목표 폭을 물체 폭보다 얼마나 더 좁게 명령하느냐**로만 조절된다.
# servo 6에는 토크 제한 레지스터가 없다 — driver_sdk가 노출하는 것은
# set_torque(on/off)뿐이라, "더 세게"는 위치 오차를 키워 정지 토크를 키우는
# 것 말고 방법이 없다. 그래서 물체별로 제각각이던 여유(4.0~11.0mm)를
# GRIPPER_SQUEEZE_MM 하나로 통일한다.
#
# ⚠️ 2026-08-24 실기(사용자 보고: "파지할 때 더 세게 잡아야할 것 같아. 너무
# 흔들흔들거려"). 같은 회차 데이터가 이유를 그대로 보여준다 — 놓친 축구공은
# 닫힘 load 0.0860에서 midpoint 0.0430, safe 0.0391(빈손과 같음)로
# 무너졌고, 성공한 축구공은 0.0978 -> 0.0821로 버텼다. 두 경우의 차이는
# 3양자(0.0117)뿐이라 기존 여유는 성공/실패 경계 위에 놓여 있었다.
GRIPPER_SQUEEZE_MM = 15.0

# 투하 시 벌릴 여유 — 물체 폭보다 이만큼만 더 연다.
#
# 2026-08-25 사용자 지시: "물체를 놓을 때 완전히 벌리지 말고 물체가 그리퍼
# 사이에서 나올 정도로만 벌려." 예전에는 preopen_width_mm(=GRIPPER_OPEN_MM,
# 168.0)으로 활짝 열었는데, 손가락 판이 바구니 위로 넓게 쓸릴 뿐 얻는 것이
# 없다. 물체가 턱 사이에서 빠져나오는 데 필요한 것은 물체 폭보다 조금 더
# 벌어지는 것뿐이다.
#
# GRIPPER_SQUEEZE_MM과 같은 15.0을 쓴다 — 닫을 때 폭에서 15 빼고, 놓을 때
# 폭에 15 더한다. 대칭이라 기억하기 쉽고, rook(24.5) 기준 39.5mm로 열려
# 168mm 대비 훨씬 좁다.
#
# ⚠️ 2026-09-04 사용자 지시로 뒤집혔다 — host+Pi 연동 실기에서 물체가
# 그리퍼에서 안 떨어지는 사고가 반복됐다("바구니에 내려놓을 때, 그리퍼
# 최대로 열어. 물체가 그리퍼에서 안 떨어져"). `_release_width()`가 이제
# 이 상수를 안 쓰고 무조건 `GRIPPER_OPEN_MM`까지 연다 — 이 상수는 그
# 이전 결정의 기록으로만 남겨 둔다. domain/task/floor_grasp_policy.py의
# 같은 자리(계층 분리로 복제된 짝)와 반드시 같이 바꿀 것.
GRIPPER_RELEASE_MM = 15.0


def _release_width(object_width_mm: float) -> float:
    """투하 시 여는 폭 — 2026-09-04부터 무조건 기구 최대 개구다.

    예전엔 물체가 턱 사이에서 빠져나올 만큼만(GRIPPER_RELEASE_MM 여유)
    벌렸는데, 실기에서 물체가 계속 걸려 안 떨어져 사용자 지시로 최대로
    여는 쪽으로 바꿨다(GRIPPER_RELEASE_MM 주석 참고). `object_width_mm`
    인자는 더 안 쓰지만, 호출부(FLOOR_GRASP_PROFILES 구성)를 안
    건드리려고 시그니처는 그대로 둔다."""
    return GRIPPER_OPEN_MM


# 2026-09-03 사용자 지시로 box/star(도메인 라벨 기준 — 여기 profile 이름으로는
# cube/star_column)만 예외를 둔다: 부피가 큰 물체라 0.0mm까지 완전히 짓누르지
# 않고 7.0mm(2026-09-02 이전 하한)를 유지한다.
# domain/task/baseline_mission.py의 _CLOSE_WIDTH_OVERRIDE_MM(같은 계층 분리로
# 복제된 짝)과 반드시 같이 맞출 것 — 2026-09-05까지 이 파일만 그 예외를
# 안 받아서, grasp_test_console.py(이 표를 직접 읽는다)가 cube를 실제 미션이
# 쓰는 7.0mm가 아니라 0.0mm로 닫는 어긋남이 실기로 드러났다.
#
# 2026-09-05 실기에서 7.0mm→12.0mm로 시험했으나 둘 다 servo 6 통신 실패가
# 재현됐고, rook 자세로 박스를 쥐게 해봐도 같은 실패가 나 자세와는 무관함이
# 드러났다 — 닫힘 load_ratio(부하)가 실패를 예측했다(servo 6에 토크
# 제한이 없어 목표 폭이 실제 물체 폭보다 한참 좁으면 스톨 부하로 오래
# 버틴다). 물체 실측 폭(cube 40mm/star_column 45mm)에 더 가깝게
# 20.0mm까지 늘려 스톨 부하를 줄여 본다 — baseline_mission.py의 같은
# 값과 함께 바꿨다.
_CLOSE_WIDTH_OVERRIDE_MM = {"cube": 20.0, "star_column": 20.0}


def _close_width(object_width_mm: float, profile: str | None = None) -> float:
    """파지 목표 폭 — 물체 폭과 무관하게 GRIPPER_GRASP_MIN_MM이다(단,
    `_CLOSE_WIDTH_OVERRIDE_MM`에 있는 profile은 그 값).

    **파지 전용** 하한이지 빈 닫힘 폭(GRIPPER_CLOSED_MM)이 아니다. 물체가
    턱을 멈춰 주므로 파지 때는 더 좁게 명령해 위치 오차(=힘)를 키울 수
    있기 때문이다.

    2026-08-25 실측으로 하한을 9.0에서 7.0으로 내렸다(사용자 지시 "최대한
    세게 잡자") — 당시는 얇은 체스말 둘만 이 하한에 걸렸고 나머지 넷은
    (물체폭 - GRIPPER_SQUEEZE_MM)이 하한 위라 그대로였다.

    2026-09-02 사용자 지시(기어 백래시 — 서보 한계까지 밀어붙여야 한다)로
    물체 폭에서 빼는 방식 자체를 버렸다 — 하한(이번에 7.0 -> 0.0)을 모든
    라벨에 직접 쓴다. baseline_mission.py(도메인, 실제 미션이 쓰는 값)와
    같은 정책이다. 2026-09-03에 box/star만 위 이유로 다시 7.0으로 되돌아갔고
    (baseline_mission.py), 이 함수도 그 예외를 받는다."""
    if profile is not None and profile in _CLOSE_WIDTH_OVERRIDE_MM:
        return _CLOSE_WIDTH_OVERRIDE_MM[profile]
    return GRIPPER_GRASP_MIN_MM


# 2026-08-24: 낮은 물체 3종(cube/star_column/soccer_polyhedron)의 파지 중심
# 높이를 20.0 -> 26.0mm로 올림. 아래 HORIZONTAL_GABE_LOW_26_DEG 주석 참고.
FLOOR_GRASP_PROFILES = {
    "cube": FloorGraspProfile(40.0, 26.0, GRIPPER_OPEN_MM, _close_width(40.0, "cube"), _release_width(40.0)),
    "star_column": FloorGraspProfile(45.0, 26.0, GRIPPER_OPEN_MM, _close_width(45.0, "star_column"), _release_width(45.0)),
    "soccer_polyhedron": FloorGraspProfile(46.0, 26.0, GRIPPER_OPEN_MM, _close_width(46.0), _release_width(46.0)),
    "chess_knight": FloorGraspProfile(22.0, 60.0, GRIPPER_OPEN_MM, _close_width(22.0), _release_width(22.0)),
    "chess_rook": FloorGraspProfile(24.5, 45.0, GRIPPER_OPEN_MM, _close_width(24.5), _release_width(24.5)),
    "chess_queen": FloorGraspProfile(17.0, 50.0, GRIPPER_OPEN_MM, _close_width(17.0), _release_width(17.0)),
}

# GRASP 단계의 물체 배치 전제 — 차체 전면에서 물체 **중심**까지, 정면으로.
#
# 2026-08-25 사용자 지시: "GRASP 시 물체의 중심은 모두 19cm 앞(정면)에 있는
# 것을 전제로 하자."
#
# 왜 180이 아니라 190인가: 같은 날 여섯 물체를 전부 차체 전면 180mm에 놓고
# 돌렸는데, star_column이 **내려오는 그리퍼 위로 올라탔다**(사용자 관찰).
# cube/star/soccer가 쓰는 GABE 저자세는 접근축이 6.49도 아래를 향해 손가락
# 판이 파지 중심보다 앞·아래로 뻗는다 — 180mm에서는 그 판이 낮은 물체를
# 감싸는 대신 그 위에 내려앉는다. 10mm가 그 여유를 만든다.
#
# ⚠️ 이 값은 depth 카메라가 보고하는 전방 거리와 **같지 않다**. 같은 날
# 물리적으로 같은 180mm에 놓인 물체들이 카메라 기준 14.4(queen) /
# 18.3(rook) / 18.7(knight) / 25.6cm(soccer)로 읽혔다 — 클래스별 K_CLASS
# 보정값에 실제 오차가 있어서, 카메라 숫자로 배치를 확인할 수 없다.
GRASP_OBJECT_CENTER_FORWARD_MM = 190.0

# The smallest successful settled load measured while holding an object was
# 0.0704.  Keep the existing domain threshold lower than that value; load alone
# is not sufficient validation, so hardware tests also require lift and hold.
MEASURED_CUBE_HOLD_LOAD_RATIO = 0.0704
MIN_GRIPPER_CLEARANCE_MM = 140.0

# Servo 1..5 angles in degrees.  These poses are specific to the measured arm
# mounting and floor.  Revalidate them after changing the arm or base mounting.
# 2026-08-20 재실측: raw (2029, 2492, 2513, 1133, 3007)에서 실제 파지
# 중심 높이 145 mm, 차체 전면 기준 전방 185 mm, 중심선 기준 좌측 20 mm.
# 최소 140 mm 계약에 측정 여유 5 mm를 둔다.
HORIZONTAL_SAFE_145_DEG = (-1.67, 39.02, 40.87, -80.42, 84.29)
HORIZONTAL_SAFE_145_RAW = (2029, 2492, 2513, 1133, 3007)
# 2026-08-20 빈손 실측: 중심 높이 195 mm, 테두리 위 약 80 mm,
# 차체 전면 기준 전방 200 mm. SAFE_145와 같은 수평 손가락 방향을 유지했다.
#
# ⚠️ 2026-09-04 사용자 지시로 300mm로 올림: "그리퍼 사이의 물체가 바구니에
# 안 닿을 것 같다"는 우려로, 195mm에서 시작해 230/250/270/300mm를 차례로
# FK 계산(tools/drop_pose_height_test.py) 후 실기(빈손 IDLE↔DROP 왕복,
# 무간섭)로 확인하며 올렸다 — 마지막 300mm에서 "충분한 것 같다"는 결론.
# 차체 전면 기준 전방거리는 200mm로 그대로다(target_x_mm을 옛 195mm 자세와
# 같게 유지한 채 servo2·3·4만 다시 풀었다) — 손가락도 계속 수평(pitch
# 동일)이다.
#
# servo1도 2029 -> 2066으로 바뀌었다: 옛 값(2029)은 IDLE(2066)과 -37raw
# (-3.25°) 차이가 있어 DROP 진입·복귀마다 base가 살짝 돌아갔다(2026-09-04
# 실기로 확인). 사용자 지시("웬만하면 그냥 고정해줘")로 IDLE의 servo1을
# 그대로 물려써 그 회전을 없앴다.
#
# 옛 195mm 값 — 필요하면 여기서 되살릴 것: (2029, 2192, 2601, 1345, 3007).
BASKET_DROP_300_RAW = (2066, 1835, 2436, 1867, 3007)
HORIZONTAL_CHESS_MID_40_DEG = (-1.67, 96.57, -9.79, -87.29, 84.30)

# ⚠️ 2026-08-24 폐기. 사용자 보고: "큐브랑 축구공은 파지 높이를 맞추기 위해서
# 내려와 로봇암이 바닥에 약간 닿아."
#
# so101.urdf FK로 확인한 원인(계산은 tests/test_floor_grasp_profiles.py의
# 기하 검사에 그대로 들어 있다):
#
#   - base_link 원점은 바닥에서 98mm 위다. 이 상수는 추정이 아니라 실측된
#     네 자세(SAFE_145 / ROOK_45 / QUEEN_50 / KNIGHT_60)의 문서화된 파지
#     중심 높이와 FK z가 전부 정확히 98mm 차이라는 데서 나온다.
#   - 체스 자세 셋은 접근축(툴 local z)이 모두 수평(+0.51도)인데, 이
#     GABE 자세만 **8.66도 아래를 향한다**. 손가락 판이 파지 중심보다
#     앞·아래로 뻗어 있으므로 이 기울기가 판 끝을 바닥 아래로 밀어넣는다.
#
# 그런데 이 기울기는 잘못 가르친 게 아니라 **불가피하다**: 접근축을 수평으로
# 둔 채 파지 중심을 20mm까지 내리려면 servo2가 107~112도여야 하는데
# shoulder_lift의 URDF 한계는 ±100도다. 즉 팔은 이 높이에서 손가락을 수평으로
# 만들 수 없고, 아래로 기울이는 것이 20mm에 닿는 유일한 방법이었다.
#
# 그래서 기울기를 없애는 대신 **파지 중심을 6mm 올린다**. servo4만 -68.88 ->
# -71.05로 2.17도 움직이는 한 관절 변경이고, 나머지 넷은 그대로다. 결과:
# 파지 중심 20.0 -> 26.0mm, 접근축 -8.66 -> -6.49도, 전방 도달 370.0 ->
# 370.8mm(0.8mm — 물체 배치 위치는 사실상 그대로). 두 효과가 겹쳐 손가락 판
# 최저점이 약 7mm 올라간다.
#
# 파지 자체는 위태롭지 않다 — 이 자세를 쓰는 물체는 폭 40/45/46mm라 실제
# 중심이 20.0/22.5/23.0mm이고, 26mm는 그보다 3~6mm 위일 뿐 손가락 판이
# 물체를 충분히 감싼다. 오히려 star/soccer는 기존 20mm가 중심보다 낮았다.
HORIZONTAL_GABE_LOW_26_DEG = (-1.39, 95.70, -18.16, -71.05, 84.18)
HORIZONTAL_CHESS_ROOK_45_DEG = (-1.67, 93.87, -6.32, -88.06, 84.30)
HORIZONTAL_CHESS_QUEEN_50_DEG = (-1.67, 91.23, -3.04, -88.70, 84.30)
HORIZONTAL_CHESS_KNIGHT_60_DEG = (-1.67, 86.10, 3.06, -89.67, 84.30)

# 2026-08-20 실측 저부하 빈손 이동 자세. servo 1..5 raw를 그대로 보존한다.
# torque를 현재 위치에 latch한 뒤 관절 load가 모두 0인 것을 확인했다.
#
# 2026-08-24: servo 1-5 전체를 reteach_idle_pose.py로 손으로 다시 잡음 —
# torque 해제 후 팔 전체를 원하는 IDLE 자세로 재포즈(그리퍼 정면 정렬 포함).
IDLE_CRADLE_RAW = (2066, 829, 3092, 2751, 3071)

# 물체를 **든 채 주행할 때만** 쓰는 자세. IDLE에서 servo 4(손목)만 들어올린다.
#
# 왜 IDLE을 그냥 안 고치는가: IDLE은 빈손 복귀·시작·정렬이 함께 쓰는 자세이고,
# 그 자세는 관절 부하가 전부 0인 크래들 안착 상태다(위 주석). 손목을 올리면
# 주행 내내 servo 4가 무게를 버텨야 하므로, 그 대가를 물체를 든 구간에만
# 치르게 한다.
#
# 왜 필요한가 (2026-08-26 실측): 나이트를 문 채 IDLE에 있으면 그리퍼와 물체가
# 라이다 정면을 통째로 가린다 — 정면 ±30도 79점 중 58점(79%)이 4.5~6.8cm로
# 막혔고, 막힌 방위가 -19~+23도로 바구니 탐지에 쓸 구간과 정확히 겹쳤다.
# servo 4를 2751 -> 2514(-237 raw, -20.8도)로 올리자 가림이 0%가 되고
# 최근접이 4.5cm에서 71.2cm로 열렸다. 손으로 재포즈해 잡은 값이다.
#
# ⚠️ 여유는 아직 모른다. 막힘(-92)과 열림(-237) 사이 어디에 경계가 있는지
# 재지 않았다 — 주행 진동으로 손목이 처지면 다시 막힐 수 있다.
# ⚠️ depth 카메라 시야는 아직 확인 안 했다. confirm_grasp()가 "CARRY에서 팔이
# 프레임 밖"을 전제하는데 20.8도 올린 뒤에도 그런지 봐야 한다.
#
# ── 2026-09-07: servo 4 를 2514 -> 2217 로 더 올린다 ──────────────────────
#
# ⚠️ 2026-09-08 정정 — **아래 동기는 원인 오진이었을 가능성이 크다.**
#
# 그날 사용자가 "파지 후 주행 자세에서 자꾸 기물을 떨어뜨린다"고 했고, 그래서
# 피치를 맞췄다. 그런데 다음 날 사용자가 정정했다 — "VLA 기반에서는 물체를
# 딱히 떨어뜨린 적이 없다. 문제는 파지했어도 파지하지 않았다고 판단하는
# 시퀀스였다."
#
# 그 말이 맞을 근거가 코드에 있었다. 판정 직전에 "확실히 닫아라"를 보내던
# 단계가 오히려 쥐는 힘을 **풀고 있었다** — 위치제어의 힘은
# P x (목표 - 현재) 인데, 그 명령이 정책이 잡아 둔 자리보다 넓었다:
#
#     정책이 명령한 자리        1007      오차 183 raw
#     판정용 닫기가 명령한 자리  1106      오차  84 raw   <- 힘이 절반
#
# 그리고 그 직후가 주행이었다. 그 시퀀스는 2026-09-08 에 통째로 사라졌다
# (baseline_mission 의 "파지와 CARRY 사이에 그리퍼 명령을 내지 않는다").
#
# **그래도 2217 은 그대로 둔다.** 이 값을 지지하는 근거가 따로 있고, 그쪽은
# 실측이다 — 위 08-26 의 라이다 가림(79% -> 0%)이고 2217 은 같은 방향으로 더
# 간다. 수평 피치는 손해 볼 것이 없으니 덤으로 남긴다. 즉 **이 값의 이유는
# "떨어져서"가 아니라 "라이다 시야"다.**
#
# 아래는 그때의 유도 과정이라 기록으로 남긴다.
#
# 그리퍼 피치는 lift + elbow + wrist 로 나온다. 위 교시 자세들이 그 규칙을
# 그대로 보여준다 — 수평 파지 셋이 전부 -0.51도다:
#
#     queen   91.23 + (-3.04) + (-88.70) = -0.51
#     rook    93.87 + (-6.32) + (-88.06) = -0.51
#     knight  86.10 +   3.06  + (-89.67) = -0.51
#     gabe    95.70 + (-18.16) + (-71.05) = +6.49   (낮은 물체 셋)
#
# 그런데 CARRY(2514)의 피치는
#
#     lift -107.16 + elbow 91.78 + wrist 40.97 = **+25.59도**
#
# 즉 물체를 문 순간(-0.5도)에서 **26도 들린 채** 주행한다. 턱 사이에서
# 물체를 붙잡는 것은 마찰뿐인데, 26도 기울이면 중력이 턱 면을 따라 미끄러지는
# 성분을 갖는다 — 그게 주행 중에 떨어지는 이유다.
#
# 그래서 CARRY 의 피치를 **파지 때와 같게** 맞춘다. 물체가 그리퍼에 대해
# 같은 자세로 남으므로 마찰 균형이 파지 순간 그대로다.
#
#     목표 피치 -0.51 -> wrist = -0.51 - (-107.16 + 91.78) = +14.87도
#                              -> raw 2048 + 14.87/360*4095 = 2217
#
# 낮은 물체 셋(+6.49도)은 7도 어긋나지만, 지금의 19도보다 훨씬 낫고 방향도
# 코가 **내려가는** 쪽이라 더 안전하다. 프로파일별로 나누지 않는 이유는
# CARRY 가 한 자세로 여러 곳(near_pose 판정 포함)에서 쓰이기 때문이다.
#
# ⚠️ 라이다 가림은 더 좋아지는 방향이다 — 2751 -> 2514 로 내려서 가림이
# 0%가 됐고, 2217 은 같은 방향으로 297 raw 더 간다. (09-08 정정 이후로는
# 이것이 **주된** 근거다.)
#
# ⚠️ 라이다를 "안 쓴다"고 오해하기 쉬운데, 꺼진 것은 투하 **판정** 관문뿐이다
# (LIDAR_INSERT_CHECK_ENABLED=False). CARRY 는 여전히 매 사이클 basket_face()
# 를 부르고, 상자 접근 중 retreat_if_too_close / within_stop_window 로 충돌을
# 막는다 — 그 둘은 저 스위치의 영향을 안 받는다. 탑뷰 추측항법이 틀렸을 때
# 바구니를 들이받지 않게 하는 유일한 장치다.
# ⚠️ 이 값은 **계산이지 실측이 아니다.** servo 4 가동범위(870~3224) 안이고
# 부호도 맞지만, 첫 운반은 눈으로 보면서 돌릴 것.
# ── 2026-09-08: IDLE 에서 **파생**시킨다 ─────────────────────────────────
#
# 사용자: "주행자세도 vla 시작자세와 통일시켜줄 수 있을까?"
#
# 물리적으로는 통일할 수 없다. 두 자세가 하는 일이 다르다:
#
#     IDLE   정책의 학습 분포다. 118회 시범의 첫 관측이 전부 이 자세라
#            여기를 옮기면 정책이 본 적 없는 자세에서 시작한다.
#     CARRY  물체를 문 각도(-0.51도)를 유지한다. IDLE 로 맞추면 피치가
#            +46.4도가 되어 위 09-07 기록의 +25.6도보다 더 심하게 흘린다.
#
# 하지만 **관계는 코드로 드러낼 수 있다.** 실제로 다른 관절은 손목 하나뿐인데
# 지금까지 같은 값 네 개를 따로 적어 두었다 — IDLE 을 재교시하면(2026-08-24 에
# 실제로 손으로 다시 잡았다) CARRY 가 조용히 어긋나는 구조였다. 손목만 갈아
# 끼우는 형태로 바꿔서 그 어긋남 자체를 없앤다.
CARRY_WRIST_RAW = 2217
#
# ⚠️ 여기에 `assert CARRY_RAW == (...)` 를 두지 않는다. IDLE 을 재교시하면
# CARRY 가 따라오는 것이 **의도**인데, 그 단언은 정확히 그때 터져서
# arm_driver 가 import 단계에서 죽는다. 값이 맞는지는 시험이 본다
# (tests/test_arm_hardware_contract.py 의 CARRY 파생 시험).
CARRY_RAW = IDLE_CRADLE_RAW[:3] + (CARRY_WRIST_RAW,) + IDLE_CRADLE_RAW[4:]

# IDLE_CRADLE과 수평 자세 사이에서 차체 접촉 없이 검증한 중간 waypoint.
VERTICAL_SAFE_OVERHEAD_DEG = (0.0, 9.2, 20.8, 55.3, 0.4)
HORIZONTAL_OVERHEAD_RAW = (2044, 2712, 2380, 1000, 3006)

HORIZONTAL_GRASP_POSES_DEG = {
    "cube": HORIZONTAL_GABE_LOW_26_DEG,
    "star_column": HORIZONTAL_GABE_LOW_26_DEG,
    "soccer_polyhedron": HORIZONTAL_GABE_LOW_26_DEG,
    "chess_rook": HORIZONTAL_CHESS_ROOK_45_DEG,
    "chess_queen": HORIZONTAL_CHESS_QUEEN_50_DEG,
    "chess_knight": HORIZONTAL_CHESS_KNIGHT_60_DEG,
}
