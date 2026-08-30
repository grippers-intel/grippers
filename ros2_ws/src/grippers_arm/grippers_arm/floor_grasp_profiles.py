"""Measured floor-grasp targets for the current SO-ARM101 end effector.

Object geometry is kept separate from the named, hardware-tested arm poses so
that a low GABE pose is not accidentally reused for taller chess pieces.
"""
# ═══════════════════════════════════════════════════════════════════════════
# 2026-08-30 — 아래 자세들은 **새 영점 기준으로 옮겨졌다.** (2026-08-29 경고 해소)
#
# 8/29 VLA 시연 준비로 LeRobot 캘리브레이션을 다시 돌리면서 서보의
# Homing_Offset 이 덮여 써졌다(lerobot/motors/feetech/feetech.py:275).
#
#     Present_Position = Actual_Position - Homing_Offset
#
# 그래서 같은 RAW 가 다른 물리 자세를 가리키게 됐는데, **캘리브레이션 직전에
# EEPROM 을 백업해 둔 덕에 다시 교시하지 않고 계산으로 옮길 수 있었다.**
#
#     관절별 영점 이동(새-옛)   pan +28  lift +5  elbow +31  wrist_flex +34
#                                wrist_roll +976   (gripper -953)
#     저장된 RAW 에서 이만큼 빼면 새 기준이 된다.
#
# ⚠️ _DEG 값도 같이 옮겼다. degrees_to_position() 이 POS_CENTER(2048) 고정
#    기준이라 각도 표기도 영점 이동의 영향을 그대로 받는다.
#
# 검증
#   * 변환식을 크래들 자세로 대조 — pan 0 / elbow -1 / lift +4 카운트 오차.
#     기구가 붙잡는 세 관절이 ±4 안에서 맞았다.
#   * 12개 자세 전부 현재 서보 위치 한계 안에 있음을 확인.
#
# 직접 실측한 것 (계산이 아님)
#   IDLE_CRADLE_RAW — tools/arm/reteach_idle_win.py 로 재교시.
#     확정 후 6초간 오차 0 / 부하 0 — 크래들이 받쳐 주고 있다.
#   elbow_flex Max_Position_Limit 을 3041 -> 3070 으로 넓혔다(EEPROM +
#     캘리브레이션 파일 양쪽). 8/29 캘리브레이션이 팔꿈치를 크래들만큼 깊이
#     접지 않아 한계가 실제보다 좁게 잡혔고, 그대로 두면 IDLE 명령이 잘렸다.
#
# 🔴 아직 실기로 확인 안 된 것 — CARRY_RAW
#   새 IDLE 은 손목이 옛것보다 142 카운트(12.5도) 다르다. 확인할 것이 둘인데
#   **중요도가 다르다.**
#
#   ① 뎁스캠 시야 — 🔴 **이쪽이 먼저다**
#      confirm_grasp() 가 "CARRY 에서 팔이 뎁스캠 프레임 밖"을 전제한다
#      (domain/ports/perception.py:64, 2026-08-25 실기 확인). 파지 성공을
#      판정하는 **독립적인 두 신호 중 하나**가 여기 걸려 있다 — 물체가 있던
#      자리를 다시 봐서 사라졌으면 집힌 것이다. 팔이 프레임에 들어오면 이
#      판정이 통째로 죽고, 그러면 GRASP_DONE/GRASP_FAILED 가 안 나가 Host 의
#      실패 경로 전체가 멈춘다.
#      ⚠️ VLA 로 파지를 옮겨도 **그대로 남는다.** 오히려 더 중요해진다 —
#         정책은 자기가 성공했는지 모른다.
#      원래 8/26 손목을 20.8도 올릴 때부터 미확인이던 항목이다.
#      재는 법: 물체를 든 채 CARRY 자세로 두고 뎁스캠 프레임 한 장 —
#              팔이 안 보이고 바닥이 보이면 통과.
#
#   ② 라이다 정면 가림 — 🟠 낮음
#      -237 은 라이다 가림을 79% -> 0% 로 만든 실측값이다. 다만 라이다는
#      **바구니 정면 판정에만** 쓰이고(baseline_ports.py:165), 그 판정을
#      탑뷰로 옮기는 안이 검토 중이다. 옮기면 이 확인은 불필요해진다.
#
# 되돌리려면:
#     python tools/arm/backup_servo_offsets.py COM8 #            --restore tools/arm/servo_backup/servo_COM8_20260830_141033.json
# ═══════════════════════════════════════════════════════════════════════════


from dataclasses import dataclass

# 절대 import를 쓴다 — align_to_idle.py 등 tools/*.py의 grippers_arm 참조와
# 같은 방식이다. 이 파일은 tests/test_floor_grasp_profiles.py에서
# importlib.util.spec_from_file_location으로 단독 로드되기도 하는데, 그
# 경로에선 패키지 컨텍스트가 없어 상대 import(`from .gripper_calibration`)가
# "attempted relative import with no known parent package"로 깨진다.
from grippers_arm.gripper_calibration import GRIPPER_GRASP_MIN_MM, GRIPPER_OPEN_MM


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


# 2026-08-30: 파지 직전 개구 폭. GRIPPER_OPEN_MM(168, 기구 상한)에서 내렸다.
#
# 이유 — **168mm 로 열면 턱이 손목캠 프레임 밖으로 나간다.** 실측으로 확인했다:
#   40mm 양쪽 턱 또렷 / 65mm 잘 보임 / 80mm 화면 가장자리에 걸침 / 120·168mm 안 보임
#
# 손목캠의 존재 이유가 **턱과 물체의 mm 관계**를 담는 것인데, 턱이 프레임 밖이면
# 그 관계가 영상에 아예 없다. 사람이 시연할 때 조준을 못 하고, VLA 정책도
# 같은 이유로 못 배운다.
#
# 65 인 이유: 투하 폭 최대값(soccer 61mm)을 덮는 가장 좁은 값이다. 이보다
# 좁히면 soccer 를 놓을 수 없다.
#
# ⚠️ 팔로워 gripper 의 range_max 도 같은 raw(2378)로 맞춰 뒀다. 텔레옵에서
#    **리더 트리거를 끝까지 밀면 정확히 이 폭**이 되게 하기 위해서다 —
#    사람은 중간값을 재현할 수 없고 기구 끝만 재현할 수 있다(사용자 지적).
#    그래서 시연의 "완전 개방"과 미션의 preopen 이 정의상 같아진다.
#
# 8/24 에 80.0 -> 168.0 으로 올렸던 것을 되돌리는 셈이다. 그때 닫힘 행정이
# 319 -> 820 raw 로 2.6배 길어져 GRASP_SETTLE_SEC 이 모자라는 사고가 났는데,
# 좁히면 그 위험도 같이 줄어든다.
GRIPPER_PREOPEN_MM = 65.0

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
GRIPPER_RELEASE_MM = 15.0


def _release_width(object_width_mm: float) -> float:
    """물체가 턱 사이에서 빠져나올 만큼만 벌린 목표 폭.

    기구 상한(GRIPPER_OPEN_MM)을 넘지 않는다. 넓은 물체
    (soccer_polyhedron 46.0 -> 61.0)도 상한에 한참 못 미친다."""
    return min(GRIPPER_OPEN_MM, round(object_width_mm + GRIPPER_RELEASE_MM, 1))


def _close_width(object_width_mm: float) -> float:
    """물체 폭에서 GRIPPER_SQUEEZE_MM만큼 더 좁힌 목표 폭.

    **파지 전용** 하한(GRIPPER_GRASP_MIN_MM) 아래로는 내려가지 않는다 —
    빈 닫힘 폭(GRIPPER_CLOSED_MM)이 아니다. 물체가 턱을 멈춰 주므로 파지
    때는 더 좁게 명령해 위치 오차(=힘)를 키울 수 있기 때문이다.

    2026-08-25 실측으로 하한을 9.0에서 7.0으로 내렸다(사용자 지시 "최대한
    세게 잡자"). 얇은 체스말 둘이 이 하한에 걸려 있었고, 이제 knight 기준
    파지 부하가 0.0235에서 0.0626으로 2.7배가 된다. 7.0 아래로는 부하가
    포화해 더 얻을 것이 없다 — 근거는 GRIPPER_GRASP_MIN_MM 주석 참고.
    """
    return max(GRIPPER_GRASP_MIN_MM, round(object_width_mm - GRIPPER_SQUEEZE_MM, 1))


# 2026-08-24: 낮은 물체 3종(cube/star_column/soccer_polyhedron)의 파지 중심
# 높이를 20.0 -> 26.0mm로 올림. 아래 HORIZONTAL_GABE_LOW_26_DEG 주석 참고.
FLOOR_GRASP_PROFILES = {
    "cube": FloorGraspProfile(40.0, 26.0, GRIPPER_PREOPEN_MM, _close_width(40.0), _release_width(40.0)),
    "star_column": FloorGraspProfile(45.0, 26.0, GRIPPER_PREOPEN_MM, _close_width(45.0), _release_width(45.0)),
    "soccer_polyhedron": FloorGraspProfile(46.0, 26.0, GRIPPER_PREOPEN_MM, _close_width(46.0), _release_width(46.0)),
    "chess_knight": FloorGraspProfile(22.0, 60.0, GRIPPER_PREOPEN_MM, _close_width(22.0), _release_width(22.0)),
    "chess_rook": FloorGraspProfile(24.5, 45.0, GRIPPER_PREOPEN_MM, _close_width(24.5), _release_width(24.5)),
    "chess_queen": FloorGraspProfile(17.0, 50.0, GRIPPER_PREOPEN_MM, _close_width(17.0), _release_width(17.0)),
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
HORIZONTAL_SAFE_145_DEG = (-4.13, 38.58, 38.14, -83.41, -1.51)
HORIZONTAL_SAFE_145_RAW = (2001, 2487, 2482, 1099, 2031)
# 2026-08-20 빈손 실측: 중심 높이 195 mm, 테두리 위 약 80 mm,
# 차체 전면 기준 전방 200 mm. SAFE_145와 같은 수평 손가락 방향을 유지한다.
BASKET_DROP_195_RAW = (2001, 2187, 2570, 1311, 2031)
HORIZONTAL_CHESS_MID_40_DEG = (-4.13, 96.13, -12.52, -90.28, -1.5)

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
HORIZONTAL_GABE_LOW_26_DEG = (-3.85, 95.26, -20.89, -74.04, -1.62)
HORIZONTAL_CHESS_ROOK_45_DEG = (-4.13, 93.43, -9.05, -91.05, -1.5)
HORIZONTAL_CHESS_QUEEN_50_DEG = (-4.13, 90.79, -5.77, -91.69, -1.5)
HORIZONTAL_CHESS_KNIGHT_60_DEG = (-4.13, 85.66, 0.33, -92.66, -1.5)

# 2026-08-20 실측 저부하 빈손 이동 자세. servo 1..5 raw를 그대로 보존한다.
# torque를 현재 위치에 latch한 뒤 관절 load가 모두 0인 것을 확인했다.
#
# 2026-08-24: servo 1-5 전체를 reteach_idle_pose.py로 손으로 다시 잡음 —
# torque 해제 후 팔 전체를 원하는 IDLE 자세로 재포즈(그리퍼 정면 정렬 포함).
IDLE_CRADLE_RAW = (2038, 828, 3060, 2859, 2001)

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
CARRY_RAW = (2038, 828, 3060, 2480, 2001)

# IDLE_CRADLE과 수평 자세 사이에서 차체 접촉 없이 검증한 중간 waypoint.
VERTICAL_SAFE_OVERHEAD_DEG = (-2.46, 8.76, 18.07, 52.31, -85.4)
HORIZONTAL_OVERHEAD_RAW = (2016, 2707, 2349, 966, 2030)

HORIZONTAL_GRASP_POSES_DEG = {
    "cube": HORIZONTAL_GABE_LOW_26_DEG,
    "star_column": HORIZONTAL_GABE_LOW_26_DEG,
    "soccer_polyhedron": HORIZONTAL_GABE_LOW_26_DEG,
    "chess_rook": HORIZONTAL_CHESS_ROOK_45_DEG,
    "chess_queen": HORIZONTAL_CHESS_QUEEN_50_DEG,
    "chess_knight": HORIZONTAL_CHESS_KNIGHT_60_DEG,
}
