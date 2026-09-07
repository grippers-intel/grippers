"""Contracts for measured and proposed SO-ARM101 floor-grasp profiles."""

import importlib.util
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
GRIPPERS_ARM_SRC = ROOT / "ros2_ws" / "src" / "grippers_arm"
PROFILE_MODULE = GRIPPERS_ARM_SRC / "grippers_arm" / "floor_grasp_profiles.py"

# floor_grasp_profiles.py가 `from grippers_arm.gripper_calibration import ...`로
# 절대 import하므로, 단독 로드 전에 grippers_arm의 부모 디렉터리를 sys.path에
# 얹어야 한다 — tests/test_align_to_idle.py와 같은 이유·같은 방식.
if str(GRIPPERS_ARM_SRC) not in sys.path:
    sys.path.insert(0, str(GRIPPERS_ARM_SRC))


def _load_profiles():
    spec = importlib.util.spec_from_file_location("floor_grasp_profiles", PROFILE_MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_floor_grasp_profiles_match_measured_object_geometry():
    module = _load_profiles()
    profiles = module.FLOOR_GRASP_PROFILES

    assert (profiles["cube"].object_width_mm, profiles["cube"].grasp_center_height_mm) == (
        40.0,
        26.0,
    )
    assert profiles["star_column"].object_width_mm == 45.0
    assert profiles["soccer_polyhedron"].object_width_mm == 46.0
    # 2026-09-02: 물체 폭과 무관하게 여섯 전부 파지 전용 하한(GRIPPER_
    # GRASP_MIN_MM)을 직접 쓴다(기어 백래시 — 서보 한계까지 밀어붙여야
    # 한다는 사용자 지시). 2026-08-25에는 얇은 체스말 둘만 이 하한에
    # 걸렸었다. 2026-09-03에 box/star(여기 profile 이름으로는 cube/
    # star_column)만 예외가 다시 생겼다 — 아래 test_부피가_큰_물체는_
    # 완전히_짓누르지_않는다 참고.
    bottoms_out = {name: p for name, p in profiles.items()
                   if name not in ("cube", "star_column")}
    assert all(p.close_width_mm == module.GRIPPER_GRASP_MIN_MM
              for p in bottoms_out.values())
    assert all(profile.preopen_width_mm == 168.0 for profile in profiles.values())


def test_부피가_큰_물체는_완전히_짓누르지_않는다():
    """2026-09-03 사용자 지시로 cube/star_column(도메인 라벨 box/star)만
    예외를 둔다 — 부피가 커서 0.0mm(서보 한계)까지 밀어붙이지 않고
    2026-09-02 이전 하한이던 7.0mm를 유지한다.

    domain/task/baseline_mission.py의 _CLOSE_WIDTH_OVERRIDE_MM(같은
    계층 분리로 복제된 짝, test_baseline_mission.py의
    test_부피가_큰_box_star는_완전히_짓누르지_않는다과 대칭)과 반드시
    같이 맞출 것 — 2026-09-05, 이 파일만 그 예외를 안 받아서
    grasp_test_console.py가 cube를 0.0mm로(도메인이 실제로 쓰는 7.0mm가
    아니라) 닫는 어긋남이 실기로 드러났다.

    2026-09-05 실기에서 7.0mm·12.0mm 둘 다 servo 6 통신 실패가 재현됐고,
    닫힘 load_ratio(부하)가 자세와 무관하게 실패를 예측한다는 게
    드러났다 — 스톨 부하를 줄이려 물체 실측 폭에 더 가까운 20.0mm로
    늘렸다(사용자 지시 — "안 잡힐 수도 있지만")."""
    module = _load_profiles()
    profiles = module.FLOOR_GRASP_PROFILES

    assert profiles["cube"].close_width_mm == 20.0
    assert profiles["star_column"].close_width_mm == 20.0
    # soccer_polyhedron은 언급되지 않아 여전히 GRIPPER_GRASP_MIN_MM이다.
    assert profiles["soccer_polyhedron"].close_width_mm == module.GRIPPER_GRASP_MIN_MM


def test_every_profile_squeezes_by_the_same_margin_unless_the_jaw_bottoms_out():
    """파지력을 키우는 유일한 수단이 위치 오차이므로(servo 6에는 토크 제한
    레지스터가 없다), 여유는 물체마다 손으로 고른 값이 아니라 한 상수여야
    한다 — 사용자 보고 "너무 흔들흔들거려"(2026-08-24)."""
    module = _load_profiles()

    for name, profile in module.FLOOR_GRASP_PROFILES.items():
        if name in ("cube", "star_column"):
            continue  # 2026-09-03 예외(7.0mm 하한) — 위 test_부피가_큰_물체는_완전히_짓누르지_않는다 참고
        squeeze = profile.object_width_mm - profile.close_width_mm
        bottomed_out = profile.close_width_mm == module.GRIPPER_GRASP_MIN_MM
        assert bottomed_out or squeeze == module.GRIPPER_SQUEEZE_MM, name


def test_the_thin_chess_pieces_are_the_ones_that_bottom_out():
    """2026-09-02까지는 queen(17.0mm)·knight(22.0mm)만 파지 전용 하한에
    걸렸다 — 나머지 넷은 (물체폭 - GRIPPER_SQUEEZE_MM)이 하한 위였다.

    2026-09-02 사용자 지시(기어 백래시 — 서보 한계까지 밀어붙여야 한다)로
    물체 폭에서 빼는 방식 자체를 버리고 모든 라벨이 하한을 직접 쓴다 —
    2026-09-03에 cube/star_column만 다시 예외(7.0mm)가 됐으므로, 지금은
    나머지 넷이 "바닥"이다."""
    module = _load_profiles()
    profiles = module.FLOOR_GRASP_PROFILES

    bottomed = {
        name
        for name, profile in profiles.items()
        if profile.close_width_mm == module.GRIPPER_GRASP_MIN_MM
    }
    assert bottomed == set(profiles) - {"cube", "star_column"}
    assert (
        profiles["chess_knight"].object_width_mm,
        profiles["chess_knight"].grasp_center_height_mm,
    ) == (22.0, 60.0)
    assert (
        profiles["chess_rook"].object_width_mm,
        profiles["chess_rook"].grasp_center_height_mm,
    ) == (24.5, 45.0)
    assert (
        profiles["chess_queen"].object_width_mm,
        profiles["chess_queen"].grasp_center_height_mm,
    ) == (17.0, 50.0)


def test_floor_grasp_commands_are_ordered_and_inside_safe_calibration_range():
    module = _load_profiles()

    for profile in module.FLOOR_GRASP_PROFILES.values():
        assert module.GRIPPER_GRASP_MIN_MM <= profile.close_width_mm < profile.object_width_mm
        assert profile.object_width_mm < profile.preopen_width_mm <= 168.0


def test_hardware_acceptance_contract_records_verified_cube_load():
    module = _load_profiles()

    assert module.MEASURED_CUBE_HOLD_LOAD_RATIO == 0.0704
    assert module.MIN_GRIPPER_CLEARANCE_MM == 140.0


def test_horizontal_arm_poses_keep_gabe_and_chess_heights_separate():
    module = _load_profiles()

    assert module.HORIZONTAL_SAFE_145_DEG == (-1.67, 39.02, 40.87, -80.42, 84.29)
    assert module.HORIZONTAL_SAFE_145_RAW == (2029, 2492, 2513, 1133, 3007)
    assert module.BASKET_DROP_300_RAW == (2066, 1835, 2436, 1867, 3007)
    assert module.HORIZONTAL_CHESS_MID_40_DEG == (-1.67, 96.57, -9.79, -87.29, 84.30)
    assert module.HORIZONTAL_GABE_LOW_26_DEG == (-1.39, 95.70, -18.16, -71.05, 84.18)
    assert module.HORIZONTAL_CHESS_MID_40_DEG != module.HORIZONTAL_GABE_LOW_26_DEG
    # 바닥을 긁던 20mm 자세는 되살아나면 안 된다.
    assert not hasattr(module, "HORIZONTAL_GABE_LOW_20_DEG")


def test_every_object_profile_has_a_horizontal_arm_pose():
    module = _load_profiles()

    assert set(module.HORIZONTAL_GRASP_POSES_DEG) == set(module.FLOOR_GRASP_PROFILES)
    assert module.HORIZONTAL_GRASP_POSES_DEG["chess_rook"] == (
        -1.67,
        93.87,
        -6.32,
        -88.06,
        84.30,
    )
    assert module.HORIZONTAL_GRASP_POSES_DEG["chess_queen"][1] == 91.23
    assert module.HORIZONTAL_GRASP_POSES_DEG["chess_knight"][1] == 86.10


def test_idle_cradle_and_transition_waypoints_match_measured_contract():
    module = _load_profiles()

    assert module.IDLE_CRADLE_RAW == (2066, 829, 3092, 2751, 3071)
    assert module.VERTICAL_SAFE_OVERHEAD_DEG == (0.0, 9.2, 20.8, 55.3, 0.4)
    assert module.HORIZONTAL_OVERHEAD_RAW == (2044, 2712, 2380, 1000, 3006)


def _fk():
    """so101.urdf 순기구학. numpy가 없는 환경에서는 건너뛴다."""
    import pytest

    pytest.importorskip("numpy")
    soarm_lab = ROOT / "third_party" / "soarm_provided_d" / "soarm_lab"
    if not (soarm_lab / "so101.urdf").exists():
        pytest.skip("so101.urdf 없음")
    # soarm_lab/__init__.py는 pyserial까지 끌어오므로 패키지가 아니라 모듈을
    # 직접 얹어 로드한다(같은 디렉터리를 sys.path에 넣는 flat import).
    if str(soarm_lab) not in sys.path:
        sys.path.insert(0, str(soarm_lab))
    from fk_core import FKSo101

    return FKSo101()


def _tip(fk, pose_deg):
    """(파지 중심 높이 mm, 전방 도달 mm, 접근축 pitch deg)."""
    import math

    import numpy as np

    position, rotation = fk.fk_deg(list(pose_deg))
    approach = rotation @ np.array([0.0, 0.0, 1.0])
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, float(approach[2])))))
    return position[2] * 1000.0 + BASE_ABOVE_FLOOR_MM, position[0] * 1000.0, pitch


# base_link 원점의 바닥 위 높이. 아래 테스트가 실측 자세들로부터 이 값을
# 스스로 검증하므로 여기 적힌 숫자는 가정이 아니라 계약이다.
BASE_ABOVE_FLOOR_MM = 98.0


def test_fk_reproduces_the_measured_grasp_heights_from_a_single_base_offset():
    """네 자세의 문서화된 파지 중심 높이가 FK z + 98mm와 전부 일치한다 —
    이게 맞아야 아래 바닥 간섭 계산을 믿을 수 있다."""
    fk = _fk()
    module = _load_profiles()

    measured = {
        145.0: module.HORIZONTAL_SAFE_145_DEG,
        60.0: module.HORIZONTAL_GRASP_POSES_DEG["chess_knight"],
        50.0: module.HORIZONTAL_GRASP_POSES_DEG["chess_queen"],
        45.0: module.HORIZONTAL_GRASP_POSES_DEG["chess_rook"],
    }
    for documented_mm, pose in measured.items():
        height_mm, _, _ = _tip(fk, pose)
        # SAFE_145는 다른 방식으로 실측돼 8mm 어긋난다. 파지 자세 셋은 1mm 안.
        tolerance = 10.0 if documented_mm == 145.0 else 1.0
        assert abs(height_mm - documented_mm) < tolerance, documented_mm


def test_low_pose_lifts_the_finger_plates_clear_of_the_floor():
    """사용자 보고(2026-08-24): cube/soccer에서 팔이 바닥에 약간 닿는다.
    올린 자세는 파지 중심이 6mm 높고 접근축 기울기도 줄어야 한다 — 두 효과가
    모두 손가락 판 최저점을 올린다."""
    fk = _fk()
    module = _load_profiles()

    scraping = (-1.39, 95.70, -18.16, -68.88, 84.18)  # 폐기된 20mm 자세
    old_h, old_x, old_pitch = _tip(fk, scraping)
    new_h, new_x, new_pitch = _tip(fk, module.HORIZONTAL_GABE_LOW_26_DEG)

    assert abs(old_h - 20.0) < 0.5
    assert abs(new_h - 26.0) < 0.5
    assert new_pitch > old_pitch  # 덜 숙인다(둘 다 음수)
    assert abs(new_x - old_x) < 2.0  # 물체 배치 위치는 그대로여야 한다

    for profile_name in ("cube", "star_column", "soccer_polyhedron"):
        profile = module.FLOOR_GRASP_PROFILES[profile_name]
        assert abs(profile.grasp_center_height_mm - new_h) < 0.5, profile_name


def test_a_level_gripper_cannot_reach_the_low_grasp_height():
    """기울기를 없애는 대신 높이를 올린 이유의 근거. 접근축을 체스 자세와
    같은 수평(+0.51도)으로 둔 채 파지 중심 20mm에 닿으려면 shoulder_lift가
    URDF 한계(±100도)를 넘어야 한다."""
    import numpy as np

    fk = _fk()
    module = _load_profiles()
    base_pose = module.HORIZONTAL_GABE_LOW_26_DEG
    _, _, level_pitch = _tip(fk, module.HORIZONTAL_GRASP_POSES_DEG["chess_rook"])

    def residual(joints):
        height, reach, pitch = _tip(fk, (base_pose[0], *joints, base_pose[4]))
        return np.array([height - 20.0, reach - 370.0, pitch - level_pitch])

    joints = np.array(base_pose[1:4], dtype=float)
    for _ in range(200):
        r = residual(joints)
        jacobian = np.zeros((3, 3))
        for i in range(3):
            nudged = joints.copy()
            nudged[i] += 1e-4
            jacobian[:, i] = (residual(nudged) - r) / 1e-4
        joints = joints - np.clip(np.linalg.solve(jacobian, r), -5.0, 5.0)
        if np.abs(residual(joints)).max() < 1e-5:
            break

    assert np.abs(residual(joints)).max() < 1e-3  # 해는 존재한다
    shoulder_lift_limit = fk.limits_deg()["shoulder_lift"][1]
    assert joints[0] > shoulder_lift_limit  # 다만 관절 한계 밖이다


def test_safe_145_degree_and_raw_records_describe_the_same_pose():
    module = _load_profiles()
    converted = tuple(
        round(2048 + degrees * 4096 / 360) for degrees in module.HORIZONTAL_SAFE_145_DEG
    )

    assert converted == module.HORIZONTAL_SAFE_145_RAW


def test_release_width_is_now_the_full_open():
    """투하는 최대로 연다 (사용자 지시, 2026-09-04 — 2026-08-25 결정을 뒤집음).

    2026-08-25엔 "완전히 벌리지 말고 물체가 나올 정도로만"이었지만,
    host+Pi 연동 실기에서 물체가 그리퍼에서 안 떨어지는 사고가 반복돼
    최대 개구(GRIPPER_OPEN_MM)로 되돌렸다."""
    module = _load_profiles()
    for name, profile in module.FLOOR_GRASP_PROFILES.items():
        assert profile.release_width_mm == module.GRIPPER_OPEN_MM, name
        assert profile.release_width_mm > profile.close_width_mm, name


def test_release_width_never_exceeds_the_mechanical_limit():
    """폭이 아주 넓은 물체가 생겨도 기구 상한을 넘지 않는다(지금은 그
    상한 자체가 목표값이다)."""
    module = _load_profiles()
    assert module._release_width(1000.0) == module.GRIPPER_OPEN_MM


def test_every_label_now_uses_the_grasp_floor_directly():
    """사용자 지시(2026-09-02, 기어 백래시 — 서보 한계까지 밀어붙여야 한다)의
    실제 결과 — 2026-08-25에는 하한에 걸려 있던 queen/knight 둘만 바뀌고
    나머지 넷(rook/cube/star_column/soccer_polyhedron)은 물체 폭 기반
    공식값을 그대로 썼다. 물체 폭과 무관하게 하한을 직접 쓰되, 2026-09-03
    예외(cube/star_column, 위 test_부피가_큰_물체는_완전히_짓누르지_않는다
    참고)는 그 하한이 아니라 20.0mm다(2026-09-05, 스톨 부하 완화)."""
    profiles = _load_profiles()
    floor = profiles.GRIPPER_GRASP_MIN_MM

    for name in profiles.FLOOR_GRASP_PROFILES:
        if name in ("cube", "star_column"):
            assert profiles.FLOOR_GRASP_PROFILES[name].close_width_mm == 20.0
        else:
            assert profiles.FLOOR_GRASP_PROFILES[name].close_width_mm == floor


def test_close_width_clamps_at_the_grasp_floor_not_the_empty_closed_width():
    """빈 닫힘 하한으로 clamp하면 파지가 쓸 수 있는 힘을 버린다 — 두 하한을
    나눈 이유가 그것이다."""
    profiles = _load_profiles()

    assert profiles._close_width(10.0) == profiles.GRIPPER_GRASP_MIN_MM
    assert profiles.GRIPPER_GRASP_MIN_MM < 9.0


# ── 운반 자세는 파지 때 그리퍼 각도를 유지한다 (2026-09-07) ───────────────


def _pitch_deg(lift_raw, elbow_raw, wrist_raw):
    """그리퍼 피치 = lift + elbow + wrist. driver_sdk 의 환산과 같다
    (POS_CENTER 2048, 4095 카운트 = 360도)."""
    to_deg = lambda raw: (raw - 2048) / 4095.0 * 360.0
    return to_deg(lift_raw) + to_deg(elbow_raw) + to_deg(wrist_raw)


def test_교시_수평_파지는_전부_같은_피치다():
    """이 규칙(lift+elbow+wrist)이 성립해야 아래 시험이 뜻을 갖는다."""
    module = _load_profiles()
    for name in ("HORIZONTAL_CHESS_QUEEN_50_DEG", "HORIZONTAL_CHESS_ROOK_45_DEG",
                 "HORIZONTAL_CHESS_KNIGHT_60_DEG"):
        _pan, lift, elbow, wrist, _roll = getattr(module, name)
        assert abs((lift + elbow + wrist) - (-0.51)) < 0.05, name


def test_운반_자세_피치가_파지_피치와_같다():
    """⚠️ 2026-09-07 사용자 보고: "기본주행 자세에서 그리퍼의 각도가 달라서
    자꾸 기물을 떨어뜨리고 있어."

    턱 사이에서 물체를 붙잡는 것은 마찰뿐이라, 파지 때와 각도가 달라지면
    중력이 턱 면을 따라 미끄러지는 성분을 갖는다. 예전 CARRY(servo4 2514)는
    파지보다 26도 들려 있었다.

    ⚠️ 2026-09-08 정정: 이 시험이 지키는 것은 맞지만 **동기는 오진이었다.**
    사용자가 "VLA 기반에서는 물체를 딱히 떨어뜨린 적이 없다 — 문제는 파지했어도
    파지하지 않았다고 판단하는 시퀀스였다"고 정정했고, 실제로 그 판정용 닫기가
    쥐는 힘을 절반으로 풀고 있었다(baseline_mission 의 같은 주석).

    값을 그대로 두는 이유는 **라이다 시야**다 — 그쪽은 실측이 있다. 아래
    숫자는 IDLE 로 통일하면 피치가 어떻게 되는지의 기록이다:

        IDLE   2751 -> +46.4도
        옛 값  2514 -> +25.6도    <- 이때 계속 떨어뜨렸다
        지금   2217 ->  -0.5도    <- 문 순간 그대로

    라이다도 같은 방향이다(IDLE 에서 나이트를 물면 정면 ±30도 79점 중 58점
    막힘, 2026-08-26 실측). 두 자세는 하는 일이 다르므로 통일하지 않는다 —
    관계만 코드로 드러냈다(test_carry_는_idle_에서_손목만_바꾼_자세다)."""
    module = _load_profiles()
    _pan, lift, elbow, wrist, _roll = module.CARRY_RAW

    carry_pitch = _pitch_deg(lift, elbow, wrist)
    grasp_pitch = -0.51        # 교시 수평 파지 셋의 공통 피치(위 시험)

    assert abs(carry_pitch - grasp_pitch) < 1.5, (
        f"운반 피치 {carry_pitch:+.1f}도가 파지 {grasp_pitch:+.1f}도와 다르다 "
        f"— 물체가 미끄러진다")


def test_운반_손목은_IDLE보다_더_들려_있다():
    """라이다 가림 때문에 올린 것이라(CARRY_RAW 주석) 방향이 뒤집히면 안 된다.
    servo 4 는 raw 가 작아지는 쪽이 '들리는' 쪽이다."""
    module = _load_profiles()

    assert module.CARRY_RAW[3] < module.IDLE_CRADLE_RAW[3]
    # 가동범위 안이어야 한다.
    low, high = module.TAUGHT_POSITION_LIMITS[4]
    assert low < module.CARRY_RAW[3] < high


def test_carry_는_idle_에서_손목만_바꾼_자세다():
    """⚠️ 2026-09-08: 두 자세를 **통일하지 말 것.** 하는 일이 다르다.

    사용자가 "주행자세도 vla 시작자세와 통일시켜줄 수 있을까?" 라고 물었고,
    재 보니 통일할 수 없다는 답이 나왔다. 다만 **다른 관절은 손목 하나뿐**이라
    그 관계를 코드로 드러냈다 — 예전에는 같은 값 네 개를 따로 적어 두어서,
    IDLE 을 재교시하면(2026-08-24 에 실제로 손으로 다시 잡았다) CARRY 가
    조용히 어긋났다.

    이 시험이 지키는 것은 "손목 말고는 늘 IDLE 을 따라간다"이다."""
    m = _load_profiles()
    idle, carry = m.IDLE_CRADLE_RAW, m.CARRY_RAW

    assert len(carry) == len(idle) == 5
    差 = [i for i, (a, b) in enumerate(zip(idle, carry)) if a != b]
    assert 差 == [3], f"손목(index 3) 말고 다른 관절이 달라졌다: {差}"
    assert carry[3] == m.CARRY_WRIST_RAW
