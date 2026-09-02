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
    # 걸렸었다.
    assert all(profile.close_width_mm == module.GRIPPER_GRASP_MIN_MM
              for profile in profiles.values())
    assert all(profile.preopen_width_mm == 168.0 for profile in profiles.values())


def test_every_profile_squeezes_by_the_same_margin_unless_the_jaw_bottoms_out():
    """파지력을 키우는 유일한 수단이 위치 오차이므로(servo 6에는 토크 제한
    레지스터가 없다), 여유는 물체마다 손으로 고른 값이 아니라 한 상수여야
    한다 — 사용자 보고 "너무 흔들흔들거려"(2026-08-24)."""
    module = _load_profiles()

    for name, profile in module.FLOOR_GRASP_PROFILES.items():
        squeeze = profile.object_width_mm - profile.close_width_mm
        bottomed_out = profile.close_width_mm == module.GRIPPER_GRASP_MIN_MM
        assert bottomed_out or squeeze == module.GRIPPER_SQUEEZE_MM, name


def test_the_thin_chess_pieces_are_the_ones_that_bottom_out():
    """2026-09-02까지는 queen(17.0mm)·knight(22.0mm)만 파지 전용 하한에
    걸렸다 — 나머지 넷은 (물체폭 - GRIPPER_SQUEEZE_MM)이 하한 위였다.

    2026-09-02 사용자 지시(기어 백래시 — 서보 한계까지 밀어붙여야 한다)로
    물체 폭에서 빼는 방식 자체를 버리고 모든 라벨이 하한을 직접 쓴다 —
    이제는 여섯 전부가 "바닥"이다."""
    module = _load_profiles()
    profiles = module.FLOOR_GRASP_PROFILES

    bottomed = {
        name
        for name, profile in profiles.items()
        if profile.close_width_mm == module.GRIPPER_GRASP_MIN_MM
    }
    assert bottomed == set(profiles)
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
    assert module.BASKET_DROP_195_RAW == (2029, 2192, 2601, 1345, 3007)
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


def test_release_width_is_object_width_plus_clearance_not_full_open():
    """투하는 활짝 열지 않는다 (사용자 지시, 2026-08-25).

    물체가 턱 사이에서 빠져나오는 데 필요한 것은 물체 폭보다 조금 더 벌어지는
    것뿐이다. GRIPPER_OPEN_MM(168)까지 열면 손가락 판이 바구니 위로 넓게
    쓸릴 뿐 얻는 것이 없다. 닫힘(폭 − 15)과 대칭으로 폭 + 15 를 쓴다.
    """
    module = _load_profiles()
    for name, profile in module.FLOOR_GRASP_PROFILES.items():
        expected = round(profile.object_width_mm + module.GRIPPER_RELEASE_MM, 1)
        assert profile.release_width_mm == expected, name
        # 물체가 빠질 만큼은 벌어져야 하고, 활짝 열어서는 안 된다
        assert profile.release_width_mm > profile.object_width_mm, name
        assert profile.release_width_mm < module.GRIPPER_OPEN_MM, name
        assert profile.release_width_mm > profile.close_width_mm, name


def test_release_width_never_exceeds_the_mechanical_limit():
    """폭이 아주 넓은 물체가 생겨도 기구 상한을 넘지 않는다."""
    module = _load_profiles()
    assert module._release_width(1000.0) == module.GRIPPER_OPEN_MM


def test_every_label_now_uses_the_grasp_floor_directly():
    """사용자 지시(2026-09-02, 기어 백래시 — 서보 한계까지 밀어붙여야 한다)의
    실제 결과 — 2026-08-25에는 하한에 걸려 있던 queen/knight 둘만 바뀌고
    나머지 넷(rook/cube/star_column/soccer_polyhedron)은 물체 폭 기반
    공식값을 그대로 썼다. 이제는 물체 폭과 무관하게 여섯 전부가 하한을
    직접 쓴다."""
    profiles = _load_profiles()
    floor = profiles.GRIPPER_GRASP_MIN_MM

    for name in profiles.FLOOR_GRASP_PROFILES:
        assert profiles.FLOOR_GRASP_PROFILES[name].close_width_mm == floor


def test_close_width_clamps_at_the_grasp_floor_not_the_empty_closed_width():
    """빈 닫힘 하한으로 clamp하면 파지가 쓸 수 있는 힘을 버린다 — 두 하한을
    나눈 이유가 그것이다."""
    profiles = _load_profiles()

    assert profiles._close_width(10.0) == profiles.GRIPPER_GRASP_MIN_MM
    assert profiles.GRIPPER_GRASP_MIN_MM < 9.0
