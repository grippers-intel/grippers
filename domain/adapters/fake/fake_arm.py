"""FakeArm — ArmDriver 포트의 테스트 더블. 하드웨어·ROS2 없이 도메인 FSM을 검증한다.
기본값은 전부 '성공'이며, 생성자 인자로 실패 시나리오를 주입한다."""

from domain.ports.arm_driver import ArmDriver
from domain.values import Point3

# 실측 기반 기본값 (2026-08-18, n=25, 정착 후 · 원시값 절대값 / 1023).
# get_load()는 포트 계약상 0~1 정규화 비율이다(domain/ports/arm_driver.py).
# 테스트가 1.0 같은 도달 불가능한 값을 쓰면 임계값이 실기와 어긋나 있어도
# 초록불이 나므로, Fake도 실측 분포 안의 값을 쓴다.
LOAD_EMPTY = 0.03  # 빈 채 / 파지 실패(놓침) 0.027~0.031
LOAD_HOLDING = 0.14  # 가베(정육면체) 0.137 — 5/5 일관
# 서보 읽기 자체가 실패했을 때의 신호값(2026-09-05) — LOAD_EMPTY와 다르다.
# ros2_arm_driver.LOAD_UNKNOWN / arm_driver_node.GRIPPER_LOAD_READ_FAILED와
# 같은 값이어야 한다(세 곳 다 서로 import하지 않고 독립적으로 정의한다).
LOAD_READ_FAILED = -1.0


class FakeArm(ArmDriver):
    def __init__(
        self,
        move_ok: bool = True,
        yaw_offset_ok: bool = True,
        drop_yaw_offset_ok: bool = True,
        reorient_ok: bool = True,
        fold_ok: bool = True,
        load_ratio: float | list[float] = LOAD_HOLDING,
    ):
        self._move_ok = move_ok
        self._yaw_offset_ok = yaw_offset_ok
        self._drop_yaw_offset_ok = drop_yaw_offset_ok
        self._reorient_ok = reorient_ok
        self._fold_ok = fold_ok
        # get_load()는 GRASP(높을수록 성공)과 HANDOVER(낮을수록 성공)가 정반대
        # 의미로 같이 쓴다 — 상수 하나로는 두 상태를 동시에 성공시킬 수 없어
        # ScriptedPerception.script처럼 호출 순서대로 값을 반환하고, 소진되면
        # 마지막 값을 반복한다. 스칼라를 주면 항상 그 값(기존 동작과 동일)이다.
        self._load_ratios = (
            [load_ratio] if isinstance(load_ratio, (int, float)) else list(load_ratio)
        )
        self._load_call_count = 0
        self.move_calls = []
        self.floor_pose_calls = []
        self.gripper_widths = []
        self.yaw_offsets = []
        self.drop_yaw_offsets = []
        # 붙잡기는 안전 경로다 — 복구가 실패했을 때 최소한 이건 불렸는지
        # 테스트가 확인할 수 있어야 한다(2026-08-29).
        self.hold_calls = 0
        # 턱을 막고 있는 물체의 위치(raw). None 이면 빈 턱이라 끝까지 닫힌다.
        # 기본은 초기 위치(=물고 있는 값)라, set_gripper 로 벌렸다 닫아도
        # 시험이 정한 "물고 있음"이 유지된다 — 실기에서도 물체는 그대로 있다.
        # 파지 실패(빈 턱)를 시늉하려면 gripper_position_raw_value 와 함께
        # 이 값도 낮추거나 None 으로 둔다.
        self.jaw_blocked_raw: int | None = self.gripper_position_raw_value

    def move_to_floor_pose(self, profile: str, stage: str) -> bool:
        self.floor_pose_calls.append((profile, stage))
        return self._move_ok

    def move_to_cartesian(self, xyz_m: Point3, down: bool = False) -> bool:
        self.move_calls.append((xyz_m, down))
        return self._move_ok

    #: 활짝(168mm) 열었을 때와 빈 채로 다 닫았을 때의 위치. 2026-09-07 실측.
    OPEN_RAW = 1989
    #: ⚠️ 2026-09-07 저녁 재실측: 1112 -> 1040. 서보 하한을 1090 -> 1007 로
    #: 내리면서 빈 턱이 더 깊이 들어간다(부하 136~144 로 스토퍼를 밀고 있다).
    #: baseline_constants.GRIPPER_EMPTY_POSITION_RAW 와 같은 값이어야 한다.
    EMPTY_RAW = 1040

    #: False 면 명령을 받아도 턱이 안 움직인다 — servo 6 통신이 죽은 상황.
    #: 2026-09-07 실기에서 실제로 그랬다(팀원 보고: "정리상자에 넣는 순간
    #: servo 6 오류로 그리퍼를 안 푼다").
    gripper_opens: bool = True

    def _raw_for(self, width_mm: float) -> int:
        """폭(mm) -> 위치(raw). 실측 두 점(0mm=1112, 168mm=1989)의 직선.

        보정표(gripper_calibration)의 3점 꺾은선을 그대로 옮기지 않는 이유는
        여기서 필요한 것이 "넓게 열면 크고 좁게 닫으면 작다"는 단조성뿐이기
        때문이다. 정확한 환산이 필요한 판정은 실기 상수를 쓴다."""
        span = self.OPEN_RAW - self.EMPTY_RAW
        return int(self.EMPTY_RAW + span * max(0.0, min(width_mm, 168.0)) / 168.0)

    def set_gripper(self, width_mm: float) -> None:
        """⚠️ 2026-09-07까지 이 함수는 폭을 **기록만** 하고 위치에는 아무
        영향이 없었다. 그래서 "명령은 보냈지만 안 열렸다"를 시늉할 수가
        없었고, 실기의 놓기 실패 버그가 시험을 그대로 통과했다
        (`release_until_open` 주석).

        포트 계약상 반환값이 없으므로(실패해도 조용하다) 호출부는 위치를
        읽어 확인해야 한다 — 그 확인이 의미를 가지려면 여기가 위치를
        움직여야 한다.

        턱 사이에 물체가 있으면 그 두께에서 멈춘다(`jaw_blocked_raw`) —
        실기에서 위치 판정이 성립하는 이유가 그 잔차다."""
        self.gripper_widths.append(width_mm)
        if not self.gripper_opens:
            return                      # 서보가 죽었다 — 명령을 받아도 안 움직인다
        target = self._raw_for(width_mm)
        if self.jaw_blocked_raw is not None:
            target = max(target, self.jaw_blocked_raw)
        self.gripper_position_raw_value = target

    #: 파지 성공 판정이 읽는 servo 6 위치(raw). 기본은 **물고 있는** 값이다 —
    #: 기존 시험 대부분이 "성공한 파지"를 전제로 쓰이기 때문이다.
    #: 빈 턱을 흉내 내려면 GRIPPER_EMPTY_POSITION_RAW(1112) 로 낮춘다.
    gripper_position_raw_value: int = 1200

    def gripper_position_raw(self) -> int:
        return self.gripper_position_raw_value

    def get_load(self) -> float:
        idx = min(self._load_call_count, len(self._load_ratios) - 1)
        self._load_call_count += 1
        return self._load_ratios[idx]

    def reorient(self, phi_rad: float) -> bool:
        return self._reorient_ok

    #: 접기를 몇 번 요청받았는지. VLA 시작 자세가 실제로 맞춰졌는지를
    #: 보는 시험이 쓴다(test_vla_only.py) — 안 세면 "불렀다"를 확인할 길이 없다.
    fold_calls: int = 0

    def fold_to_cradle(self) -> bool:
        self.fold_calls += 1
        return self._fold_ok

    def offset_base_yaw(self, offset_rad: float) -> bool:
        """`yaw_offset_ok=False`로 한계각 초과·관절 범위 밖 거부를 주입한다."""
        self.yaw_offsets.append(offset_rad)
        return self._yaw_offset_ok

    def correct_drop_yaw(self, offset_rad: float) -> bool:
        """`drop_yaw_offset_ok=False`로 한계각 초과·관절 범위 밖 거부를
        주입한다 — offset_base_yaw와 별도 한계각을 쓰는 safe_300 전용
        메서드라 호출 기록도 따로 남긴다."""
        self.drop_yaw_offsets.append(offset_rad)
        return self._drop_yaw_offset_ok

    def hold_position(self) -> None:
        self.hold_calls += 1
