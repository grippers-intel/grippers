"""ArmDriver 포트 — ROS2를 전혀 모르는 순수 ABC."""

from abc import ABC, abstractmethod

from domain.values import Point3


class ArmDriver(ABC):
    @abstractmethod
    def move_to_floor_pose(self, profile: str, stage: str) -> bool:
        """실측된 수평 바닥 파지 자세로 단계 이동한다.

        ``stage`` 는 ``idle``, ``safe``(145 mm), ``grasp``, ``midpoint`` 또는
        바구니 투하용 ``drop``(300 mm, 2026-09-04부터 — 옛 195 mm)이다.
        프로필/단계를 지원하지 않거나 하드웨어 이동에 실패하면 ``False``다.
        """

    @abstractmethod
    def move_to_cartesian(self, xyz_m: Point3, down: bool = False) -> bool:
        """손끝을 xyz_m(m)로 이동한다. 도달 불가하면 False.
        그리퍼 개폐는 이 메서드가 하지 않는다 — `set_gripper()` 를 별도로 호출한다.

        **실패(도달 불가 · 서버 부재 · 응답 없음)는 예외가 아니라 `False`.**

        ⚠️ 2026-08-28: 현재 실기 FSM(`baseline_mission.BaselineGraspState`)은
        이 메서드를 안 부른다 — 좌표 기반 파지였던 예전 설계의 흔적이고,
        지금은 실측 프로필/단계 기반인 `move_to_floor_pose()`로 대체됐다.
        real/fake 어댑터와 액션 서버는 계속 살아 있고 테스트도 돌지만,
        미션 경로에서는 죽은 코드다 — 지우지 않은 이유는 손끝 좌표 이동이
        나중에 다시 필요해질 수 있어서다(예: 임의 물체 위치 대응)."""

    @abstractmethod
    def set_gripper(self, width_mm: float) -> None:
        """그리퍼 개구 폭을 width_mm(mm)로 맞춘다.
        ⚠️ 단위가 deg(각도)에서 mm(개구 폭)로 바뀌었다. 서보 각도 변환은
        어댑터(FeetechArm) 내부 캘리브레이션 테이블이 담당한다 (미결 #4 결과 반영).

        **돌려줄 값이 없으므로 실패는 로그로만 남는다.** 다만 조용히 삼켜지지는
        않는다 — 그리퍼가 닫히지 않았으면 뒤이은 `get_load()` 가 빈 채 부하를
        읽어 `GRASP` 가 파지 실패로 판정한다."""

    @abstractmethod
    def get_load(self) -> float:
        """그리퍼(id6) 부하 비율 — **0.0~1.0 으로 정규화된 값**이다.

        ⚠️ 서보 원시값(STS3215 PRESENT_LOAD 는 0~1023)을 그대로 돌려주면 안 된다.
        정규화는 어댑터 뒤편(arm_driver_node)의 몫이다 — 도메인은 서보 레지스터
        범위를 알지 못한다. Fake 는 정규화된 값을, real 은 원시값을 주는 식으로
        계약이 갈라지면 CI는 통과하는데 실기에서만 파지 판정이 항상 실패한다.

        **실패(서비스 부재 · 응답 없음 · 서보 읽기 실패)는 `-1.0`**
        (2026-09-05까지는 `0.0`이었다 — 실제 부하값의 유효 범위 안이라 '빈
        채로 읽었다'와 '읽기 자체가 실패했다'를 구분하지 못했고, 이게
        2026-09-04 box/queen 파지 오판정과 그 뒤 INSERT 부하-안정성 오판의
        공통 원인이었다). `-1.0`은 정규화 비율이 절대 나올 수 없는 값이라
        실제 부하와 충돌하지 않는다. **호출부는 이 값을 "모른다"로 다뤄야지
        "비었다"로 다루면 안 된다** — 문턱과 그대로 비교하는 것(우연히 미달로
        나온다)까지는 괜찮지만, 직전 표본과의 차분(부하 안정성·놓기 확인 같은
        검사)에는 절대 그대로 넣으면 안 된다. 진짜 부하가 뚝 떨어지거나 안
        줄어든 것처럼 보인다 — 반드시 먼저 `< 0`으로 걸러내고 그 사이클은
        판정을 보류해야 한다."""

    @abstractmethod
    def reorient(self, phi_rad: float) -> bool:
        """손목을 장축-수평면 각 φ(rad)로 재조정한다. 정착에 실패하면 False.

        **서버 부재 · 응답 없음도 `False`.**

        ⚠️ 2026-08-28: 실기 서버(`arm_driver_node._execute_reorient`)가 아직
        스텁이다 — 실제 손목 재조정 없이 `settled=True`만 돌려준다. 현재
        실기 FSM도 이 메서드를 안 부른다. move_to_cartesian과 같은 이유로
        남겨 둔, 아직 완성되지 않은 미래용 훅이다."""

    @abstractmethod
    def fold_to_cradle(self) -> bool:
        """팔을 이동용 거치 자세로 접는다. **실패는 `False`.**

        구현(`arm_driver_node._on_fold_to_cradle`)은 서보 부하를 접기 전후로
        확인하는 완성된 로직이고 테스트도 있다(`test_arm_hardware_contract.py`).
        다만 2026-08-28 기준 `baseline_mission`의 실기 FSM은 이 메서드를 안
        부른다 — 수동 도구·향후 이동 단계용으로 남겨 둔 상태다."""

    @abstractmethod
    def offset_base_yaw(self, offset_rad: float) -> bool:
        """servo 1(팔 베이스 요)을 현재 위치에서 offset_rad만큼 돌린다.

        GRASP 하강 **전에** 부르는 좌우 보정이다 — 물체가 턱이 쓸고 갈 영역
        안에 있지만 가운데가 아닐 때 Pi가 스스로 고치는 수단이다(사용자 지시
        2026-08-26). 메카넘 옆걸음이 아니라 이 관절을 쓰는 이유는 베이스의
        속도 데드밴드 때문에 최소 옆걸음이 15mm로 고치려는 오차보다 커서다.

        **한계각을 넘거나 관절 범위를 벗어나면 움직이지 않고 `False`.**
        무리하게 돌리는 것보다 Host에 다시 세워 달라고 하는 편이 싸다."""

    @abstractmethod
    def correct_drop_yaw(self, offset_rad: float) -> bool:
        """servo 1(팔 베이스 요)을 현재 위치에서 offset_rad만큼 돌린다.

        safe_300 — INSERT가 "drop"(300mm) 자세에 도달해 그리퍼를 열기
        **전에** 부르는 요 보정이다(사용자 지시, 2026-09-05). Host가 차량을
        NUDGE 경계선에서 방향에 상관없이 세우고 남은 잔여 지향 오차를
        yaw_correction_deg로 실어 보내면, 여기서 그만큼 흡수한다.

        `offset_base_yaw()`와 구현·서비스가 같은 모양(servo 1만 상대 회전)
        이지만 **한계각이 다르다** — 완전히 별도의 메서드로 둔다. 그쪽
        한계는 GRASP 턱 폭 허용치에서 역산된 물리적 근거가 있는 값이라
        건드리지 않는다(arm_driver_node.MAX_BASE_YAW_OFFSET_RAD 주석
        참고); 여기(드랍 직전, 물체를 이미 확실히 쥔 채 바닥 접촉 없는
        상황)는 물리적 상황 자체가 달라 더 넓게(사용자 지시로 45도) 잡는다.

        **한계각을 넘거나 관절 범위를 벗어나면 움직이지 않고 `False`.**
        그런 경우도 INSERT 자체는 포기하지 않는다 — 보정 없이 여는 것이
        물체를 든 채 무한정 멈춰 있는 것보다 낫다(BaselineInsertState
        참고)."""

    @abstractmethod
    def hold_position(self) -> None:
        """현재 관절 자세를 그대로 유지한다 (E-STOP 시 파지물 낙하 방지용).

        E-STOP 경로다 — **응답을 기다리지 않는다.** 실패해도 돌려줄 값이 없으므로
        로그만 남긴다."""
