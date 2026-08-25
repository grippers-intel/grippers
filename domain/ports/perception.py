"""Perception 포트 — ROS2를 전혀 모르는 순수 ABC.
grippers_perception의 LearnedPerception이 이걸 구현한다."""

from abc import ABC, abstractmethod

from domain.values import BoxObservation, Clearance, Destination, Detection


class Perception(ABC):
    @abstractmethod
    def scan_floor(self) -> list[Detection]:
        """바닥을 전역 관측해 검출 목록을 반환한다.
        더 처리할 대상이 없으면(상자 영역 마스킹 포함) **빈 리스트**를 반환해야 한다 —
        `SCAN` 은 빈 리스트를 '남은 대상 없음'으로 해석해 `DONE` 으로 전이한다.

        **실패(서비스 부재 · 응답 없음)도 빈 리스트.** 관측이 안 되는데 계속 도는
        것보다 미션을 끝내고 이유를 로그로 남기는 편이 낫다."""

    @abstractmethod
    def find_box(self, dest: Destination) -> BoxObservation | None:
        """지정한 목적지(왼쪽/오른쪽) 바구니를 관측한다. 찾지 못하면 **`None`** 을
        반환해야 한다 — `TRANSPORT` 는 `None` 을 받으면 대상을 보류 등록하고
        `SCAN` 으로 복귀한다. 서비스 부재 · 응답 없음도 같은 `None` 이다.

        ⚠️ 2026-08-23 확정 미션 명세서: 바구니 좌표는 하드코딩이고 색 탐색은
        하지 않는다. 그래도 이 메서드가 남아 있는 이유는, 하드코딩된 좌표
        근처에서 실제 바구니 자세(opening_mm·long_axis_rad 등 INSERT가 쓰는
        정밀값)를 확인하는 역할까지는 아직 없애지 않았기 때문이다 — "색으로
        어디 있는지 찾는다"에서 "위치는 이미 알고, 그 자리의 상세를 잰다"로
        의미가 바뀌었을 뿐이다."""

    @abstractmethod
    def measure_opening(self, box: BoxObservation) -> float | None:
        """`box` 앞에 정렬한 상태에서 입구 폭(mm)을 정밀 실측한다.
        `POSE_PLAN` 이 이 값으로 φ 해 구간을 계산한다.

        **실측하지 못하면(서비스 부재 · 응답 없음) `None`** — '해 없음' 취급이라
        `POSE_PLAN` 이 `REJECT` 로 보낸다. 입구 폭을 모르는 채로 투입을 시도하면
        상자 테두리에 물체를 찍는다."""

    @abstractmethod
    def monitor_clearance(self) -> Clearance:
        """여유 공간을 관측한다. **실측 수단이 없거나 응답이 없으면(서비스 부재 ·
        타임아웃 포함) `contact_risk=True`(정지)** — '모르면 멈춘다'가 이 포트의
        기본값이다. 측정 실패를 통과 신호로 두면 실제 장애물을 못 보고 밀고
        지나가는 사고로 직결되므로, 이 메서드만은 실패값이 안전 쪽으로 치우쳐
        있어야 한다.

        ⚠️ '항상 True' 가 아니다. 비전이 미구현인 동안 real 구현이 True를 고정
        반환하는 것은 **그 구현의 현재 상태**이지 포트 계약이 아니다. 시나리오를
        주입받는 테스트 더블은 '모르는' 상태가 아니므로 기본값이 happy path여도
        계약 위반이 아니다 (`ScriptedPerception.monitor_clearance` 참고)."""

    @abstractmethod
    def remember_target(self, raw_cls: str) -> bool:
        """GRASP 로 내려가기 **직전에** 목표 물체를 기억해 둔다.

        `confirm_grasp()` 가 "그때 거기 있던 것이 지금은 없다"를 판정하려면
        기준이 필요하다. 기억에 실패하면(관측 불가 등) **`False`** — 그 뒤의
        `confirm_grasp()` 는 비교 기준이 없으므로 판정을 포기한다.
        """

    @abstractmethod
    def confirm_grasp(self) -> bool:
        """CARRY_IDLE 에서 정면을 다시 보고 목표 물체가 사라졌는지 확인한다.

        원리 — **부재가 증거다.** 파지에 성공했으면 물체는 바닥에서 사라져
        그리퍼에 있다. 실패했으면 여전히 바닥에 있다. 그리퍼캠으로 "손끝에
        물려 있는가"를 보려던 예전 방식은 실측으로 무효였다(빈 그리퍼 닫힘
        165990px² 가 룩을 문 상태 70384px² 보다 컸다). 물체가 있던 자리를
        보는 쪽이 훨씬 다루기 쉬운 신호다.

        CARRY_IDLE 에서 팔이 depth 카메라를 가리지 않는다는 것은 2026-08-25
        실기로 확인했다 — 팔은 프레임 밖이고 바닥이 그대로 보인다.

        ⚠️ 이것만으로 성공을 단정하면 안 된다. 내려오는 그리퍼가 물체를
        **쳐서 시야 밖으로 밀어낸** 경우에도 "사라짐"으로 보인다. load 신호와
        **독립적인 두 번째 근거**로 쓰는 것이 이 포트의 목적이다 —
        load 는 "무언가를 쥐었다"를, 이쪽은 "목표가 그 자리에서 없어졌다"를
        말하므로 실패 양상이 서로 겹치지 않는다.

        **관측 불가 · 기준 없음은 `False`** — 다른 관측 포트와 같은 "모르면
        실패" 관례를 따른다."""
