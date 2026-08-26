"""Perception 포트 — ROS2를 전혀 모르는 순수 ABC.
grippers_perception의 LearnedPerception이 이걸 구현한다."""

from abc import ABC, abstractmethod

from domain.values import Clearance, TargetObservation


class Perception(ABC):
    @abstractmethod
    def identify_target(self) -> TargetObservation | None:
        """정면에서 집을 물체를 자기 뎁스 카메라로 관측한다.

        raw YOLO 라벨과 함께 **전방 거리·좌우 오프셋**을 낸다 — GRASP 진입
        판정이 "물체가 턱이 쓸고 갈 영역 안에 있는가"를 재려면 라벨만으로는
        부족하기 때문이다(domain/task/grasp_alignment.py).

        **못 찾거나 확신할 수 없으면 `None`** — GRASP 조건 판정이 그걸
        미충족으로 읽어 Host에 되돌려준다(preconditions.check_grasp).
        찾았지만 거리를 환산 못 했으면 `metric_ok=False`로 돌려준다.

        왜 Pi가 이걸 하는가. 2026-08-26 팀 확정으로 Host 명령에는 좌표도
        라벨도 없다(state와 속도 넷뿐). 그런데 **내려가는 것은 이 팔**이고,
        어떤 교시 자세로 내려갈지는 무엇을 집는지에 달려 있다. 그래서
        "무엇이 앞에 있는가"만은 Pi가 자기 눈으로 확인한다.

        Host의 Geti 모델과 충돌하지 않는다 — 저쪽은 아레나 전체에서 목표를
        고르는 일이고, 이쪽은 이미 정해진 목표를 코앞에서 확인하는 일이다.
        훈련 데이터부터 다르다(이 모델은 로봇 시점 합성 데이터로 배웠다)."""

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
