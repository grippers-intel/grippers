"""FakeVla — 파지 정책 포트의 테스트 더블.

파지 경로가 정책 하나뿐이라(2026-09-07, classic 시퀀스 제거) 미션을 통주행
시키는 시험은 전부 이 포트를 갖고 있어야 한다. 없으면 GRASP 가 "VLA 포트가
없다"로 그 자리에서 죽는다.

⚠️ **턱은 안 건드린다.** 실기에서는 정책이 그리퍼를 직접 몰지만, 성공 판정
직전에 `BaselineGraspState.execute` 가 어차피 한 번 확실히 닫으라고 명령한다
(그 함수의 2026-09-07 주석). 물었는지 안 물었는지는 `FakeArm.jaw_blocked_raw`
가 정하므로, 여기서 폭을 또 건드리면 시험이 정한 상황을 덮어쓴다.
"""


class FakeVla:
    """포트 계약: `run_grasp(label, pan_bias_deg=0.0) -> bool`.

    `ok=False` 는 "정책 루프가 끝까지 못 돌았다"이다 — 물체를 놓쳤다는 뜻이
    아니다. 놓친 상황은 턱 위치로 시늉한다.
    """

    def __init__(self, ok: bool = True):
        self.ok = ok
        self.calls: list[tuple[str, float]] = []

    def run_grasp(self, label: str, pan_bias_deg: float = 0.0) -> bool:
        self.calls.append((label, float(pan_bias_deg)))
        return self.ok
