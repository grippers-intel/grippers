"""GRASP/INSERT 전제 조건 판정의 계약을 고정한다 (팀 확정 임무 2번·4번).

Pi가 판단하는 것은 자기 센서로만 알 수 있는 것뿐이다. 좌표가 필요한
판단(물체 앞에 제대로 섰는가 등)이 여기 들어오면 그건 Host의 일이 샌 것이다."""

import math

from domain.task import baseline_constants as bc
from domain.task.preconditions import (
    GraspInputs,
    InsertInputs,
    check_grasp,
    check_insert,
)


def _grasp(**overrides):
    base = dict(base_stopped=True, detected_label="queen")
    base.update(overrides)
    return GraspInputs(**base)


def _insert(**overrides):
    base = dict(estop_set=False, base_stopped=True, gripper_load=0.0626,
                face_ok=True, face_distance_m=bc.BASKET_STOP_LIDAR_M,
                face_yaw_error_rad=0.0, face_reason="정면 확보", profile="chess_queen",
                # 2026-08-26 검증 지점의 실측 수준 값들.
                face_point_count=97, face_lateral_offset_m=0.0,
                face_lateral_known=False,
                distance_change_m=0.0, load_change=0.0)
    base.update(overrides)
    return InsertInputs(**base)


# ── GRASP ─────────────────────────────────────────────────────────────────


def test_전부_충족되면_통과한다():
    report = check_grasp(_grasp())

    assert report.ok
    assert report.reasons == ()
    assert report.detected_label == "queen"


def test_자기_카메라가_목표를_못_보면_막는다():
    report = check_grasp(_grasp(detected_label=None))

    assert not report.ok
    assert any("찾지 못했다" in reason for reason in report.reasons)


def test_차체가_움직이는_중이면_막는다():
    """팔이 내려가는 동안 차체가 움직이면 교시 자세의 전제가 깨진다."""
    report = check_grasp(_grasp(base_stopped=False))

    assert not report.ok


def test_미충족_사유를_전부_모아서_돌려준다():
    """무엇을 고쳐야 하는지 알려줘야 Host가 수정된 명령을 만들 수 있다."""
    report = check_grasp(_grasp(base_stopped=False, detected_label=None))

    assert len(report.reasons) == 2
    assert report.detail.count("/") == 1


# ── INSERT ────────────────────────────────────────────────────────────────


def test_라이다가_거리와_정렬을_맞추면_통과한다():
    report = check_insert(_insert())

    assert report.ok


def test_정면을_못_잡으면_거리는_보지도_않는다():
    """`ok=False`일 때 거리 숫자는 의미가 없다."""
    report = check_insert(_insert(face_ok=False, face_distance_m=math.inf,
                                  face_reason="점 부족"))

    assert not report.ok
    assert len(report.reasons) == 1
    assert "점 부족" in report.reasons[0]


def test_바구니가_허용치보다_멀면_막는다():
    far = bc.BASKET_STOP_LIDAR_M + bc.BASKET_STOP_TOLERANCE_M + 0.01
    report = check_insert(_insert(face_distance_m=far))

    assert not report.ok
    assert any("멀다" in reason for reason in report.reasons)


def test_절벽_아래를_읽으면_막는다():
    """빔이 테두리를 넘어가면 판독값이 커지는 방향으로 틀린다 — 하한 아래는
    우리가 교정한 그 면이 아닐 수 있다는 신호다."""
    report = check_insert(_insert(face_distance_m=bc.BASKET_MIN_LIDAR_M - 0.001))

    assert not report.ok
    assert any("하한보다 가깝다" in reason for reason in report.reasons)


def test_실기에서_성공한_두_지점은_모두_통과한다():
    """2026-08-26 나이트 0.1386, 퀸 0.1301 — 둘 다 투하에 성공했다."""
    assert check_insert(_insert(face_distance_m=0.1386)).ok
    assert check_insert(_insert(face_distance_m=0.1301)).ok


def test_실기에서_성공한_정렬_오차는_통과한다():
    """+2.82도로 투하에 성공했다."""
    assert check_insert(_insert(face_yaw_error_rad=math.radians(2.82))).ok


def test_정렬이_허용치를_넘으면_막는다():
    report = check_insert(_insert(face_yaw_error_rad=math.radians(20.0)))

    assert not report.ok
    assert any("정렬이 틀어졌다" in reason for reason in report.reasons)


def test_빈손이면_막는다():
    """빈손으로 투하 자세를 펼쳐 봐야 팔만 위험하게 뻗는다."""
    report = check_insert(_insert(gripper_load=0.0313))

    assert not report.ok
    assert any("비어 있다" in reason for reason in report.reasons)


def test_무엇을_들고_있는지_모르면_막는다():
    report = check_insert(_insert(profile=None))

    assert not report.ok
    assert any("놓기 폭을 정할 수 없다" in reason for reason in report.reasons)


# ── 2026-08-26에 추가한 네 가지 조건 ──────────────────────────────────────


def test_좌우로_밀려_있으면_막는다():
    """거리와 yaw만으로는 안 보이는 오차다 — 둘 다 정상인데 바깥에 떨어진다."""
    report = check_insert(_insert(face_lateral_known=True,
                                  face_lateral_offset_m=0.090))

    assert not report.ok
    assert any("좌우로 밀려" in reason for reason in report.reasons)


def test_좌우_오프셋을_모르면_통과시킨다():
    """양쪽 가장자리가 다 창 밖이면 그 자체가 '충분히 가운데'라는 뜻이다."""
    assert check_insert(_insert(face_lateral_known=False,
                                face_lateral_offset_m=0.0)).ok


def test_허용_범위_안의_좌우_오프셋은_통과한다():
    assert check_insert(_insert(face_lateral_known=True,
                                face_lateral_offset_m=0.050)).ok


def test_정면_점이_부족하면_막는다():
    """빔이 테두리를 스치기 시작하면 완전히 놓치기 전에 점이 먼저 준다."""
    report = check_insert(_insert(face_point_count=20))

    assert not report.ok
    assert any("점이 부족" in reason for reason in report.reasons)


def test_직전_판독이_없으면_한_사이클_더_본다():
    report = check_insert(_insert(distance_change_m=None))

    assert not report.ok
    assert any("직전 판독이 없다" in reason for reason in report.reasons)


def test_판독이_흔들리면_막는다():
    """합의 속도 0.1m/s면 한 사이클에 10mm 움직인다 — 주행 중이면 반드시 걸린다."""
    report = check_insert(_insert(distance_change_m=-0.010))

    assert not report.ok
    assert any("흔들린다" in reason for reason in report.reasons)


def test_잡음_수준의_변화는_통과한다():
    assert check_insert(_insert(distance_change_m=0.003)).ok


def test_부하가_떨어지고_있으면_막는다():
    """팔을 크게 펼치기 전에 미끄러짐을 잡는다."""
    report = check_insert(_insert(load_change=-0.020))

    assert not report.ok
    assert any("미끄러지는" in reason for reason in report.reasons)


def test_부하가_올라가는_것은_막지_않는다():
    assert check_insert(_insert(load_change=+0.020)).ok


# ── 부하 임계가 실측 두 띠 사이에 있는가 ──────────────────────────────────
# 2026-08-26 실측. 하드웨어에서 잰 값이라 코드가 바뀐다고 달라지지 않는다 —
# 임계를 손대면 이 테스트가 먼저 걸리게 해 둔다.
# 판독은 1/256 격자 위에서만 나온다 — 실측 표시값 0.0235/0.0430은
# 각각 6/256, 11/256이다. 격자 값으로 적어야 여유 계산이 반올림에
# 안 흔들린다.
EMPTY_LOAD_BAND = (6.0 / 256.0, 11.0 / 256.0)
# 파지 쪽 하한은 **INSERT 시점의 퀸**이다. 팔을 펼치면 물체 무게 벡터가
# 손목에 대해 회전해 CARRY(0.0547)보다 1양자 낮게 읽힌다. 가장 낮게 읽히는
# 자세를 기준으로 잡지 않으면 팔을 펴는 순간 판정이 뒤집힌다.
GRIPPED_LOAD_BAND = (13.0 / 256.0, 25.0 / 256.0)
LOAD_QUANTUM = 1.0 / 256.0            # 판독 양자 — 관측값이 전부 이 배수다


def test_부하_임계가_빈손_띠_위에_있다():
    """0.04는 빈손 상한 0.0430보다 **낮았다** — 상한 쪽으로 떠돈 순간에
    빈손을 파지 성공으로 읽었다."""
    assert bc.LOAD_THRESHOLD > EMPTY_LOAD_BAND[1]
    assert bc.EMPTY_LOAD_CEILING > EMPTY_LOAD_BAND[1]


def test_부하_임계가_INSERT_시점_파지_부하_아래에_있다():
    """0.0528은 INSERT 시점 실측 0.0508보다 **높았다** — check_insert가
    성공한 파지를 막는다. 2026-08-26 통주행이 그 조건을 실제로 지나갔다."""
    assert bc.LOAD_THRESHOLD < GRIPPED_LOAD_BAND[0]


def test_부하_임계가_양쪽으로_한_양자_이상_떨어져_있다():
    """두 띠 사이가 2양자뿐이라 각 방향으로 1양자가 데이터가 허락하는 전부다.

    여유가 이렇게 얇다는 것이 곧 **부하 단독 판정을 쓰면 안 되는 이유**다 —
    파지 판정은 부하와 뎁스 카메라를 둘 다 요구한다(사용자 지시)."""
    below = bc.LOAD_THRESHOLD - EMPTY_LOAD_BAND[1]
    above = GRIPPED_LOAD_BAND[0] - bc.LOAD_THRESHOLD

    assert below >= LOAD_QUANTUM, f"빈손 쪽 여유가 {below / LOAD_QUANTUM:.1f} 양자뿐"
    assert above >= LOAD_QUANTUM, f"파지 쪽 여유가 {above / LOAD_QUANTUM:.1f} 양자뿐"


def test_실측_두_띠는_겹치지_않는다():
    """겹치면 임계를 어디 두든 못 가른다 — 그때는 부하 조건을 판정에서 빼고
    뎁스 카메라에만 기대야 한다. 그 경계를 눈에 보이게 못박아 둔다."""
    assert EMPTY_LOAD_BAND[1] < GRIPPED_LOAD_BAND[0]
