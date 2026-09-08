"""조준각(pan_bias)은 좌표계 오프셋이지 매 청크마다 더하는 값이 아니다.

## 2026-09-08 실기에서 팔이 40도까지 돌아갔다

`vla_inference_node` 는 정책 출력의 shoulder_pan 열에 조준각을 더한다. 그런데
정책이 읽는 `state` 는 **실측 관절값**이라 직전에 더한 조준각이 이미 들어 있다.
정책은 절대 위치를 내므로 지금 자세를 대체로 유지하고, 거기에 또 더하니 청크마다
톱니처럼 쌓였다. 그날 기록(runs/vla/.../chunks.jsonl):

    청크  실측pan  정책출력
      1    -2.95    -3.67
      2     3.82     2.68     <- +6.8
      4    14.99    15.11
      7    38.02    36.75     <- 학습 범위는 -15.4 ~ +16.9

학습 분포 밖이라 파지가 될 리 없다 — grasp_alignment.VLA_PAN_LIMIT_DEG 주석이
"좌우는 사실상 배우지 못한 축이라 분포 밖으로 나가면 그냥 실패한다"고 적고 있다.

## 고침

정책을 **조준하지 않은 좌표계**에서 돌린다. 입력 state 에서 빼고 출력에 더하면
정상 상태에서 `정책이 보는 값 = 실측 - bias = 자기 출력` 이라 더 쌓이지 않는다.
"""

import ast
import pathlib

NODE = (pathlib.Path(__file__).resolve().parent.parent / "ros2_ws" / "src"
        / "grippers_vla" / "grippers_vla" / "vla_inference_node.py")


def _source() -> str:
    return NODE.read_text(encoding="utf-8")


def test_state에서_빼고_출력에_더한다():
    """⚠️ 한쪽만 있으면 톱니가 돌아온다."""
    src = _source()
    assert "state[0] -= pan_bias" in src, "입력에서 빼지 않는다 — 톱니가 생긴다"
    # 첫 청크는 예외다 — 그 시점의 팔에는 아직 얹은 것이 없다.
    assert "if pan_bias and chunks > 0:" in src, "첫 청크 예외가 없다"
    assert "chunk[:, 0] += pan_bias" in src, "출력에 더하지 않는다 — 조준이 안 먹는다"
    # 빼는 것이 추론보다 **먼저**여야 한다. 뒤면 정책은 여전히 편향된 값을 본다.
    assert (src.index("state[0] -= pan_bias")
            < src.index("predict_chunk(frame, state, task)")
            < src.index("chunk[:, 0] += pan_bias")), "순서가 뒤집혔다"


def _envelope() -> float:
    tree = ast.parse(_source())
    return next(node.value.value for node in tree.body
                if isinstance(node, ast.Assign)
                and getattr(node.targets[0], "id", None) == "PAN_ENVELOPE_DEG")


PAN_ENVELOPE = _envelope()


def test_분포를_벗어나면_경고한다():
    """톱니가 다른 경로로 되살아나도 조용히 넘어가지 않게 하는 그물."""
    src = _source()
    assert "PAN_ENVELOPE_DEG" in src
    envelope = _envelope()
    # 학습 실측 범위(-15.4~+16.9)보다는 넓고, 그날 도달한 40도보다는 좁아야
    # 경고로서 의미가 있다.
    assert 17.0 < envelope < 40.0, envelope


#: 정책의 pan 거동 모형. 지금 본 자세에서 학습 평균 쪽으로 조금씩 되돌아온다.
#: k 가 작은 것이 근거다 — 좌우는 정책이 거의 못 배운 축이라(이미지 응답이
#: 액션 std 의 0.62%) 대체로 지금 자세를 유지한다. 2026-09-08 기록에서도
#: 정책출력이 실측을 1도 안쪽으로 따라갔고, 끝의 두 청크에서만 되돌아왔다.
PULL_BACK = 0.15
LEARNED_PAN_DEG = -2.34          # v5_all 118회차 평균


def _simulate(bias, chunks, subtract_from_state):
    """pan 궤적을 흉내 낸다. 반환값은 **실측**(= 우리가 보낸 값) 궤적이다."""
    measured = 0.0
    trail = []
    for _ in range(chunks):
        first = not trail
        seen = (measured - bias
                if (subtract_from_state and not first) else measured)
        out = seen + PULL_BACK * (LEARNED_PAN_DEG - seen)
        measured = out + bias
        trail.append(measured)
    return trail


def test_옛_방식은_청크마다_쌓인다():
    """이 시험은 **버그를 재현**한다 — 고침이 왜 필요했는지 남긴다.

    되돌아오는 힘(PULL_BACK)이 있어도, 매 청크 bias 를 얹으면 평형점이
    `학습평균 + bias/k` 로 밀려난다. k 가 작으니 그 자리가 아주 멀다."""
    trail = _simulate(bias=7.5, chunks=8, subtract_from_state=False)

    assert trail[-1] > PAN_ENVELOPE, f"쌓이지 않았다: {trail}"
    assert all(b > a for a, b in zip(trail, trail[1:])), f"단조 증가가 아니다: {trail}"


def test_지금_방식은_조준각_근처로_수렴하고_머문다():
    """좌표계 오프셋이면 정책은 자기 좌표계에서 평소대로 학습평균으로 수렴하고,
    실측은 그것 + bias 에 머문다 — bias 가 몇 번 곱해지지 않는다."""
    bias = 7.5
    trail = _simulate(bias=bias, chunks=30, subtract_from_state=True)

    settled = LEARNED_PAN_DEG + bias
    assert abs(trail[-1] - settled) < 0.5, f"수렴점이 다르다: {trail[-1]:.2f}"
    assert max(trail) < PAN_ENVELOPE, f"분포 밖으로 나갔다: {max(trail):.1f}"
    # 첫 청크에서 이미 조준각 부근이다 — 한 번에 옮겨 가고 그 뒤엔 안 쌓인다.
    # (첫 청크에 빼지 않는 이유가 이것이다. 빼면 여기서 0.8도밖에 못 간다.)
    assert abs(trail[0] - bias) < 2.0, trail[0]
