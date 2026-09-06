"""파지 청크 예산 — 정책을 갈아 끼워도 동작 시간이 같아야 한다 (2026-09-06).

## 무엇이 고장 났었나

`MAX_CHUNKS = 10` 이라는 **개수** 상한이 있었다. 그 10 은 "ACT 실측 최대가
7.1청크"에서 나왔는데, ACT 의 한 청크는 3.33초다 — 그 상한이 뜻한 것은
**33.3초의 동작**이지 "10번"이 아니었다.

DP(청크 1.07초)로 갈아 끼우자 같은 10개가 10.7초가 됐다. 정책이 팔을 뻗다
말고 잘렸고, 그 뒤 하드코딩 carry 자세가 이어받았다. 밖에서 보면 "정책이
아니라 하드코딩이 파지한다"로 보인다 — 사용자가 실제로 그렇게 보고했다.

## 그래서 무엇을 못 박는가

**정책이 바뀌어도 동작 시간 예산은 같아야 한다.** 개수는 청크 길이에서
계산되는 결과값이지 사람이 잡는 값이 아니다.
"""

import importlib.util
import pathlib

import pytest

MODULE = (pathlib.Path(__file__).resolve().parent.parent
          / "ros2_ws" / "src" / "grippers_vla" / "grippers_vla" / "grasp_budget.py")


def _load():
    spec = importlib.util.spec_from_file_location("grasp_budget", MODULE)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


gb = _load()

#: 두 정책의 실제 값. 학습 fps 는 둘 다 30 이다.
ACT_STEPS, DP_STEPS, FPS = 100, 32, 30.0


def test_ACT_는_예전과_똑같이_10청크다():
    """이 시험이 깨지면 검증된 ACT 경로의 동작이 바뀐 것이다."""
    chunk_sec, max_chunks, _ = gb.chunk_budget(ACT_STEPS, FPS, timeout_s=45.0)
    assert round(chunk_sec, 2) == 3.33
    assert max_chunks == 10


def test_DP_는_같은_시간을_32청크로_쓴다():
    chunk_sec, max_chunks, _ = gb.chunk_budget(DP_STEPS, FPS, timeout_s=45.0)
    assert round(chunk_sec, 2) == 1.07
    assert max_chunks == 32


@pytest.mark.parametrize("steps", [100, 64, 32, 16, 8])
def test_어떤_정책이든_동작_시간_예산은_같다(steps):
    """이것이 이 모듈의 존재 이유다 — 개수가 아니라 시간이 상수다."""
    chunk_sec, max_chunks, _ = gb.chunk_budget(steps, FPS, timeout_s=999.0)
    covered = chunk_sec * max_chunks
    assert covered >= gb.MAX_GRASP_MOTION_SEC, f"{steps}스텝: {covered:.1f}s 밖에 못 간다"
    # ceil 한 칸을 넘게 주지는 않는다 — 상한이 헐거워지면 안전장치가 무뎌진다.
    assert covered < gb.MAX_GRASP_MOTION_SEC + chunk_sec


def test_청크가_아주_길어도_완료_판정_기회는_남긴다():
    """MIN_CHUNKS 안에서 끝나면 '뻗었다가 복귀'를 판정할 수가 없다."""
    _, max_chunks, _ = gb.chunk_budget(n_action_steps=3000, fps=FPS, timeout_s=45.0)
    assert max_chunks > gb.MIN_CHUNKS


def test_시계가_모자라면_늘려서_돌려준다():
    """조용히 끊기면 '정책이 중간에 멈췄다'로 보인다 — 그게 이번 고장이었다."""
    _, max_chunks, budget = gb.chunk_budget(DP_STEPS, FPS, timeout_s=45.0)
    assert budget > 45.0
    assert budget >= max_chunks * (DP_STEPS / FPS)


def test_시계가_넉넉하면_건드리지_않는다():
    """호출자가 일부러 크게 준 값을 줄이면 안 된다."""
    _, _, budget = gb.chunk_budget(ACT_STEPS, FPS, timeout_s=300.0)
    assert budget == 300.0


def test_ACT_는_기존_기본값_45초로도_충분하다():
    """ACT 경로에서는 경고가 뜨면 안 된다 — 예전과 동작이 같아야 한다."""
    _, _, budget = gb.chunk_budget(ACT_STEPS, FPS, timeout_s=45.0)
    assert budget == 45.0


@pytest.mark.parametrize("fps", [0.0, -5.0])
def test_망가진_fps_에도_안_죽는다(fps):
    """0 이 들어오면 나눗셈이 터진다 — 파지 도중에 죽는 것이 최악이다."""
    chunk_sec, max_chunks, budget = gb.chunk_budget(DP_STEPS, fps, timeout_s=45.0)
    assert chunk_sec > 0 and max_chunks > gb.MIN_CHUNKS and budget > 0


def test_예산_상수가_ACT_실측에서_유도된_값_그대로다():
    """33.3 = 10청크 x 3.33초. 이 숫자를 바꾸면 검증된 동작이 바뀐다."""
    assert gb.MAX_GRASP_MOTION_SEC == pytest.approx(10 * ACT_STEPS / FPS, abs=0.05)
