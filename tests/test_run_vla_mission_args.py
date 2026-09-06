"""실행 스크립트가 넘기는 인자가 런치에 **선언돼 있는지** 검사한다 (2026-09-07).

## 왜 이 시험이 있는가

`ros2 launch` 는 모르는 인자를 오류로 알리지 않는다. 그냥 조용히 무시한다.
이 저장소는 그 때문에 같은 사고를 두 번 겪었다.

    2026-09-01  host_ip 를 안 넘겨 노드 기본값(192.168.0.10)으로 보고가 나갔다
    2026-09-05  auto_align_on_first_move:=false 가 런치에 배선이 안 돼 무시됐다

두 번째 경우가 특히 나쁘다 — 사람은 껐다고 믿고 실기를 돌렸다. 인자 이름에
오타가 나거나 런치에서 인자가 사라져도 증상은 똑같이 "조용함"이다.

그래서 실행 스크립트를 만들었으면(tools/run_vla_mission.sh) 그 스크립트가
쓰는 이름이 런치의 DeclareLaunchArgument 와 맞는지를 기계가 봐야 한다.
"""

import pathlib
import re

import pytest

ROOT = pathlib.Path(__file__).resolve().parent.parent
SCRIPT = ROOT / "tools" / "run_vla_mission.sh"
LAUNCH = (ROOT / "ros2_ws" / "src" / "grippers_bringup" / "launch"
          / "bringup.launch.py")


def _declared():
    """런치가 선언한 인자 이름."""
    return set(re.findall(r'DeclareLaunchArgument\(\s*\n\s*"(\w+)"',
                          LAUNCH.read_text(encoding="utf-8")))


def _passed():
    """스크립트가 `이름:=값` 형태로 넘기는 인자 이름."""
    text = SCRIPT.read_text(encoding="utf-8")
    # 주석 줄은 뺀다 — 설명문에 나온 이름까지 세면 안 된다.
    body = "\n".join(l for l in text.splitlines() if not l.lstrip().startswith("#"))
    return set(re.findall(r'([a-z_]+):=', body))


def test_스크립트가_넘기는_인자가_전부_선언돼_있다():
    """⚠️ 여기서 걸리는 이름은 실기에서 **조용히 무시된다.**"""
    undeclared = _passed() - _declared()
    assert not undeclared, f"런치에 없는 인자: {sorted(undeclared)}"


def test_사고났던_인자는_반드시_넘긴다():
    """host_ip 는 안 넘기면 노드 기본값 192.168.0.10 으로 보고가 샌다."""
    assert "host_ip" in _passed()


@pytest.mark.parametrize("name, why", [
    ("use_depth_gate", "뎁스캠이 이 구성에 없다 — 켜면 파지 판정이 영영 막힌다"),
    ("use_depth_camera", "켜 두면 그리퍼캠이 0.4Hz 로 굶는다(2026-09-07 실측)"),
    ("grasp_backend", "vla 가 아니면 정책이 아니라 classic 파지가 돈다"),
    ("use_vla", "false 면 그리퍼캠 발행까지 함께 꺼진다"),
])
def test_실기에서_필요했던_인자를_빠뜨리지_않는다(name, why):
    assert name in _passed(), f"{name} 이 빠졌다 — {why}"


def test_두_백엔드_모두_추론_경로를_지정한다():
    """policy_source 를 안 넘기면 기본값에 기대게 된다 — 기본값은 조용히
    바뀔 수 있고, 그때 어느 모델이 돌았는지 사후에 알 수가 없다."""
    text = SCRIPT.read_text(encoding="utf-8")
    assert "policy_source:=local" in text
    assert "policy_source:=remote" in text


def test_ACT_체크포인트가_저장소_밖을_가리킨다():
    """저장소 안에 두면 git stash -u 에 통째로 휩쓸린다 — 2026-09-05 에
    ckpt_v5_all 이 그렇게 사라졌다(런치 파일의 같은 경고)."""
    text = SCRIPT.read_text(encoding="utf-8")
    ckpt = re.search(r'^CHECKPOINT=(\S+)', text, re.M).group(1)
    assert ckpt.startswith("/shared/"), ckpt


def test_런치_기본_체크포인트와_같은_것을_쓴다():
    """둘이 갈라지면 '스크립트로 돌린 것'과 '손으로 돌린 것'이 다른 모델이
    된다 — 실기 결과를 비교할 수 없게 된다."""
    script_ckpt = re.search(r'^CHECKPOINT=(\S+)',
                            SCRIPT.read_text(encoding="utf-8"), re.M).group(1)
    launch_ckpt = re.search(
        r'"checkpoint",\s*\n\s*default_value="([^"]*)"',
        LAUNCH.read_text(encoding="utf-8")).group(1)
    assert script_ckpt == launch_ckpt
