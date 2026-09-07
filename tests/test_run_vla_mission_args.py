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


# ── 기본 백엔드와 사전 점검 (2026-09-07) ─────────────────────────────────
#
# 기본을 DP 로 돌린 것은 지금 무엇을 재고 있느냐에 따른 선택이다 — 실기에서
# 파지→운반→투하를 완주한 유일한 기록이 DP 쪽이고(06:00 판), RTC 를 붙일 수
# 있는 것도 DP 뿐이다. 대신 노트북 의존이 생기므로 그 대가를 시험으로 못 박는다.


def _var(name):
    m = re.search(rf'^{name}=(\S+)', SCRIPT.read_text(encoding="utf-8"), re.M)
    return m.group(1) if m else None


def test_기본_백엔드가_DP_다():
    assert _var("BACKEND") == "dp"


def test_ACT_로_되돌릴_길이_남아_있다():
    """노트북 없이 굴려야 할 때가 있다 — 그 길을 막으면 안 된다."""
    assert "--act)" in SCRIPT.read_text(encoding="utf-8")


def test_DP_는_띄우기_전에_서버를_두드린다():
    """⚠️ 없으면 vla_inference_node 가 기동에서 죽는데, 그 실패는 ROS 로그
    깊숙이 묻힌다. 먼저 물어보면 한 줄로 끝난다."""
    text = SCRIPT.read_text(encoding="utf-8")
    assert "/health" in text
    # 점검이 실제 실행문 **앞**에 있어야 의미가 있다.
    # ⚠️ "ros2 launch" 로 찾으면 안 된다 — 머리말 주석이 그 문구를 먼저 쓴다.
    # (2026-09-07 exec -> setsid 로 바뀌었다. stop_bringup.sh 호환용.)
    assert text.index("/health") < text.index("setsid ros2 launch")


def test_서버가_없으면_띄우지_않고_안내한다():
    text = SCRIPT.read_text(encoding="utf-8")
    assert "exit 1" in text
    assert "policy_server.py" in text, "띄우는 명령을 알려줘야 한다"
    assert ".venv-dp" in text, "ACT 용 .venv 로는 이 체크포인트가 안 읽힌다"


def test_기대하는_n_action_steps_가_63_이다():
    """서버가 정하는 값이다. 체크포인트 config 기본값 32 로 뜨면 재생이
    1.07초라 추론 542ms 대비 여유가 절반이 된다(2026-09-07 실측)."""
    assert _var("EXPECT_N_ACTION_STEPS") == "63"


def test_서버의_n_action_steps_가_다르면_경고한다():
    """조용히 다른 청크 길이로 도는 것이 가장 나쁘다 — 실기 수치를 비교할 수
    없게 된다."""
    text = SCRIPT.read_text(encoding="utf-8")
    assert "$EXPECT_N_ACTION_STEPS" in text
    assert "SRV_STEPS" in text


def test_ROS_setup_을_source_할_때는_set_u_를_끈다():
    """⚠️ 2026-09-07 첫 실행이 여기서 죽었다:

        /opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable

    ROS 의 setup.bash 가 미정의 변수를 참조하는데 `set -u` 아래서는 그게 즉시
    오류다. source 구간에서만 끄고 바로 되돌려야 우리 스크립트의 오타는 계속
    잡힌다."""
    text = SCRIPT.read_text(encoding="utf-8")
    assert "set -u" in text, "오타 검출을 위해 set -u 자체는 유지해야 한다"
    off = text.index("set +u")
    src = text.index("source /opt/ros/humble/setup.bash")
    back = text.index("set -u", off)
    last = text.index("source /ros2_ws/install/setup.bash")
    assert off < src, "source 앞에서 꺼야 한다"
    assert last < back, "마지막 source 뒤에 되돌려야 한다"


def test_플래그가_세팅한_변수는_전부_런치로_나간다():
    """⚠️ 2026-09-07: `--vla-only` 가 VLA_ONLY 변수만 세팅하고 런치에는 안
    넘어가고 있었다. 실행하면 아무 경고 없이 평소 모드로 돌았다.

    이 파일이 막으려던 바로 그 종류의 사고인데(인자가 조용히 사라진다),
    새 플래그에 검사를 안 붙여서 그대로 통과했다. 이름 하나씩 적어 두는
    대신 **플래그가 세팅하는 변수 전부**를 기계가 훑게 한다."""
    text = SCRIPT.read_text(encoding="utf-8")
    body = "\n".join(l for l in text.splitlines() if not l.lstrip().startswith("#"))
    # 런치로 나가는 구간 = POLICY_ARGS 구성부터 exec 끝까지. CHECKPOINT 나
    # POLICY_URL 은 exec 줄에 직접 안 적히고 "${POLICY_ARGS[@]}" 로 펼쳐진다.
    exec_block = body[body.index("POLICY_ARGS="):]
    assigned = set(re.findall(r"--[a-z-]+\)\s*(\w+)=", body))
    assert assigned, "플래그 분기를 못 찾았다 — 이 시험의 전제가 깨졌다"
    #: 런치로 안 나가는 것들. FORCE 는 기동 전 정리용, BACKEND 는 POLICY_ARGS
    #: 를 고르는 데만 쓰고, LOG 는 리다이렉트 대상이다.
    INTERNAL = {"FORCE", "BACKEND", "LOG"}
    for var in sorted(assigned - INTERNAL):
        assert "$" + var in exec_block, f"{var} 가 플래그로 세팅되는데 런치로 안 넘어간다"


def test_팀_정지_도구가_끌_수_있게_PGID_를_남긴다():
    """⚠️ 2026-09-07: 이걸 안 해서 `tools/ops/stop_bringup.sh` 가 우리 판을
    못 껐다 — "bringup_now.sh 로 띄운 게 아니면 못 끕니다". run_mission 의
    자동 정리도 같은 이유로 실패해서, 손으로 프로세스를 찾아 죽여야 했다.

    그 도구는 /tmp/bringup.pgid 를 읽어 **프로세스 그룹**에 SIGINT 를 보낸다
    (`kill -INT -- -$PGID`). 그러려면 새 세션의 리더로 띄우고(setsid) 그 PID
    를 남겨야 한다. 우리 스크립트만 팀 도구 밖에 있을 이유가 없다."""
    text = SCRIPT.read_text(encoding="utf-8")
    body = "\n".join(l for l in text.splitlines() if not l.lstrip().startswith("#"))
    assert "setsid ros2 launch" in body, "그룹 리더로 안 띄우면 그룹 kill 이 안 먹는다"
    assert "/tmp/bringup.pgid" in body, "팀 도구가 읽는 경로여야 한다"
    # PID 기록이 기동 **뒤**에 와야 $! 가 그 launch 를 가리킨다.
    assert body.index("setsid ros2 launch") < body.index('echo "$!"')
