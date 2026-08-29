"""Host PC -> 차량(Pi) 링크. 여기가 두 저장소가 만나는 **유일한 자리**다.

## 무엇이 바뀌었나 (2026-08-27 병합)

예전에는 이 파일이 자기만의 전선 규격(`cmd`/`status`/`robot_x`/`robot_y`/
`robot_yaw_deg`/`target_label`)을 정의했고, 그 규격이 `VEHICLE_LINK_PROTOCOL.md`
와 `PI_BRIDGE_TASK.md` 에 문서로만 적혀 있었다. 그런데 Pi 쪽은 2026-08-26 팀
확정으로 **다른 규격**(`state` + 속도 넷)을 쓰기 시작했고, 두 규격이 같은
포트(5005/5006)를 쓰면서 서로 못 알아듣는 상태였다 — Pi 의 `UdpHostLink._parse()`
는 `state` 가 없는 패킷을 전부 버린다.

이제 전선 규격은 **`domain/ports/baseline_ports.py` 를 직접 import** 한다.
문서 두 벌을 손으로 맞추는 대신 **양쪽이 같은 파일을 읽는다** — 그 파일이
스스로 경고하는 "직렬화 규약이 어긋나는 사고"(BoxColor -> Destination 개명 때
두 번 났다는)를 구조적으로 못 나게 만드는 것이 목적이다.

`baseline_ports.py` 와 `domain/task/motion.py` 는 `abc` · `dataclasses` · `math`
만 import 하는 순수 파이썬이라, ROS2 가 없는 이 Windows Host 에서도 그대로
로드된다.

## Host 내부 어휘는 그대로다

`MissionCommand`("go"/"stop"/"yaw+"/"yaw-" + mission.State 이름)는 **남는다.**
`mission.py` 가 계산해서 내놓는 것, `live_map.py` 가 화면에 찍는 것, `run_sim.py`
가 가상 차량을 굴리는 것이 전부 이 어휘다. 바뀐 것은 **전선에 실릴 때의 모양**
뿐이고, 변환은 `UdpVehicleLink` 안에서만 일어난다.

경계를 여기 하나로 몰아둔 이유: 링크 구현체를 바꾸는 것만으로 Host FSM 전체를
건드리지 않고 규격을 바꿀 수 있어야 하기 때문이다. `ConsoleVehicleLink` 와
`run_sim.SimVehicleLink` 는 이 변경의 영향을 전혀 받지 않는다.

## 역할 분담 — 좌표는 전선에 싣지 않는다

Host 가 물체 좌표 · 차량 좌표와 방향 · 경로 계산 · 차량 제어 명령을 전부
소유하고, Pi 는 그 명령을 실행하고 상태를 보고만 한다. 그래서 `HostCommand`
에는 좌표가 하나도 없다 — 로봇 pose 를 "참고용"으로라도 실어 보내면 Pi 가
그것을 읽기 시작하는 순간 역할 분담이 무너진다(`baseline_ports.py` 참고).

예전 규격이 보내던 `robot_x`/`robot_y`/`robot_yaw_deg`/`target_label` 은 그래서
전선에서 **빠진다.** 라벨도 마찬가지다 — 무엇을 집을지는 내려가는 팔이 자기
카메라로 확인한다(`baseline_mission.py` 의 `_OBJECT_WIDTH_MM`). 디버깅에 필요한
값은 `detail` 문자열로만 흘려보낸다.
"""

from __future__ import annotations

import json
import math
import re
import socket
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

# 레포 루트를 경로에 얹어 domain/ 을 직접 쓴다. host/ 는 grippers 레포의
# 하위 디렉터리이므로 parent 하나면 된다.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from domain.ports.baseline_ports import HostCommand, MissionState, Report
from domain.task.motion import AGREED_LINEAR_MPS, AGREED_ROTATION_RAD_S
# Pi 가 `fix` 에 싣는 동작 이름. 문자열을 다시 적지 않고 정본에서 가져온다
# (sysy009 설계, 2026-08-28). 여기 문자열을 복사해 두면 Pi 가 이름을 바꾸는
# 순간 Host 가 조용히 못 알아듣는다 — `tools/check_domain_sync.py` 가 이
# 파일의 동기화를 검사한다.
from domain.task.corrections import (ADVANCE as _FIX_ADVANCE,
                                     REACQUIRE as _FIX_REACQUIRE,
                                     RETREAT as _FIX_RETREAT,
                                     ROTATE as _FIX_ROTATE,
                                     WAIT as _FIX_WAIT)


@dataclass
class MissionCommand:
    """Host 내부 표현. **전선 규격이 아니다** — 전선으로 나가는 모양은
    `HostCommand` 이고, 변환은 `UdpVehicleLink.send()` 가 한다.

    `robot_*` 와 `target_label` 은 화면 표시와 로그용으로 남아 있다.
    """

    cmd: str                           # "go" | "back" | "left" | "right"
                                       #  | "stop" | "yaw+" | "yaw-"
    status: str                        # 지금 미션 단계 (mission.State 이름)
    robot_x: float
    robot_y: float
    robot_yaw_deg: float
    target_label: Optional[str] = None
    fresh: bool = True
    t: float = field(default_factory=time.monotonic)


# ---------------------------------------------------------------------------
# 어휘 대응표 — 이 두 표가 병합의 실체다
# ---------------------------------------------------------------------------

# mission.State 이름 -> MissionState (Pi 가 아는 이름)
#
# SEARCH_TARGET -> IDLE : Host 가 다음 기물을 고르는 동안 Pi 는 할 일이 없다.
#     MissionState 에 SEARCH 를 새로 넣지 않는 이유는, 넣어봤자 Pi 쪽
#     BaselineIdleState 가 하는 일과 똑같기 때문이다 — 상태를 늘리면 양쪽이
#     맞춰야 할 이름만 하나 더 는다.
# FACE_BOX -> CARRY : 아직 물체를 들고 제자리 회전 중이다. `BaselineCarryState`
#     가 CARRY/APPROACH_BOX 를 둘 다 받는다.
# NUDGE_BOX -> APPROACH_BOX : 바구니 앞 미세전진. Pi 쪽 APPROACH_BOX 의 의미와
#     정확히 같다.
# PLACE -> INSERT : 이름만 다르고 같은 동작이다.
_STATE_TO_PI = {
    "SEARCH_TARGET":  MissionState.IDLE,
    "APPROACH_PIECE": MissionState.APPROACH,
    "GRASP":          MissionState.GRASP,
    # GRASP_ALIGN -> APPROACH : Host 가 GRASP_BLOCKED 를 받고 차를 다시 세우는
    #     중이다. 이때 Pi 를 GRASP 로 두면 매 사이클 파지 판정(1.7초짜리)을
    #     다시 돌려서 차가 움직이는 동안 계속 BLOCKED 를 뱉는다. APPROACH 는
    #     Host 속도대로 주행만 하고, 다시 GRASP 가 올 때 한 번만 판정한다 —
    #     "관측 -> 소이동 -> 재관측" 폐루프가 성립하는 것이 이 매핑 덕이다.
    "GRASP_ALIGN":    MissionState.APPROACH,
    "CARRY_TO_DEST":  MissionState.CARRY,
    "FACE_BOX":       MissionState.CARRY,
    "NUDGE_BOX":      MissionState.APPROACH_BOX,
    "PLACE":          MissionState.INSERT,
    "DONE":           MissionState.DONE,
}

# Pi 가 돌려주는 Report -> mission.py 가 기다리는 옛 문자열
#
# mission.py 는 "GRASP_DONE" 과 "PLACE_DONE" 두 개만 본다. 나머지는 여기서
# 흡수하되 **버리지 않는다** — `last_report` 에 남기고 경고를 찍는다.
# GRASP_BLOCKED / INSERT_BLOCKED 에 실제로 대응하는 로직(수정된 명령을 다시
# 내는 것)은 다음 단계에서 mission.py 에 들어간다. 지금은 그 신호가 오고
# 있다는 사실이 보이게만 해 둔다.
_BLOCKING_REPORTS = {
    Report.GRASP_BLOCKED, Report.GRASP_CENTERING, Report.INSERT_BLOCKED,
}

# 한 번 나오면 mission.py 의 상태 전이를 좌우하는 값들. `poll_status()` 에서
# 다른 보고에 덮이면 안 된다.
_TERMINAL = {"GRASP_DONE", "PLACE_DONE", "FAILED"}


# ---------------------------------------------------------------------------
# GRASP_BLOCKED 보정 요청 — Pi 의 한글 사유를 mission.py 가 쓸 값으로 옮긴다
# ---------------------------------------------------------------------------
#
# Pi 는 "왜 못 내려가는지"를 사람이 읽는 문장으로 보낸다
# (`preconditions.PreconditionReport.detail`, `grasp_alignment.judge` 의 reason).
# 그중 **Host 가 차를 다시 세워서 고칠 수 있는 것은 세 가지**뿐이고, 나머지는
# Host 가 아무리 움직여도 안 풀린다(E-STOP·미실측 상수·그리퍼가 안 비었음).
#
# ⚠️ 문자열 매칭이라 깨지기 쉽다. Pi 쪽 `grasp_alignment.py` 의 문구를 누가
#    고치면 여기가 조용히 실패한다 — Host 가 UNFIXABLE 로 보고 대상을 포기해
#    버린다. **제대로 된 해법은 `baseline_ports.py` 에 보정 종류 상수를 두고
#    Pi 가 그 코드를 `detail` 과 함께 보내는 것**이고, 그건 양쪽 합의가
#    필요해서 지금은 안 했다. 그때까지의 임시 다리다.
#    아래 `_CORRECTION_KEYS` 의 문구는 `grasp_alignment.judge()` 의 리터럴을
#    그대로 옮긴 것이다 — 그 파일을 고치면 여기도 같이 고칠 것.

BACK_OFF = "BACK_OFF"      # 물체가 턱 선보다 가깝다 -> 뒤로
CREEP_IN = "CREEP_IN"      # 물체가 전진 거리 밖이다 -> 앞으로
RE_AIM = "RE_AIM"          # 물체가 턱 폭 밖이다 -> 좌우로 다시 겨눔
SHIFT = "SHIFT"            # 좌우로 밀렸다 -> 메카넘 횡이동. lateral_mm 부호로 방향
WAIT = "WAIT"              # 기다리면 풀린다. 움직이면 오히려 나빠진다
UNFIXABLE = "UNFIXABLE"    # Host 가 움직여서 고칠 수 있는 게 아니다

_CORRECTION_KEYS = (
    ("후진 필요", BACK_OFF),
    ("재직진 필요", CREEP_IN),
    ("재회전 필요", RE_AIM),
    # servo 1 이 거부했거나 팔 길이가 미실측이라 Pi 가 못 고치는 좌우 치우침도
    # 결국 차를 다시 겨누는 것으로 푼다.
    ("재회전", RE_AIM),
    ("Pi가 못 고친다", RE_AIM),
)

_LATERAL_RE = re.compile(r"좌우\s*([+-]?\d+(?:\.\d+)?)\s*mm")

# INSERT_BLOCKED 의 문장에서 숫자를 되꺼낸다. GRASP 쪽과 같은 임시 다리다 —
# 제대로 하려면 Pi 가 보정을 구조화해서 보내야 하고, 그건 양쪽 합의가 필요하다.
# 아래 문구는 Pi `preconditions.check_insert()` 의 리터럴을 그대로 옮긴 것이니
# 그 파일을 고치면 여기도 같이 고칠 것.
#
#   "바구니가 멀다 (라이다 0.351m > 0.155m) / 좌우로 밀려 있다 (-79mm > ±70mm)"
#
# ⚠️ "라이다 판독이 하한보다 가깝다" 쪽은 일부러 안 잡는다. 그 경우는
# 테두리를 넘겨보고 있을 수 있어 판독 자체를 믿으면 안 되고, 더 붙이면
# 상황이 나빠지기만 한다(Pi corrections.from_insert 주석).
_BASKET_DIST_RE = re.compile(r"바구니가 멀다 \(라이다\s*([\d.]+)\s*m")
_BASKET_LATERAL_RE = re.compile(r"좌우로 밀려 있다\s*\(([+-]?[\d.]+)\s*mm")


@dataclass(frozen=True)
class BasketFix:
    """Pi 가 INSERT_BLOCKED 로 알려 준, 바구니 앞에서 고쳐야 할 양.

    `distance_m` 은 **라이다 판독**이고 차체 기준 거리가 아니다. `lateral_m`
    은 로봇 기준 좌우로 +가 왼쪽이며, 바구니 중심이 어디 있는지를 뜻한다 —
    즉 로봇이 그 부호 방향으로 가야 가운데에 선다. 못 읽은 값은 None 이다.
    """

    distance_m: Optional[float] = None
    lateral_m: Optional[float] = None
    #: Pi 가 직접 계산한 전후 오차(m, +면 더 가야 한다). 구조화된 `fix` 에서
    #: 나오며, 있으면 `distance_m` 보다 이쪽이 우선이다 — 라이다 판독에서
    #: 목표를 빼는 계산을 Host 가 다시 하지 않아도 되고, 두 목표값이 갈라질
    #: 여지도 없어진다.
    forward_m: Optional[float] = None


def basket_fix_from_fix(fix) -> Optional[BasketFix]:
    """Pi 의 구조화된 `fix` -> BasketFix. 못 읽으면 None.

    ⚠️ **너무 가까운 경우가 여기서만 살아난다.** 아래 문장 파서는 "바구니가
    멀다"만 잡고 "하한보다 가깝다"는 일부러 안 잡는다 — 그래서 바구니에
    너무 붙어 서면 Host 가 아무 보정도 못 받고 그 자리에 영원히 서 있었다.
    Pi 의 `corrections.from_insert` 는 그 경우에 `retreat` 를 정확히 계산해
    보내고 있었고, 읽기만 하면 된다.

    `forward_m` 은 Pi 가 이미 계산해 둔 오차(판독 - 목표)다. 그대로 실어
    보낸다 — 거리로 되돌렸다가 Host 가 자기 목표로 다시 빼면 같은 계산을 두
    번 하는 셈이고, 두 목표값이 갈라지는 순간 조용히 어긋난다.

    이 모듈은 `mission_config` 를 import 하지 않는다. 여기는 전선 계층이고,
    무엇을 목표로 삼을지는 미션의 판단이다."""
    if not isinstance(fix, dict):
        return None
    action = fix.get("action")
    forward = fix.get("forward_m")
    lateral = fix.get("lateral_m")
    if action in ("advance", "retreat") and isinstance(forward, (int, float)):
        return BasketFix(forward_m=float(forward))
    if action == "rotate" and isinstance(lateral, (int, float)):
        return BasketFix(lateral_m=float(lateral))
    return None


def parse_basket_fix(detail: str) -> Optional[BasketFix]:
    """INSERT_BLOCKED 의 detail -> BasketFix. 읽을 숫자가 없으면 None."""
    d = _BASKET_DIST_RE.search(detail or "")
    lat = _BASKET_LATERAL_RE.search(detail or "")
    if d is None and lat is None:
        return None
    return BasketFix(
        distance_m=float(d.group(1)) if d else None,
        lateral_m=float(lat.group(1)) / 1000.0 if lat else None,
    )


@dataclass(frozen=True)
class GraspCorrection:
    """Pi 가 요청한 재정렬. `kind` 는 위 여섯 상수 중 하나다.

    `lateral_mm` 은 **+ 가 왼쪽**이다(Pi `TargetObservation.lateral_m` 규약).
    RE_AIM/SHIFT 일 때 방향이 여기서 나온다 — 부호를 못 읽으면 어느 쪽으로
    갈지 모르므로 보정하지 않는 편이 낫다(반대로 가면 더 나빠진다).
    """

    kind: str
    detail: str = ""
    lateral_mm: Optional[float] = None
    #: Pi 가 `fix` 로 준 실제 오차량. 산문 파싱으로는 못 얻는 값이라 그때는 None.
    #: **크기를 그대로 쓰지 말 것** — INSERT 의 forward 는 라이다 판독 기준이라
    #: Pi 가 "줄어드는 방향으로 조금씩 움직이며 다시 물어라"라고 못박았다
    #: (`domain/task/corrections.py::from_insert`). 부호를 믿는 데 쓴다.
    forward_mm: Optional[float] = None
    yaw_deg: Optional[float] = None

    @property
    def actionable(self) -> bool:
        """Host 가 차를 움직여 고칠 수 있는가."""
        if self.kind == UNFIXABLE:
            return False
        if self.kind == WAIT:
            return False   # 움직여서 고치는 게 아니다. 기다린다
        if self.kind in (RE_AIM, SHIFT) and self.lateral_mm is None:
            return False   # 방향을 모른다 — 찍어서 움직이지 않는다
        return True


def correction_from_fix(fix, *, insert: bool) -> Optional[GraspCorrection]:
    """Pi 의 `fix` 필드 -> `GraspCorrection`. 모르는 action 이면 None.

    **이것이 정식 경로다.** `classify_correction` 은 `fix` 가 없는 보고를 위한
    폴백일 뿐이다 — Pi 가 판정을 내린 자리에서 같이 만든 수치를 받는 쪽이,
    사람이 읽으라고 쓴 문장을 정규식으로 뜯는 것보다 언제나 낫다.

    `insert` 로 갈리는 곳이 하나 있다. Pi 의 `ROTATE` + `lateral_m` 은 "좌우로
    이만큼 어긋나 있다"는 뜻이고 **없애는 경로는 Host 가 정한다**
    (`corrections.py` 의 설계 원칙). 기물 앞에서는 회전이 맞다 — 턱을 물체 쪽으로
    돌리는 것이다. 하지만 바구니 앞에서는 회전하면 거리와 yaw 가 같이 틀어져
    여섯 조건을 동시에 흔들므로 **메카넘 횡이동**으로 없앤다.

    ⚠️ `REACQUIRE` 는 UNFIXABLE 이다. "판정할 수 없으니 다시 보이게 세워
    달라"에는 **방향이 없어서**, 여기서 움직이면 찍는 것이다. 방향을 아는
    경우에 Pi 는 REACQUIRE 가 아니라 RETREAT/ADVANCE 를 보낸다 — 2026-08-29
    에 "뎁스캠이 목표를 못 봄"이 그렇게 바뀌었다(`from_grasp_precondition`).

    이 함수는 sysy009 가 grippers 저장소에 쓴 것과 같은 설계다(2026-08-28,
    커밋 48b782f). 같은 일을 두 벌로 두지 않으려고 그쪽에 맞췄다.
    """
    if not isinstance(fix, dict):
        return None
    action = fix.get("action")
    lat_mm = float(fix.get("lateral_m", 0.0) or 0.0) * 1000.0
    fwd_mm = float(fix.get("forward_m", 0.0) or 0.0) * 1000.0
    yaw_deg = math.degrees(float(fix.get("yaw_rad", 0.0) or 0.0))
    detail = (f"fix={action} 좌우 {lat_mm:+.0f}mm 전후 {fwd_mm:+.0f}mm "
              f"yaw {yaw_deg:+.1f}도")

    if action == _FIX_WAIT:
        return GraspCorrection(WAIT, detail, lat_mm, fwd_mm, yaw_deg)
    if action == _FIX_ADVANCE:
        return GraspCorrection(CREEP_IN, detail, lat_mm, fwd_mm, yaw_deg)
    if action == _FIX_RETREAT:
        return GraspCorrection(BACK_OFF, detail, lat_mm, fwd_mm, yaw_deg)
    if action == _FIX_REACQUIRE:
        return GraspCorrection(UNFIXABLE, detail, lat_mm, fwd_mm, yaw_deg)
    if action == _FIX_ROTATE:
        if abs(yaw_deg) > 0.0:
            return GraspCorrection(RE_AIM, detail, yaw_deg, fwd_mm, yaw_deg)
        if insert:
            return GraspCorrection(SHIFT, detail, lat_mm, fwd_mm, yaw_deg)
        return GraspCorrection(RE_AIM, detail, lat_mm, fwd_mm, yaw_deg)
    return None


def classify_correction(detail: str) -> GraspCorrection:
    """Pi 의 `detail` 문장 -> `GraspCorrection`. 모르면 UNFIXABLE.

    **폴백이다.** `fix` 가 실려 오면 `correction_from_fix` 가 정식 경로이고,
    이 함수는 그것이 없는 옛 Pi 빌드와 붙기 위해서만 남는다."""
    m = _LATERAL_RE.search(detail or "")
    lateral = float(m.group(1)) if m else None
    for key, kind in _CORRECTION_KEYS:
        if key in (detail or ""):
            return GraspCorrection(kind, detail, lateral)
    return GraspCorrection(UNFIXABLE, detail, lateral)

# 같은 경고를 이 간격보다 자주 찍지 않는다. REJECTED 는 Pi 워치독이 발동할
# 때마다 나오는데, Host 주기가 워치독 한계보다 느리면 초당 여러 번이 된다 —
# 그대로 찍으면 콘솔이 묻히고, 진짜 인코더 버그가 났을 때 그 한 줄이 안 보인다.
_WARN_REPEAT_SEC = 5.0


def encode(cmd: MissionCommand) -> HostCommand:
    """Host 내부 명령 -> 전선에 실릴 `HostCommand`.

    네 가지 동작이 속도 넷으로 어떻게 옮겨지는가:

        go     -> linear_x = +AGREED_LINEAR_MPS
        back   -> linear_x = -AGREED_LINEAR_MPS
        left   -> linear_y = +AGREED_LINEAR_MPS       (메카넘 횡이동)
        right  -> linear_y = -AGREED_LINEAR_MPS
        stop   -> stop = True            (나머지를 무시하는 가장 센 명령)
        yaw+   -> angular_z = +AGREED_ROTATION_RAD_S   (반시계)
        yaw-   -> angular_z = -AGREED_ROTATION_RAD_S   (시계)

    Host 는 회전과 병진을 **절대 섞지 않는다** — `_send_drive()` 가 셋 중
    하나만 고르므로, Pi 의 `resolve_motion()` 이 "제자리회전에 병진이 섞였다"
    로 거부하는 경로에 걸릴 일이 없다.

    속도 크기는 `domain/task/motion.py` 의 합의값을 그대로 가져온다. 여기에
    숫자를 다시 적으면 두 벌이 되고, 갈라지는 순간 Pi 가 조용히 잘라낸 값으로
    돌아 Host 의 경로 계산과 실제 주행이 어긋난다. 안전 한계 자체는 여전히
    Pi 가 집행한다 — Host 가 무엇을 보내든 바퀴를 돌리는 쪽이 자른다.
    """
    state = _STATE_TO_PI.get(cmd.status)
    if state is None:
        # 모르는 상태 이름을 추측해서 보내지 않는다. 정지가 안전하다.
        return HostCommand(state=MissionState.IDLE, stop=True)

    if cmd.cmd == "go":
        return HostCommand(state=state, linear_x=AGREED_LINEAR_MPS)
    if cmd.cmd == "back":
        # 예전 4어휘(go/stop/yaw+/yaw-)에는 후진이 없었다. 속도 형식으로
        # 바뀌면서 부호만 뒤집으면 되는 것이 됐다 — Pi 의 `_clamp` 가
        # copysign 이라 음수 크기를 그대로 잘라 준다. GRASP_ALIGN 이 쓴다.
        return HostCommand(state=state, linear_x=-AGREED_LINEAR_MPS)
    if cmd.cmd in ("left", "right"):
        # 메카넘 횡이동. 바구니 앞 좌우 정렬에만 쓴다 — 바구니와 나란한 채
        # 옆으로 밀려 있으면 거리도 yaw 도 정상으로 나오는데 물체는 바구니
        # 밖에 떨어진다(Pi basket_lidar_align.face_lateral_offset_m 주석).
        # 돌아서 고치려 하면 거리와 yaw 가 같이 틀어지므로 옆으로 간다.
        #
        # 부호는 ROS 규약 그대로 +y = 왼쪽이다. Pi 의 실기 확인(2026-08-28)
        # 으로 linear_y 는 0.03 m/s 까지 실제로 돈다.
        sign = 1.0 if cmd.cmd == "left" else -1.0
        return HostCommand(state=state, linear_y=sign * AGREED_LINEAR_MPS)
    if cmd.cmd == "yaw+":
        return HostCommand(state=state, angular_z=AGREED_ROTATION_RAD_S)
    if cmd.cmd == "yaw-":
        return HostCommand(state=state, angular_z=-AGREED_ROTATION_RAD_S)
    # "stop" 과 모르는 값 전부 — 모르면 정지한다.
    return HostCommand(state=state, stop=True)


class VehicleLink:
    """전송 어댑터의 추상 인터페이스."""

    #: 마지막으로 받은 Pi 보고 (report, state, detail). 아직 없으면 None.
    last_report: Optional[tuple[str, str, str]] = None

    #: 구동계가 명령을 받아 갈 상태가 아니라고 Pi 가 알려 온 마지막 사유.
    #: 한 번 뜨면 지우지 않는다 — 실행이 끝난 뒤에도 "그 실행에서 구동계
    #: 경보가 있었다"가 남아 있어야 로그를 읽는 사람이 원인을 짚는다.
    base_alarm: Optional[str] = None

    #: 마지막 INSERT_BLOCKED 가 요청한 재정렬. GRASP 쪽과 칸을 나눠 두는
    #: 이유: 한 칸에 두면 GRASP 의 `take_correction()` 이 바구니 보정을 집어
    #: 가서 기물 앞에서 엉뚱하게 움직인다(sysy009 도 같은 이유로 나눴다).
    last_insert_correction: Optional[GraspCorrection] = None

    #: 마지막 GRASP_BLOCKED 가 요청한 재정렬. mission.py 의 GRASP 가 읽고
    #: GRASP_ALIGN 으로 넘어간다. **읽은 쪽이 지운다**(take_correction) —
    #: 한 번의 요청으로 한 번만 움직이기 위해서다.
    last_correction: Optional[GraspCorrection] = None

    #: 마지막 INSERT_BLOCKED 가 알려 준 바구니 보정량. **읽은 쪽이 지운다**.
    last_basket_fix: Optional[BasketFix] = None

    def take_basket_fix(self) -> Optional[BasketFix]:
        """마지막 바구니 보정량을 꺼내고 지운다.

        지우는 이유는 GRASP 보정과 같다 — 한 번 읽은 값으로 두 번 움직이면
        같은 오차를 두 배로 고치게 된다. 다음 판정은 다음 보고를 기다린다."""
        f, self.last_basket_fix = self.last_basket_fix, None
        return f

    def take_correction(self) -> Optional[GraspCorrection]:
        """보정 요청을 **소비한다.** 없으면 None.

        지우지 않고 두면 Host 가 한 번의 BLOCKED 로 계속 움직인다 — Pi 는
        재관측할 때마다 새로 보고하므로, 매 요청당 한 걸음이 맞다."""
        c, self.last_correction = self.last_correction, None
        return c

    def send(self, cmd: MissionCommand) -> None:
        raise NotImplementedError

    def poll_status(self) -> str:
        """차량이 보고하는 상태.

        "IDLE" | "BUSY" | "GRASP_DONE" | "PLACE_DONE" | "FAILED" 중 하나.
        """
        raise NotImplementedError


class ConsoleVehicleLink(VehicleLink):
    """전송 없이 콘솔에만 찍는다. 차량 없이 mission.py 로직만 시험할 때 쓴다.

    GRASP/PLACE 명령을 보내는 즉시 완료된 것으로 치고 다음 상태로 넘어간다.
    """

    def __init__(self, auto_complete: bool = True) -> None:
        self._auto_complete = auto_complete
        self._pending_done: Optional[str] = None

    def send(self, cmd: MissionCommand) -> None:
        extra = f"target={cmd.target_label}" if cmd.target_label else ""
        print(f"\r[vehicle_link] {cmd.cmd:5s} [{cmd.status:14s}] "
              f"robot=({cmd.robot_x:6.3f},{cmd.robot_y:6.3f},{cmd.robot_yaw_deg:6.1f}°) "
              f"{extra}   ",
              end="", flush=True)
        if self._auto_complete and cmd.status in ("GRASP", "PLACE"):
            self._pending_done = f"{cmd.status}_DONE"

    def poll_status(self) -> str:
        if self._pending_done:
            status, self._pending_done = self._pending_done, None
            return status
        return "IDLE"


class UdpVehicleLink(VehicleLink):
    """실제 차량(Pi)과 UDP+JSON 으로 말한다. 명령 5005 송신 / 보고 5006 수신.

    ## 왜 최신 것만 보는가

    이 링크가 실어 나르는 것은 **그 순간의 속도 명령**이라 오래된 패킷은
    쓸모가 없다. TCP 로 재전송을 기다리는 것보다 다음 사이클 것을 쓰는 쪽이
    항상 낫다. 그래서 수신도 큐를 쌓지 않고 마지막 것만 본다.

    ## 안 닿아도 예외를 내지 않는다

    UDP 라 Pi 가 아직 안 켜져 있어도 `send()` 는 조용히 나간다. 링크가
    끊긴 것을 판정하는 것은 **받는 쪽(Pi)의 워치독**이다 — Host 가 말을
    멈추면 차량도 멈춘다.
    """

    def __init__(self, pi_ip: str, cmd_port: int = 5005, status_port: int = 5006,
                 bind_ip: str = "0.0.0.0", verbose: bool = True) -> None:
        self.pi_ip = pi_ip
        self.cmd_port = cmd_port
        self.verbose = verbose
        self.last_report: Optional[tuple[str, str, str]] = None
        self.last_insert_correction: Optional[GraspCorrection] = None
        self.base_alarm: Optional[str] = None
        self._warn_seen: dict[str, tuple[float, int]] = {}

        # INSERT 는 두 번 보고된다: INSERT_DONE(또는 INSERT_FAILED) 다음에
        # 반드시 IDLE_DONE 이 온다(baseline_mission.BaselineInsertState).
        # 팔이 접히기 전에 차를 움직이면 안 되므로 **IDLE_DONE 을 완료 신호로
        # 쓰고**, 그 직전 결과를 여기 기억해 성패를 가른다.
        self._insert_ok: Optional[bool] = None

        self._send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._recv_sock.setblocking(False)
        self._recv_sock.bind((bind_ip, status_port))

    # --- 송신 ---------------------------------------------------------

    def send(self, cmd: MissionCommand) -> None:
        host_cmd = encode(cmd)
        payload = json.dumps({
            "state":     host_cmd.state,
            "linear_x":  host_cmd.linear_x,
            "linear_y":  host_cmd.linear_y,
            "angular_z": host_cmd.angular_z,
            "stop":      host_cmd.stop,
        }).encode("utf-8")
        try:
            self._send_sock.sendto(payload, (self.pi_ip, self.cmd_port))
        except OSError as exc:
            # 네트워크가 잠깐 끊겨도 미션 루프는 안 죽어야 한다 — 다음
            # 사이클에 다시 시도된다.
            self._warn(f"전송 실패 — {exc}")

    # --- 수신 ---------------------------------------------------------

    def poll_status(self) -> str:
        """논블로킹. 그 사이 쌓인 보고를 전부 읽되 **완료 신호는 놓치지 않는다.**

        여러 개가 와 있으면 마지막 것만 쓰는 것이 이 프로젝트의 관례지만,
        보고는 속도 명령과 달리 **사건**이라 덮어쓰면 안 된다 — INSERT_DONE
        과 IDLE_DONE 이 한 사이클 안에 같이 도착하는 일이 실제로 생긴다.

        ⚠️ 여기서 한 번 더 나눈다: 완료/실패(`_TERMINAL`)는 **그 밖의 값에
        절대 덮이지 않는다.** 그냥 "마지막 것"을 돌려주면 이런 순서에서
        신호가 통째로 사라진다:

            GRASP_DONE  ->  STATE  ->  REJECTED     (한 사이클에 같이 도착)
                            ^^^^^^^^^^^^^^^^^^ 이게 덮어써서 "BUSY" 가 나감

        mission.py 의 GRASP 는 `poll_status() == "GRASP_DONE"` 한 번을 보고
        전이하는데, 그 한 번을 놓치면 **영원히 GRASP 에 머문다.** 그리고
        REJECTED 는 워치독이 발동할 때마다 나오므로(Host 주기가 Pi 워치독
        한계보다 느리면 초당 여러 번) 이 순서는 드문 사고가 아니라 상시
        상황이다.
        """
        terminal = None      # 완료/실패 — 최우선
        other = "IDLE"       # BUSY/IDLE — 참고용
        while True:
            try:
                data, _addr = self._recv_sock.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError as exc:
                self._warn(f"수신 오류 — {exc}")
                break
            translated = self._handle(data)
            if translated is None:
                continue
            if translated in _TERMINAL:
                terminal = translated
            else:
                other = translated
        return terminal if terminal is not None else other

    def _handle(self, data: bytes) -> Optional[str]:
        """보고 하나를 옛 문자열로 옮긴다. 옮길 게 없으면 None."""
        try:
            msg = json.loads(data)
        except json.JSONDecodeError:
            self._warn("Pi 보고 파싱 실패 — 버림")
            return None

        report = msg.get("report")
        state = msg.get("state", "")
        detail = msg.get("detail", "")
        if not isinstance(report, str):
            self._warn("Pi 보고에 report 가 없다 — 버림")
            return None
        self.last_report = (report, state, detail)

        if report == Report.GRASP_DONE:
            return "GRASP_DONE"
        if report == Report.GRASP_FAILED:
            self._warn(f"파지 실패 — {detail}")
            return "FAILED"

        # INSERT: 결과를 기억해 두고 IDLE_DONE 에서 판정한다.
        if report == Report.INSERT_DONE:
            self._insert_ok = True
            return "BUSY"
        if report == Report.INSERT_FAILED:
            self._insert_ok = False
            self._warn(f"투하 실패 — {detail}")
            return "BUSY"
        if report == Report.IDLE_DONE:
            ok, self._insert_ok = self._insert_ok, None
            if ok is False:
                return "FAILED"
            return "PLACE_DONE"

        if report == Report.BASE_UNRESPONSIVE:
            # 구동계가 명령을 받아 갈 상태가 아니다 — **소프트웨어로는 차를
            # 세울 수 없다.** 2026-08-28에 정지를 836회 보내고도 못 세운
            # 상태가 이것이다. 미션을 자동으로 중단하지는 않는다: 명령이 안
            # 닿는 상태라 중단해도 차가 서지 않고, 복구되면 그대로 이어가는
            # 편이 낫다. 대신 사람이 못 지나치게 크게 찍는다.
            self.base_alarm = detail
            self._warn(f"\n{'=' * 64}\n"
                       f"🚨 구동계 이상 [{state}] {detail}\n"
                       f"   소프트웨어 정지가 바퀴까지 닿지 않을 수 있습니다.\n"
                       f"   ▶ 차체 전원 스위치를 쓰세요. 그게 진짜 비상정지입니다.\n"
                       f"{'=' * 64}")
            return "BUSY"

        # `fix` 가 있으면 그것이 정본이다. 없을 때만 문장을 뜯는다 — 폴백을
        # 남겨 두는 것은 옛 Pi 빌드와도 붙기 위해서다.
        fix = msg.get("fix")
        if report in (Report.GRASP_BLOCKED, Report.GRASP_CENTERING):
            self.last_correction = (correction_from_fix(fix, insert=False)
                                    or classify_correction(detail))
        elif report == Report.INSERT_BLOCKED:
            self.last_insert_correction = (correction_from_fix(fix, insert=True)
                                           or classify_correction(detail))
            # 바구니 폐루프는 예산·데드밴드·정체 감시를 들고 있어서 별도
            # 경로를 그대로 쓴다. 입력만 구조화된 값에서 받는다.
            basket = basket_fix_from_fix(fix) or parse_basket_fix(detail)
            if basket is not None:
                self.last_basket_fix = basket

        if report in _BLOCKING_REPORTS:
            # Pi 가 "조건이 안 맞는다, 수정된 명령을 달라"고 말하는 중이다.
            # 지금 Host 에는 그 요청에 응답하는 로직이 없다 — 기다리기만 한다.
            # 다음 단계에서 mission.py 에 GRASP_ALIGN 을 넣어 대응한다.
            self._warn(f"Pi 가 대기 중: {report} [{state}] {detail}")
            return "BUSY"

        if report == Report.REJECTED:
            # Pi 가 명령 자체를 실행할 수 없다고 되돌려줬다. 링크 문제가
            # 아니라 **Host 인코더 버그** 신호다 — 조용히 넘기면 안 된다.
            self._warn(f"⚠️ Pi 가 명령을 거부했다: [{state}] {detail}")
            return "BUSY"

        if report == Report.GRASP_READY or report == Report.INSERT_READY:
            return "BUSY"
        if report == Report.STATE:
            return "IDLE" if state == MissionState.IDLE else "BUSY"

        self._warn(f"모르는 Pi 보고: {report} [{state}] {detail}")
        return None

    def close(self) -> None:
        self._send_sock.close()
        self._recv_sock.close()

    def _warn(self, message: str) -> None:
        """같은 문구는 `_WARN_REPEAT_SEC` 마다 한 번만, 그동안 몇 번 더 났는지와
        함께 찍는다. 눌러 버리지 않고 **세어서 보여주는** 이유: 워치독 발동이
        상시가 됐다는 사실 자체가 진단 정보이기 때문이다."""
        if not self.verbose:
            return
        now = time.monotonic()
        last, count = self._warn_seen.get(message, (0.0, 0))
        if now - last < _WARN_REPEAT_SEC:
            self._warn_seen[message] = (last, count + 1)
            return
        suffix = f"  (직전 {_WARN_REPEAT_SEC:.0f}초간 {count}회 더)" if count else ""
        print(f"\n[vehicle_link] {message}{suffix}")
        self._warn_seen[message] = (now, 0)
