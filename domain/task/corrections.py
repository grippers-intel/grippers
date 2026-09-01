"""Host가 **기계적으로 실행할 수 있는** 보정 요구 (2026-08-26).

## 왜 필요한가

팀 확정에서 Pi의 임무 중 하나는 조건 미충족 시 보고하고 **수정된 명령을
기다리는** 것이다. Pi는 이미 정확한 수치를 안다 — "좌우 +95mm, 한계 ±76mm"
까지 계산해 놓는다. 그런데 그것을 산문으로만 보내면 Host는 읽을 수가 없고,
결국 재시도밖에 못 한다. **재시도는 같은 자리에서 같은 이유로 또 막힌다.**

2026-08-26 대조에서 확인된 것도 같다 — Host 어휘에는 `FAILED`만 있고
"이렇게 고쳐 달라"가 없다. 그 빈칸을 메우는 것이 이 모듈이다.

## 설계 원칙 — Pi는 "무엇이 틀렸는지"만 말한다

Pi는 경로를 계산하지 않는다(그건 Host 소유다). 그래서 여기서 내는 것은
"이만큼 돌려라"가 아니라 **"이만큼 어긋나 있다"**이다. 그것을 어떤 경로로
없앨지는 Host가 정한다.

    action      Host가 할 일의 종류
    lateral_m   좌우 오차 (+는 물체가 왼쪽 / 로봇이 오른쪽으로 치우침)
    forward_m   전후 오차 (+면 더 가야 한다, -면 물러나야 한다)
    yaw_rad     방위 오차

값이 0이면 그 축은 문제가 아니라는 뜻이다.

## 산문을 파싱하지 않는다

보정값은 **판정을 내린 자리에서 같이 만든다.** 문장을 만들고 그걸 다시
정규식으로 뜯는 방식은 문구를 고칠 때마다 조용히 깨진다 — 이 저장소가
"모르면 실패"를 지키는 것과 같은 이유로, 아는 값을 그대로 들고 다닌다.
"""

from dataclasses import dataclass, asdict

# Host가 실행할 수 있는 동작. 여기 없는 것은 Pi가 요구하지 않는다.
ROTATE = "rotate"      # 방위를 다시 맞춰라 (좌우 오차를 없애는 쪽)
ADVANCE = "advance"    # 더 전진하라
RETREAT = "retreat"    # 물러나라
WAIT = "wait"          # Pi가 스스로 고치는 중 — 명령을 바꾸지 말고 기다려라
REACQUIRE = "reacquire"  # Pi가 판정할 수 없다 — 다시 보이게 세워 달라

ACTIONS = (ROTATE, ADVANCE, RETREAT, WAIT, REACQUIRE)


@dataclass(frozen=True)
class Correction:
    """Host가 받는 보정 요구. 보고 JSON의 `fix` 필드가 된다."""

    action: str
    lateral_m: float = 0.0
    forward_m: float = 0.0
    yaw_rad: float = 0.0

    def __post_init__(self):
        if self.action not in ACTIONS:
            raise ValueError(f"모르는 보정 동작: {self.action}")

    def as_dict(self) -> dict:
        return asdict(self)


def from_alignment(verdict, jaw_line_m: float | None = None) -> Correction | None:
    """GRASP 정렬 판정 -> 보정 요구. 고칠 것이 없으면 None.

    `HOST_CORRECTION`이 나오는 세 경우를 축으로 갈라 준다 — 좌우로 벗어난
    것과 전후로 벗어난 것은 Host가 해야 할 일이 다르다."""
    from domain.task import grasp_alignment as ga

    if verdict.action == ga.READY:
        return None
    if verdict.action == ga.PI_CENTER:
        # Pi가 servo 1로 고치는 중이다. Host가 이때 차를 움직이면 Pi의
        # 보정과 겹쳐 오히려 어긋난다.
        return Correction(WAIT, lateral_m=verdict.lateral_error_m)
    if verdict.action == ga.UNKNOWN:
        return Correction(REACQUIRE)

    # HOST_CORRECTION — 전후가 원인이면 forward, 아니면 좌우다.
    if verdict.forward_error_m:
        action = ADVANCE if verdict.forward_error_m > 0 else RETREAT
        return Correction(action, forward_m=verdict.forward_error_m)
    return Correction(ROTATE, lateral_m=verdict.lateral_error_m)


def from_grasp_precondition(inputs) -> Correction | None:
    """GRASP 기본 전제 판정 -> 보정 요구. 움직여서 못 고치면 None.

    ⚠️ 2026-09-01 사용자 지시로 `GraspInputs`에서 estop_set·gripper_load·
    profile_known이 빠지면서(preconditions.check_grasp 문서 참고) 여기서
    보던 것도 둘로 줄었다. 그중 **Host가 차를 움직여 고칠 수 있는 것은
    여전히 하나뿐**이다 — 자기 뎁스캠이 목표를 못 본 경우다. 물체가 너무
    가까워 화각 아래로 빠졌거나 가려졌을 수 있고, 그때는 물러나서 다시
    보면 풀린다. "아직 안 멈췄다"는 자리 문제가 아니다 — 다음 사이클에
    저절로 풀린다.

    ⚠️ 2026-08-28 이전에는 이 자리에서 보정을 하나도 안 보냈다. 그래서
    Host가 "뎁스 카메라가 정면에서 목표를 찾지 못했다"를 고칠 수 없는 것으로
    읽고 **기물을 통째로 포기했다**(run6 로그의 `rook 보류: 고칠 수 없음`).
    실제로는 물러나면 되는 상황이었다."""
    if not inputs.base_stopped:
        return None
    if inputs.detected_label is None:
        # RETREAT 이지 REACQUIRE 가 아니다 (2026-08-29).
        #
        # REACQUIRE 는 "판정할 수 없으니 다시 보이게 세워 달라"이고 **방향이
        # 없다** — Host 는 그것을 받으면 찍어서 움직이지 않고 대상을 보류하는
        # 것이 맞다. 하지만 여기서는 Pi 가 방향을 안다. 정면에서 목표가 안
        # 보이는 상황에서 물러나는 것은 추측이 아니라 **유일하게 나아지는
        # 방향**이다: 물체가 너무 가까워 화각 아래로 빠졌으면 물러나야 다시
        # 들어오고, 더 붙으면 어느 경우에도 나빠지기만 한다.
        #
        # 방향을 아는 쪽이 방향을 말한다 — 그것이 이 모듈의 설계 원칙이다.
        # 크기는 안 싣는다. 얼마나 물러나야 하는지는 모르고, 지어낸 크기를
        # 주면 Host 가 그만큼 움직인다. Host 는 한 걸음 물러난 뒤 다시 묻는다.
        return Correction(RETREAT)
    return None


def from_insert(inputs) -> Correction | None:
    """INSERT 조건 판정 -> 보정 요구.

    ⚠️ 거리는 **라이다 판독 기준**이다. 차체 기준으로 환산해서 주지 않는다 —
    바구니는 높이마다 앞뒤가 달라 "차체에서 바구니까지"라는 단일 거리가
    정의되지 않기 때문이다(판으로 잰 값과 바구니로 잰 값이 2.6cm 어긋난
    실측이 근거다). Host는 이 값을 그대로 쓰지 말고 **줄어드는 방향으로
    조금씩 움직이며 다시 물어야** 한다."""
    from domain.task import baseline_constants as bc

    if not inputs.face_ok:
        return Correction(REACQUIRE)

    distance = inputs.face_distance_m
    if distance is None:
        return Correction(REACQUIRE)

    # 절벽 아래는 "더 가라"가 아니라 "물러나라"다. 그 아래로는 판독이
    # 커지는 방향으로 틀리므로 더 붙이면 상황이 나빠지기만 한다.
    if distance < bc.BASKET_MIN_LIDAR_M:
        return Correction(RETREAT,
                          forward_m=distance - bc.BASKET_STOP_LIDAR_M)

    error = distance - bc.BASKET_STOP_LIDAR_M
    if abs(error) > bc.BASKET_STOP_TOLERANCE_M:
        return Correction(ADVANCE if error > 0 else RETREAT, forward_m=error)

    if abs(inputs.face_yaw_error_rad) > bc.BASKET_YAW_TOLERANCE_RAD:
        return Correction(ROTATE, yaw_rad=inputs.face_yaw_error_rad)

    if (inputs.face_lateral_known
            and abs(inputs.face_lateral_offset_m) > bc.BASKET_LATERAL_TOLERANCE_M):
        return Correction(ROTATE, lateral_m=inputs.face_lateral_offset_m)

    # 남은 미충족(점 개수·안정성·부하)은 Host가 고칠 수 있는 것이 아니다.
    # 서 있는 자리 문제가 아니라 관측이나 파지 상태 문제이므로 None을 준다 —
    # 지어낸 보정을 주면 Host가 엉뚱하게 움직인다.
    return None
