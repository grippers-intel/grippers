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
