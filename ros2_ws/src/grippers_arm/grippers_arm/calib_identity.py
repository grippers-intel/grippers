"""팔에 지금 실려 있는 캘리브레이션이 교시 자세와 같은 것인지 판정한다.

## 왜 이것이 필요한가

`floor_grasp_profiles.py` 의 파지 자세는 **RAW 서보값**이다. 그런데 그 RAW
값이 가리키는 물리 자세는 서보 EEPROM 의 `Homing_Offset` 에 달려 있다.

    Present_Position = Actual_Position - Homing_Offset

즉 같은 숫자가 오프셋이 바뀌면 **다른 자세**가 된다. 2026-08-29 에 VLA 시연
수집을 준비하며 LeRobot 캘리브레이션을 돌렸고, 그때 이 오프셋이 덮여 썼다.

**오프셋은 서보 안에 있지 git 에 있지 않다.** 브랜치를 바꿔도 팔은 안
바뀐다. 그래서 아무 경고 없이 이런 일이 일어날 수 있다.

    git checkout kica927/baseline_mission   (코드는 베이스라인)
    ros2 launch ... bringup.launch.py       (팔은 VLA 캘리브레이션)
    -> 교시된 RAW 값이 엉뚱한 물리 자세로 간다

sysy009 의 실측으로 shoulder_pan 가동폭이 2493 -> 2087 로 줄었다 — 차체와
라이다에 막히는 범위다. 그 방향으로 잘못 가면 부딪힌다. 그래서 이것은
경고가 아니라 **기동 거부** 사유다.

## 순수 함수다

서보를 읽는 것은 부르는 쪽이 하고, 여기서는 읽은 값으로 판정만 한다.
하드웨어 없이 테스트된다.
"""

# 판정
MATCH = "MATCH"              # 교시 당시와 같다 — 베이스라인을 돌려도 된다
MISMATCH = "MISMATCH"        # 다르다 — 교시 자세가 무효다
UNREADABLE = "UNREADABLE"    # 못 읽었다 — 모른다는 것과 같다는 것은 다르다

# STS3215 는 한 바퀴가 4096 카운트다.
COUNTS_PER_REV = 4096
DEGREES_PER_COUNT = 360.0 / COUNTS_PER_REV      # 0.0879도

# 오프셋은 EEPROM 정수라 저절로 흔들리지 않는다. 그래서 기본 허용치는 0 이다
# — 1카운트라도 다르면 누군가 캘리브레이션을 다시 돌린 것이다.
TOLERANCE_DEFAULT = 0


class CalibVerdict:
    def __init__(self, state, differences=None, unreadable=None):
        self.state = state
        self.differences = differences or {}    # servo_id -> (교시, 현재)
        self.unreadable = unreadable or []

    @property
    def ok(self) -> bool:
        return self.state == MATCH

    def message(self) -> str:
        if self.state == MATCH:
            return "교시 당시 캘리브레이션과 일치합니다"
        if self.state == UNREADABLE:
            return (f"Homing_Offset 을 못 읽은 서보가 있습니다 — id={self.unreadable}. "
                    "읽지 못한 것은 '같다'가 아닙니다. 교시 자세를 믿을 수 없습니다")

        lines = ["팔의 캘리브레이션이 교시 당시와 다릅니다 — "
                 "floor_grasp_profiles.py 의 RAW 자세가 전부 무효입니다."]
        for servo_id in sorted(self.differences):
            taught, current = self.differences[servo_id]
            delta = current - taught
            lines.append(
                f"  servo {servo_id}: 교시 {taught} · 현재 {current} "
                f"({delta:+d} 카운트 = {delta * DEGREES_PER_COUNT:+.1f}도)")
        lines.append("")
        lines.append("  베이스라인 미션을 돌리려면 교시 당시 오프셋을 되돌리세요.")
        lines.append("  파이에서(LeRobot 불필요, 팔이 내려옵니다):")
        lines.append("    python3 tools/arm/restore_taught_offsets.py --apply --yes")
        lines.append("  윈도우에서 LeRobot 으로:")
        lines.append("    python tools/arm/backup_servo_offsets.py COM8 "
                     "--restore tools/arm/servo_backup/servo_COM8_20260829_181124.json")
        lines.append("")
        lines.append("  VLA 수집 중이라면 이 팔로는 베이스라인을 돌리지 않습니다 —"
                     " 브랜치 kica927/smolVLA-version 쪽입니다.")
        return "\n".join(lines)


def verdict(current, taught, tolerance: int = TOLERANCE_DEFAULT) -> CalibVerdict:
    """`current`(servo_id -> 읽은 오프셋 또는 None)를 `taught` 와 비교한다.

    `taught` 에 없는 서보는 보지 않는다 — 교시 자세가 안 걸린 관절까지
    기동을 막을 이유가 없다."""
    unreadable = [sid for sid in taught if current.get(sid) is None]
    if unreadable:
        return CalibVerdict(UNREADABLE, unreadable=sorted(unreadable))

    differences = {sid: (taught[sid], int(current[sid]))
                   for sid in taught
                   if abs(int(current[sid]) - taught[sid]) > tolerance}
    if differences:
        return CalibVerdict(MISMATCH, differences=differences)
    return CalibVerdict(MATCH)


# 이 버스는 패킷을 이따금 흘린다. 서보 6개 연속 읽기라 묶음이 깨질 확률이
# 그만큼 쌓이므로, 단발 읽기로는 '못 읽었다'가 자주 나온다 — 그리고 못 읽은
# 것은 거부 사유다. 재시도가 없으면 패킷 하나 유실이 기동 거부가 된다.
# (arm_driver_node.JOINT_READ_ATTEMPTS 와 같은 값·같은 이유)
READ_ATTEMPTS_DEFAULT = 3
READ_RETRY_SEC = 0.05


def read_offsets(driver, servo_ids, attempts: int = READ_ATTEMPTS_DEFAULT):
    """`driver.get_homing_offset` 으로 한 벌 읽는다 — 실패하면 다시 시도한다.

    arm_driver_node 는 자기 `_read_with_retry` 를 쓴다(그쪽 계약과 로그를
    맞추려고). 이 함수는 노드 밖 도구용이다."""
    import time

    out = {}
    for sid in servo_ids:
        value = None
        for attempt in range(max(1, attempts)):
            value = driver.get_homing_offset(sid)
            if value is not None:
                break
            if attempt + 1 < attempts:
                time.sleep(READ_RETRY_SEC)
        out[sid] = value
    return out
