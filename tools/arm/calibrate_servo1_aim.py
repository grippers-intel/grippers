#!/usr/bin/env python3
"""탑뷰 방위각을 servo 1 각도로 옮기는 환산을 실측한다 (2026-09-07).

## 무엇을 재는가

사용자 지시:

    "approach 이후 grip 하기 전에 아루코마커와 기물이 일직선상에 놓여있지
     않아서 1번모터를 활용해 yaw 값을 수정해서 일직선상에 놓고 싶어.
     탑뷰카메라 기준으로 로봇 아루코마커와 기물의 좌표가 일직선상에 위치하게."

탑뷰는 로봇 마커의 자세(x, y, yaw)와 기물의 좌표를 안다. 그러니 "마커에서
기물을 보는 방위각"은 순수 기하로 매 회차 나온다:

    θ = atan2(좌우, 전방)

문제는 그 θ 를 servo 1 에 **얼마로** 주느냐다. 두 가지를 모르기 때문이다.

    부호   servo 1 의 + 방향과 탑뷰의 + 방향이 같은가 반대인가.
           지금까지 `-θ` 라고 **가정**만 하고 실기로 확인한 적이 없다.
    영점   마커의 정면 축과 팔 베이스의 0 도가 기계적으로 어긋나 있다.
           이것이 예전 PIECE_AIM_YAW_TRIM_DEG(4.5도)가 잡으려던 값이고,
           눈대중 두 번(0 -> 6.8 -> 4.5)으로도 1~2cm 가 남았다.

둘을 한 번에 재는 방법은 하나뿐이다 — **θ 를 여러 값으로 바꿔 가며 실제로
맞는 servo 1 각도를 찾고 직선을 긋는 것**이다.

    servo1 = a·θ + b

    a  부호와 배율.  기대값 -1.0 (부호가 반대이고 1:1)
    b  영점 오프셋.  예전 트림이 잡으려던 그 값

한 점만 재면 a 와 b 가 섞여서 못 가른다. 그래서 여러 점을 받는다.

## 어떻게 재는가 — 토크를 풀고 손으로 맞춘 뒤 **읽는다**

사용자 지시(2026-09-07): "move_to_floor_pose 는 팀원 자세라 우리 VLA 형식에
맞지 않는다. 차라리 토크값을 다 풀고 6,2,0,2,6 기준에 가져다 대 줄 테니까
직접 확인하는 방식으로 하고 싶다."

그 편이 낫다. servo 1 을 **명령해서** 맞추면 명령 경로의 한계(±15도)와
데드밴드가 측정에 섞이는데, 손으로 맞추고 **읽기만** 하면 그것들이 빠진다.
서보는 토크가 꺼져 있어도 자기 위치를 정확히 읽는다.

    1. 팔 토크를 푼다                      --free
    2. 기물을 정해진 좌우 자리에 놓는다     -60 / -20 / 0 / +20 / +60 mm
    3. 손으로 팔 베이스를 돌려 그리퍼가 그 기물을 정면으로 겨누게 한다
    4. Enter — servo 1 위치를 읽어 (θ, servo1) 한 쌍을 남긴다
    5. 다섯 자리를 다 돈 뒤 --fit

⚠️ 좌우 자리는 **마커 정면 축**에서 재야 한다. 차체 중심선이 아니다 —
운영 때 θ 를 내는 기준이 마커이므로, 재는 기준과 쓰는 기준이 같아야 b 가
뜻을 갖는다.

⚠️ **bringup 을 내리고 실행한다.** arm_driver 가 /dev/soarm 을 배타 잠금하고
있어 붙을 수 없고, 떠 있으면 토크도 다시 켜 버린다.

⚠️ --free 는 **servo 1 만** 푼다. 2~6 을 같이 풀면 팔이 중력으로 쓰러진다
(park_release_torque.py 의 같은 경고). 베이스 회전만 손으로 돌리면 된다.

## 쓰는 법

    ./tools/ops/stop_bringup.sh                                먼저 내린다

    python3 tools/arm/calibrate_servo1_aim.py --free           토크를 푼다
    python3 tools/arm/calibrate_servo1_aim.py --record         한 점씩 모은다
    python3 tools/arm/calibrate_servo1_aim.py --fit            직선을 긋는다
    python3 tools/arm/calibrate_servo1_aim.py --show           모은 점 보기
    python3 tools/arm/calibrate_servo1_aim.py --hold           토크를 되켠다
"""
import argparse
import json
import math
import pathlib
import sys

#: 기본 기록 파일. /shared 에 두는 이유는 저장소 안에 두면 git stash -u 에
#: 휩쓸리기 때문이다(2026-09-05 에 체크포인트로 겪었다).
DEFAULT_LOG = "/shared/servo1_aim_calibration.jsonl"

DEFAULT_PORT = "/dev/soarm"

#: GRASP 좌우 보정이 실제로 쓸 수 있는 한계
#: (arm_driver.MAX_BASE_YAW_OFFSET_RAD). 잰 값이 이보다 크면 조준으로 풀
#: 문제가 아니라는 신호라 경고한다.
LIMIT_DEG = 15.0

#: 파지 진입 전방 거리(mm). host/mission_config.GRASP_TRIGGER_DIST_M 과 같아야
#: 한다 — 같은 거리에서 재야 각이 운영과 맞는다.
DEFAULT_FORWARD_MM = 320.0

#: 기물을 놓을 좌우 자리(mm). 음수가 오른쪽, 양수가 왼쪽 —
#: run_mission 의 `[파지 진입] 좌우` 와 같은 부호다.
LATERAL_LADDER_MM = (-60.0, -20.0, 0.0, 20.0, 60.0)

#: 교시 IDLE 의 servo 1(floor_grasp_profiles.IDLE_CRADLE_RAW[0]).
#: 잰 절대각을 "IDLE 기준 오프셋"으로도 보려는 기준점이다.
IDLE_SERVO1_RAW = 2066

#: 4096 카운트 = 360도, 중앙 2048. driver_sdk 와 같은 환산.
POS_CENTER = 2048


def bearing_deg(forward_mm: float, lateral_mm: float) -> float:
    """전방·좌우에서 방위각(도). 좌우 부호를 그대로 따른다.

    run_mission 의 `[파지 진입]` 줄과 같은 부호 규약이다 — 좌우가 음수면
    기물이 오른쪽, 양수면 왼쪽이다.
    """
    if forward_mm <= 0.0:
        raise ValueError(f"전방 거리가 0 이하다: {forward_mm}")
    return math.degrees(math.atan2(lateral_mm, forward_mm))


def raw_to_deg(raw: int) -> float:
    """servo raw -> 도. driver_sdk.position_to_degrees 와 같은 환산."""
    return (raw - POS_CENTER) / 4095.0 * 360.0


def fit_line(samples):
    """(θ, servo1) 점들에 servo1 = a·θ + b 를 최소제곱으로 맞춘다.

    numpy 를 안 쓴다 — 이 컨테이너에 있는지 보장이 없고, 2변수 최소제곱은
    닫힌 식이 짧다.
    """
    n = len(samples)
    if n < 2:
        raise ValueError("점이 2개 미만이면 직선을 못 긋는다")
    sx = sum(t for t, _s in samples)
    sy = sum(s for _t, s in samples)
    sxx = sum(t * t for t, _s in samples)
    sxy = sum(t * s for t, s in samples)
    denom = n * sxx - sx * sx
    if abs(denom) < 1e-9:
        raise ValueError(
            "θ 가 전부 같은 값이다 — 기물을 좌우로 옮겨 가며 재야 "
            "부호(a)와 영점(b)이 갈린다")
    a = (n * sxy - sx * sy) / denom
    b = (sy - a * sx) / n
    resid = [s - (a * t + b) for t, s in samples]
    rms = math.sqrt(sum(r * r for r in resid) / n)
    return a, b, rms, resid


class Arm:
    """servo 1 에 직접 붙는다 — 토크를 끄고 위치를 읽기 위해서다.

    ⚠️ arm_driver 를 안 거친다. 그 노드가 /dev/soarm 을 배타 잠금하므로
    **bringup 이 내려가 있어야** 한다. 그리고 이 측정의 요점이 "명령하지 않고
    읽는 것"이라 명령 경로를 거칠 이유도 없다.
    """

    SERVO1 = 1

    def __init__(self, port: str):
        # driver_sdk(pyserial 의존)는 여기서만 import 한다 —
        # park_release_torque.py 의 _connect() 와 같은 이유.
        import soarm_lab  # noqa: F401  (flat import 를 위해 먼저 import)
        from driver_sdk import STS3215Driver

        self._drv = STS3215Driver(port)
        if not self._drv.connect():
            raise SystemExit(
                f"{port} 연결 실패 — arm_driver 가 떠 있으면 배타 잠금 때문이다. "
                "먼저 ./tools/ops/stop_bringup.sh 로 내릴 것")

    def position_raw(self):
        """servo 1 의 현재 위치(raw). 못 읽으면 None."""
        return self._drv.get_position(self.SERVO1)

    def set_torque(self, on: bool) -> bool:
        return bool(self._drv.set_torque(self.SERVO1, on))

    def close(self) -> None:
        self._drv.disconnect()


def _ask(prompt: str) -> str:
    try:
        return input(prompt).strip()
    except EOFError:
        return "q"


def free(port: str) -> int:
    """servo 1 토크만 푼다. 2~6 은 안 건드린다 — 중력으로 쓰러진다."""
    arm = Arm(port)
    try:
        before = arm.position_raw()
        ok = arm.set_torque(False)
        where = (f"지금 {before} = {raw_to_deg(before):+.2f}도"
                 if before is not None else "위치 읽기 실패")
        print(f"servo 1 토크 해제: {'성공' if ok else '실패'}  ({where})")
        print("이제 팔 베이스를 손으로 돌릴 수 있다. servo 2~6 은 그대로 잠겨 있다.")
        return 0 if ok else 1
    finally:
        arm.close()


def hold(port: str) -> int:
    """토크를 되켠다.

    goal 은 건드리지 않는다 — STS3215 는 goal 을 쓰면 토크가 자동으로 켜지면서
    그 목표로 **움직인다**(arm_driver._latch_torque_at_present 주석). 토크만
    켜면 서보가 지금 위치를 그대로 유지한다.
    """
    arm = Arm(port)
    try:
        raw = arm.position_raw()
        if raw is None:
            print("⚠️ 위치를 못 읽었다 — 배선을 보고 다시 시도할 것")
            return 1
        ok = arm.set_torque(True)
        print(f"servo 1 토크 복구: {'성공' if ok else '실패'}  "
              f"(위치 {raw} = {raw_to_deg(raw):+.2f}도)")
        return 0 if ok else 1
    finally:
        arm.close()


def record(path: pathlib.Path, port: str, forward_mm: float) -> int:
    """좌우 사다리를 한 자리씩 돌며 (θ, servo1) 을 모은다."""
    arm = Arm(port)
    saved = 0
    idle_deg = raw_to_deg(IDLE_SERVO1_RAW)
    try:
        if arm.position_raw() is None:
            print("⚠️ servo 1 위치를 못 읽는다 — 배선·전원을 볼 것")
            return 1

        ladder = ", ".join(f"{v:+.0f}" for v in LATERAL_LADDER_MM)
        print(f"전방 {forward_mm:.0f}mm 기준 · 좌우 자리 {ladder} mm")
        print("각 자리에서 기물을 놓고, 팔 베이스를 손으로 돌려 그리퍼가 그 기물을")
        print("정면으로 겨누게 한 뒤 Enter.   (s = 이 자리 건너뜀, q = 그만)")
        print("")

        for lateral in LATERAL_LADDER_MM:
            theta = bearing_deg(forward_mm, lateral)
            side = "정면" if lateral == 0 else ("왼쪽" if lateral > 0 else "오른쪽")
            answer = _ask(f"기물을 {side} {abs(lateral):.0f}mm 에 놓고 겨눈 뒤 "
                          f"Enter (θ={theta:+.2f}도) > ")
            if answer == "q":
                break
            if answer == "s":
                print("  건너뜀")
                print("")
                continue

            raw = arm.position_raw()
            if raw is None:
                print("  ⚠️ 위치 읽기 실패 — 이 자리는 버린다")
                print("")
                continue

            servo1 = raw_to_deg(raw)
            offset = servo1 - idle_deg
            if abs(offset) > LIMIT_DEG:
                print(f"  ⚠️ IDLE 기준 {offset:+.1f}도 — 운영 한계 "
                      f"±{LIMIT_DEG:.0f}도 밖이다. 기록은 하되, 이대로면 "
                      f"조준으로 못 푼다.")

            sample = {"forward_mm": forward_mm, "lateral_mm": lateral,
                      "theta_deg": round(theta, 3),
                      "servo1_deg": round(servo1, 3), "servo1_raw": int(raw)}
            with path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(sample, ensure_ascii=False) + "\n")
            saved += 1
            print(f"  기록: θ={theta:+.2f}  servo1={servo1:+.2f}도 "
                  f"(raw {raw}, IDLE 기준 {offset:+.2f}도)   [{saved}점]")
            print("")
    finally:
        arm.close()

    print(f"{saved}점을 {path} 에 남겼다.")
    if saved:
        print("끝났으면 --hold 로 토크를 되켤 것.")
    return 0


def load(path: pathlib.Path):
    if not path.exists():
        return []
    out = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            row = json.loads(line)
            out.append((row["theta_deg"], row["servo1_deg"]))
    return out


def show(path: pathlib.Path) -> int:
    samples = load(path)
    if not samples:
        print(f"{path} 에 점이 없다")
        return 1
    print(f"{len(samples)}점")
    print("   θ(도)   servo1(도)")
    for theta, servo1 in samples:
        print(f"  {theta:+7.2f}   {servo1:+7.2f}")
    spread = max(t for t, _ in samples) - min(t for t, _ in samples)
    print("")
    print(f"θ 범위 {spread:.1f}도")
    if spread < 6.0:
        print("⚠️ 범위가 좁다 — 기물을 좌우로 더 벌려 가며 재야 "
              "부호와 영점이 갈린다")
    return 0


def fit(path: pathlib.Path) -> int:
    samples = load(path)
    if len(samples) < 2:
        print(f"{path} 에 점이 {len(samples)}개뿐이다 — 최소 2점, 5점 이상 권장")
        return 1
    a, b, rms, resid = fit_line(samples)
    idle_deg = raw_to_deg(IDLE_SERVO1_RAW)

    print(f"{len(samples)}점 최소제곱")
    print("")
    print(f"    servo1(절대) = {a:+.4f} · θ  {b:+.3f}도")
    print(f"    IDLE 기준    = {a:+.4f} · θ  {b - idle_deg:+.3f}도")
    print(f"    잔차 RMS {rms:.2f}도")
    worst = max(range(len(resid)), key=lambda k: abs(resid[k]))
    print(f"    최대 잔차 {resid[worst]:+.2f}도 (θ={samples[worst][0]:+.2f})")
    print("")

    print("읽는 법")
    if a < 0:
        print(f"  부호: a 가 음수({a:+.2f}) — 탑뷰 θ 와 servo 1 이 **반대**다.")
        print("        코드가 가정하던 -θ 가 맞았다는 뜻이다.")
    else:
        print(f"  ⚠️ 부호: a 가 양수({a:+.2f}) — 탑뷰 θ 와 servo 1 이 **같은** 방향이다.")
        print("        코드의 -θ 가정이 틀렸다. 부호를 뒤집어야 한다.")
    if abs(abs(a) - 1.0) > 0.25:
        print(f"  ⚠️ 배율 |a|={abs(a):.2f} 이 1 에서 멀다. 1:1 이 아니라면 "
              f"마커 장착각(YAW_OFFSET_DEG)이나 팔 길이 전제를 볼 것.")
    print(f"  영점: IDLE 기준 {b - idle_deg:+.2f}도. 마커 정면 축과 팔 베이스의")
    print("        어긋남이고, 예전 PIECE_AIM_YAW_TRIM_DEG(4.5도, 눈대중)가")
    print("        잡으려던 값이다.")
    if abs(b - idle_deg) > LIMIT_DEG:
        print(f"  ⚠️ 영점이 운영 한계 ±{LIMIT_DEG:.0f}도 밖이다 — 조준으로 풀 "
              f"문제가 아니라 마커나 팔 장착을 봐야 한다.")
    if rms > 1.5:
        print("")
        print(f"⚠️ 잔차 RMS {rms:.2f}도가 크다. 점이 흩어져 있다는 뜻이고, "
              f"'정면으로 겨눴다' 판정이 자리마다 달랐을 가능성이 높다.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="탑뷰 방위각 -> servo 1 각도 환산을 실측한다")
    parser.add_argument("--log", default=DEFAULT_LOG,
                        help=f"기록 파일 (기본 {DEFAULT_LOG})")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--forward-mm", type=float, default=DEFAULT_FORWARD_MM,
                        help=f"파지 진입 전방 거리 (기본 {DEFAULT_FORWARD_MM:.0f})")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--free", action="store_true", help="servo 1 토크를 푼다")
    mode.add_argument("--hold", action="store_true", help="servo 1 토크를 되켠다")
    mode.add_argument("--record", action="store_true", help="한 점씩 모은다")
    mode.add_argument("--fit", action="store_true", help="모은 점에 직선을 긋는다")
    mode.add_argument("--show", action="store_true", help="모은 점을 본다")
    args = parser.parse_args()

    path = pathlib.Path(args.log)
    if args.free:
        return free(args.port)
    if args.hold:
        return hold(args.port)
    if args.record:
        path.parent.mkdir(parents=True, exist_ok=True)
        return record(path, args.port, args.forward_mm)
    if args.show:
        return show(path)
    return fit(path)


if __name__ == "__main__":
    sys.exit(main())
