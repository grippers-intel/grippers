#!/usr/bin/env python3
"""탑뷰 방위각을 servo 1 각도로 옮기는 환산을 실측한다 (2026-09-07).

## 무엇을 재는가

사용자 지시:

    "approach 이후 grip 하기 전에 아루코마커와 기물이 일직선상에 놓여있지
     않아서 1번모터를 활용해 yaw 값을 수정해서 일직선상에 놓고 싶어.
     탑뷰카메라 기준으로 로봇 아루코마커와 기물의 좌표가 일직선상에 위치하게."

탑뷰는 로봇 마커의 자세(x, y, yaw)와 기물의 좌표를 안다. 그러니 "마커에서
기물을 보는 방위각"은 순수 기하로 매 회차 나온다:

    θ = atan2(좌우, 전방)          (run_mission 의 `[파지 진입]` 줄이 그대로 준다)

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

한 점만 재면 a 와 b 가 섞여서 못 가른다. 그래서 이 도구는 **여러 점**을
받는다.

## 절차

준비: bringup 이 떠 있어야 한다(arm_driver 서비스를 쓴다). 미션은 IDLE 로
두고, 팔은 기물을 겨누는 것이 보이는 자세여야 한다.

    ros2 action send_goal /arm_driver/move_to_floor_pose \\
        grippers_interfaces/action/MoveToFloorPose \\
        "{profile: chess_queen, stage: safe}"

1. 기물을 로봇 앞에 놓는다. **좌우 위치를 회차마다 바꾼다** — 왼쪽 6cm,
   왼쪽 2cm, 정면, 오른쪽 2cm, 오른쪽 6cm 처럼. 한쪽에만 몰아서 재면
   a 와 b 가 다시 섞인다.
2. run_mission 이 그 자리에서 찍는 줄을 읽는다:

       [파지 진입] 물체가 로봇 기준 전방 +307mm · 좌우 -20mm

3. 이 도구를 --record 로 돌려 그 두 숫자를 입력한다.
4. servo 1 을 조금씩 돌려(숫자 입력) **그리퍼가 기물을 정면으로 겨눌 때**
   ok 를 친다. 도구가 (θ, servo1) 한 쌍을 남긴다.
5. 5점 이상 모으고 --fit 을 돌린다.

## 쓰는 법

    python3 tools/arm/calibrate_servo1_aim.py --record     # 한 점씩 모은다
    python3 tools/arm/calibrate_servo1_aim.py --fit        # 직선을 긋는다
    python3 tools/arm/calibrate_servo1_aim.py --show       # 모은 점 보기

⚠️ servo 1 은 `/arm_driver/offset_base_yaw` 로만 돌린다. 그 서비스가 ±15도
한계를 걸고 있고, 미션이 실제로 쓸 경로도 그것이라 같은 길로 재야 한다.

⚠️ 각 점이 끝나면 servo 1 을 **출발 위치로 되돌린다.** 안 그러면 다음 점의
영점이 이번 점만큼 밀려서 직선이 휜다.
"""
import argparse
import json
import math
import pathlib
import sys

#: 기본 기록 파일. /shared 에 두는 이유는 저장소 안에 두면 git stash -u 에
#: 휩쓸리기 때문이다(2026-09-05 에 체크포인트로 겪었다).
DEFAULT_LOG = "/shared/servo1_aim_calibration.jsonl"

#: 4096 카운트 = 360도. driver_sdk.position_to_degrees 와 같은 환산이다.
RAW_PER_DEG = 4095.0 / 360.0

#: offset_base_yaw 가 거는 한계(arm_driver.MAX_BASE_YAW_OFFSET_RAD).
#: 여기서도 같은 값으로 막아 서비스가 거부하기 전에 알려 준다.
LIMIT_DEG = 15.0


def bearing_deg(forward_mm: float, lateral_mm: float) -> float:
    """탑뷰가 준 전방·좌우에서 방위각(도). 좌우 부호를 그대로 따른다.

    run_mission 의 `[파지 진입]` 줄과 같은 부호 규약이다 — 좌우가 음수면
    기물이 오른쪽, 양수면 왼쪽이다(그 줄을 찍는 host/mission.py 참고).
    """
    if forward_mm <= 0.0:
        raise ValueError(f"전방 거리가 0 이하다: {forward_mm}")
    return math.degrees(math.atan2(lateral_mm, forward_mm))


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
    """arm_driver 서비스를 통해서만 servo 1 을 돌린다."""

    def __init__(self):
        import rclpy
        from grippers_interfaces.srv import OffsetBaseYaw
        from rclpy.node import Node

        rclpy.init()
        self._rclpy = rclpy
        self._node = Node("calibrate_servo1_aim")
        self._cli = self._node.create_client(OffsetBaseYaw, "/arm_driver/offset_base_yaw")
        self._srv = OffsetBaseYaw
        if not self._cli.wait_for_service(timeout_sec=10.0):
            raise SystemExit(
                "/arm_driver/offset_base_yaw 가 없다 — bringup 이 떠 있는지 볼 것")

    def nudge(self, deg: float):
        """현재 위치에서 deg 만큼 돌리고 (ok, message, position_raw) 를 준다."""
        req = self._srv.Request(offset_rad=float(math.radians(deg)))
        future = self._cli.call_async(req)
        self._rclpy.spin_until_future_complete(self._node, future, timeout_sec=20.0)
        res = future.result()
        if res is None:
            return False, "응답 없음", None
        return res.ok, res.message, res.position_raw

    def close(self):
        self._node.destroy_node()
        self._rclpy.shutdown()


def _ask(prompt: str) -> str:
    try:
        return input(prompt).strip()
    except EOFError:
        return "q"


def record(path: pathlib.Path) -> int:
    arm = Arm()
    saved = 0
    try:
        while True:
            line = _ask("\n탑뷰 전방mm 좌우mm (그만두려면 q): ")
            if line in ("q", "quit", ""):
                break
            try:
                forward_s, lateral_s = line.split()
                theta = bearing_deg(float(forward_s), float(lateral_s))
            except ValueError as e:
                print(f"  두 숫자를 공백으로 띄어 입력할 것 ({e})")
                continue

            print(f"  θ = {theta:+.2f}도  "
                  f"(기물이 {'왼쪽' if theta > 0 else '오른쪽'})")
            print("  servo 1 을 돌려 그리퍼가 기물을 정면으로 겨누게 하세요.")
            print("  숫자 = 그만큼 상대 회전(도), ok = 확정, s = 이 점 버림")

            applied = 0.0
            while True:
                answer = _ask(f"  [누적 {applied:+.2f}도] > ")
                if answer in ("ok", "s", "q"):
                    break
                try:
                    step = float(answer)
                except ValueError:
                    print("    숫자거나 ok / s 여야 한다")
                    continue
                if abs(applied + step) > LIMIT_DEG:
                    print(f"    누적이 한계 ±{LIMIT_DEG:.0f}도를 넘는다 — 거부")
                    continue
                ok, message, raw = arm.nudge(step)
                if not ok:
                    print(f"    실패: {message}")
                    continue
                applied += step
                print(f"    servo 1 raw {raw}")

            # 무슨 일이 있어도 출발 위치로 되돌린다 — 다음 점의 영점이
            # 이번 점만큼 밀리면 직선이 휜다.
            if applied != 0.0:
                back_ok, back_msg, _raw = arm.nudge(-applied)
                if not back_ok:
                    print(f"  ⚠️ 원위치 복귀 실패: {back_msg} — 다음 점 전에 손으로 맞출 것")

            if answer == "q":
                break
            if answer == "s":
                print("  버렸다")
                continue

            sample = {"forward_mm": float(forward_s), "lateral_mm": float(lateral_s),
                      "theta_deg": round(theta, 3), "servo1_deg": round(applied, 3)}
            with path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(sample, ensure_ascii=False) + "\n")
            saved += 1
            print(f"  기록: θ={theta:+.2f}  servo1={applied:+.2f}   (총 {saved}점)")
    finally:
        arm.close()
    print(f"\n{saved}점을 {path} 에 남겼다.")
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
    print(f"{len(samples)}점\n   θ(도)   servo1(도)")
    for theta, servo1 in samples:
        print(f"  {theta:+7.2f}   {servo1:+7.2f}")
    spread = max(t for t, _ in samples) - min(t for t, _ in samples)
    print(f"\nθ 범위 {spread:.1f}도")
    if spread < 6.0:
        print("⚠️ 범위가 좁다 — 기물을 좌우로 더 벌려 가며 재야 부호와 영점이 갈린다")
    return 0


def fit(path: pathlib.Path) -> int:
    samples = load(path)
    if len(samples) < 2:
        print(f"{path} 에 점이 {len(samples)}개뿐이다 — 최소 2점, 5점 이상 권장")
        return 1
    a, b, rms, resid = fit_line(samples)

    print(f"{len(samples)}점 최소제곱\n")
    print(f"    servo1 = {a:+.4f} · θ  {b:+.3f}도")
    print(f"    잔차 RMS {rms:.2f}도")
    worst = max(range(len(resid)), key=lambda k: abs(resid[k]))
    print(f"    최대 잔차 {resid[worst]:+.2f}도 (θ={samples[worst][0]:+.2f})\n")

    print("읽는 법")
    if a < 0:
        print(f"  부호: a 가 음수({a:+.2f}) — 탑뷰 θ 와 servo 1 이 **반대**다.")
        print("        지금까지 코드가 가정하던 -θ 가 맞았다는 뜻이다.")
    else:
        print(f"  ⚠️ 부호: a 가 양수({a:+.2f}) — 탑뷰 θ 와 servo 1 이 **같은** 방향이다.")
        print("        코드의 -θ 가정이 틀렸다. 부호를 뒤집어야 한다.")
    if abs(abs(a) - 1.0) > 0.25:
        print(f"  ⚠️ 배율 |a|={abs(a):.2f} 이 1 에서 멀다. 1:1 이 아니라면 "
              f"마커 장착각(YAW_OFFSET_DEG)이나 팔 길이 전제를 볼 것.")
    print(f"  영점: b = {b:+.2f}도. 마커 정면 축과 팔 베이스 0 도의 어긋남이다 —")
    print(f"        예전 PIECE_AIM_YAW_TRIM_DEG(4.5도, 눈대중)가 잡으려던 값.")
    if rms > 1.5:
        print(f"\n⚠️ 잔차 RMS {rms:.2f}도가 크다. 점이 흩어져 있다는 뜻이고, "
              f"'정면으로 겨눴다'는 판정이 회차마다 달랐을 가능성이 높다.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="탑뷰 방위각 -> servo 1 각도 환산을 실측한다")
    parser.add_argument("--log", default=DEFAULT_LOG, help=f"기록 파일 (기본 {DEFAULT_LOG})")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--record", action="store_true", help="한 점씩 모은다")
    mode.add_argument("--fit", action="store_true", help="모은 점에 직선을 긋는다")
    mode.add_argument("--show", action="store_true", help="모은 점을 본다")
    args = parser.parse_args()

    path = pathlib.Path(args.log)
    if args.record:
        path.parent.mkdir(parents=True, exist_ok=True)
        return record(path)
    if args.show:
        return show(path)
    return fit(path)


if __name__ == "__main__":
    sys.exit(main())
