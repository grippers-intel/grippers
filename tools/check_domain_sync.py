#!/usr/bin/env python3
"""벤더링한 `domain/` 사본이 grippers 본 저장소와 같은지 확인한다.

## 왜 사본이 있는가

`host/vehicle_link.py` 는 규격을 문서에서 베끼지 않고
`domain/ports/baseline_ports.py` 와 `domain/task/motion.py` 를 **직접
import** 한다. 두 파일 다 `abc`/`dataclasses`/`math` 만 쓰는 순수 파이썬이라
ROS2 없이도 로드된다. 좋은 설계다 — 문서와 코드가 갈라질 여지가 없다.

문제는 이 저장소가 monorepo 가 아니라는 것이다. 그래서 같은 레이아웃
(`repo/host/`, `repo/domain/`)을 유지하되 두 파일만 복사해 두었다.
`vehicle_link.py` 는 한 줄도 안 고쳤다.

## 그래서 갈라질 수 있다

원본이 바뀌면 이 사본은 조용히 낡는다. **grippers 본 저장소가 사본 문제로
이미 세 번 당했다**(queen 의 K, 턱 선 세 곳, JAW_LINE_FOR_HINT 의 star).
주석으로 "같이 고칠 것" 이라고 적어 두는 것으로는 못 막는다는 것이 그 저장소의
결론이었고, 그래서 거기도 실행되는 검사(`tests/test_constant_copies.py`)로
바꿨다. 여기도 같은 이유로 검사를 둔다.

## 실행

    python3 tools/check_domain_sync.py --upstream ~/Desktop/intel/grippers

일치하면 0, 다르면 1 을 돌려준다.
"""
import argparse
import hashlib
import pathlib
import sys

FILES = ("domain/ports/baseline_ports.py", "domain/task/motion.py")
ROOT = pathlib.Path(__file__).resolve().parent.parent


def digest(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()[:16]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--upstream", required=True,
                    help="grippers 본 저장소 경로")
    args = ap.parse_args()
    up = pathlib.Path(args.upstream).expanduser().resolve()

    if not up.is_dir():
        print(f"⛔ 본 저장소를 못 찾았다: {up}", file=sys.stderr)
        return 2

    bad = 0
    for rel in FILES:
        mine, theirs = ROOT / rel, up / rel
        if not theirs.exists():
            print(f"⛔ 원본 없음: {theirs}")
            bad += 1
            continue
        a, b = digest(mine), digest(theirs)
        if a == b:
            print(f"✅ {rel}  {a}")
        else:
            print(f"⛔ {rel}  사본 {a} != 원본 {b}")
            print(f"   고치려면: cp {theirs} {mine}")
            bad += 1
    if bad:
        print(f"\n{bad}개가 어긋났다. vehicle_link 가 쓰는 규격이 낡았다는 뜻이다.")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
