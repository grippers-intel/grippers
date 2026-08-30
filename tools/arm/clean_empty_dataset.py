"""빈 LeRobot 데이터셋 폴더를 지운다 — **에피소드가 0개일 때만.**

## 왜 필요한가

`lerobot-record` 는 폴더를 **먼저 만들고 하드웨어를 나중에** 연다.

    dataset = LeRobotDataset.create(...)   # lerobot_dataset.py:488  폴더 생성
    ...
    robot.connect()                        # 519  카메라/모터 연결

그래서 카메라가 잡히지 않는 등으로 뒤에서 죽으면 **껍데기 폴더가 남는다.**
그 상태로 다시 실행하면 이렇게 막힌다 — 폴더 생성이 `exist_ok=False` 라서다.

    FileExistsError: [WinError 183] 파일이 이미 있으므로 만들 수 없습니다

손으로 지우다 보면 **진짜 찍어 둔 데이터를 날릴 위험**이 있다. 실제로 이
프로젝트에서 v1(5 에피소드)과 v2(5 에피소드)가 같은 자리에 나란히 있다.
그래서 이 스크립트는 `meta/info.json` 의 `total_episodes` 를 읽어
**0 이 아니면 무조건 거부한다.**

    python tools/arm/clean_empty_dataset.py lsy0284/gripper_pick_v3_grip65
    python tools/arm/clean_empty_dataset.py --list        # 전부 보기
"""
import json
import shutil
import sys
from pathlib import Path

ROOT = Path.home() / ".cache/huggingface/lerobot"


def _episodes(d: Path):
    """에피소드 수. 읽을 수 없으면 None — 그때는 지우지 않는다."""
    info = d / "meta/info.json"
    if not info.exists():
        return None
    try:
        return int(json.loads(info.read_text(encoding="utf-8"))["total_episodes"])
    except Exception:
        return None


def _listing() -> int:
    found = False
    for d in sorted(ROOT.glob("*/*")):
        if not d.is_dir() or d.name == "calibration":
            continue
        n = _episodes(d)
        size = sum(f.stat().st_size for f in d.rglob("*") if f.is_file()) / 1e6
        mark = "🗑  비어 있음" if n == 0 else ("?  판독 불가" if n is None else f"   {n} 에피소드")
        print(f"  {mark:16s} {size:8.1f}MB  {d.parent.name}/{d.name}")
        found = True
    if not found:
        print("  (데이터셋 없음)")
    return 0


def main(argv) -> int:
    if not argv:
        print(__doc__)
        return 2
    if argv[0] in ("--list", "-l"):
        return _listing()

    d = ROOT / argv[0]
    if not d.exists():
        print(f"없는 경로: {d}")
        return 1

    n = _episodes(d)
    if n is None:
        print(f"❌ {d}\n   meta/info.json 을 읽을 수 없어 몇 개인지 모릅니다 — 지우지 않습니다.")
        return 1
    if n != 0:
        print(f"❌ {d}\n   에피소드가 {n}개 들어 있습니다 — 지우지 않습니다.")
        print("   정말 버리시려면 탐색기에서 직접 지우세요.")
        return 1

    size = sum(f.stat().st_size for f in d.rglob("*") if f.is_file())
    shutil.rmtree(d)
    print(f"✅ 삭제: {d}  (에피소드 0개 · {size/1024:.1f}KB)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
