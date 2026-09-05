#!/usr/bin/env python3
"""VLA 파지 한 번을 한 장으로 붙여 본다 — 정책이 무엇을 보고 무엇을 했는가.

## 왜 필요한가

파지가 끝난 뒤 무슨 일이 있었는지 볼 방법이 없었다. 2026-09-06 실기에서
사용자 관찰이 이랬다.

    거리 차이가 일관적이지 않다 — 그리퍼보다 가깝거나 멀거나가 반복된다.
    실제 파지로 이어지면 물체가 그리퍼에 가려지거나, 그리퍼에 밀려 살짝
    앞으로 날아간다.

프레임이 없으면 이게 **정책의 거리 판단** 문제인지 **차량 정지 위치** 문제인지
가릴 수가 없다. 둘은 고치는 곳이 완전히 다르다 — 전자는 학습 데이터, 후자는
주행 정지 정밀도다.

## 무엇을 보여 주는가

청크마다 정책이 본 원본 프레임을 시간 순으로 격자에 붙이고, 각 칸에 그 청크의
관절값 변화를 적는다. 접근이 어떻게 진행됐는지 한 눈에 보인다.

    01  pan +0.3  lift -12.4  ...      <- 첫 관측(팔이 IDLE)
    02  pan +0.1  lift -31.0  ...      <- 내려가는 중
    03  ...                            <- 턱이 물체에 닿는 순간
    04  ...                            <- 닫힘

## 쓰는 법

Pi 에서 record_dir 를 켜고 미션을 돌린 뒤:

    ros2 launch ... vla_record_dir:=/grippers/runs/vla

그다음 노트북에서:

    python tools/vla_grasp_review.py --pull           # 최신 한 판 받아서 열기
    python tools/vla_grasp_review.py --pull --all     # 전부 받기
    python tools/vla_grasp_review.py <폴더>           # 이미 받은 것 보기
"""

from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
import sys

PI = "pi@192.168.0.7"
REMOTE_BASE = "/home/pi/docker/shared/grippers/runs/vla"
LOCAL_BASE = pathlib.Path(__file__).resolve().parent.parent / "runs" / "vla"

#: 관절 이름. 정책 출력 순서 그대로다.
JOINTS = ("pan", "lift", "elbow", "wrist", "roll", "grip")


def _pull(only_latest: bool) -> pathlib.Path | None:
    """Pi 에서 기록을 받아온다. 받은 것 중 가장 최근 폴더를 돌려준다."""
    LOCAL_BASE.mkdir(parents=True, exist_ok=True)
    listing = subprocess.run(
        ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", PI,
         f"ls -1t {REMOTE_BASE} 2>/dev/null"],
        capture_output=True, text=True, encoding="utf-8", errors="replace",
        timeout=60)
    names = [n for n in listing.stdout.split() if n]
    if not names:
        print(f"{PI}:{REMOTE_BASE} 에 기록이 없습니다.")
        print("bringup 을 vla_record_dir:=/grippers/runs/vla 로 띄웠는지 보십시오.")
        return None
    targets = names[:1] if only_latest else names
    for name in targets:
        subprocess.run(
            ["scp", "-q", "-r", f"{PI}:{REMOTE_BASE}/{name}", str(LOCAL_BASE)],
            timeout=300)
        print(f"  받음  {name}")
    return LOCAL_BASE / targets[0]


def _load_meta(run_dir: pathlib.Path) -> dict[int, dict]:
    meta: dict[int, dict] = {}
    path = run_dir / "chunks.jsonl"
    if not path.exists():
        return meta
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            row = json.loads(line)
        except ValueError:
            continue
        meta[int(row["chunk"])] = row
    return meta


def _delta_text(row: dict) -> str:
    """이 청크에서 각 관절이 얼마나 움직이라고 했는가."""
    first, last = row.get("action_first"), row.get("action_last")
    if not first or not last:
        return ""
    parts = []
    for name, a, b in zip(JOINTS, first, last):
        d = b - a
        if abs(d) >= 1.0:            # 1도(그리퍼는 1%) 미만은 노이즈
            parts.append(f"{name}{d:+.0f}")
    return " ".join(parts) if parts else "거의 정지"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", nargs="?", help="볼 폴더. 없으면 --pull 필요")
    ap.add_argument("--pull", action="store_true", help="Pi 에서 받아온다")
    ap.add_argument("--all", action="store_true", help="--pull 과 함께: 전부 받는다")
    ap.add_argument("--no-show", action="store_true", help="창을 안 띄우고 저장만")
    args = ap.parse_args()

    if args.pull:
        run_dir = _pull(only_latest=not args.all)
        if run_dir is None:
            return 1
    elif args.run_dir:
        run_dir = pathlib.Path(args.run_dir)
    else:
        ap.error("폴더를 주거나 --pull 을 쓰십시오")

    frames = sorted(run_dir.glob("*.jpg"))
    if not frames:
        print(f"{run_dir} 에 프레임이 없습니다.")
        return 1
    meta = _load_meta(run_dir)

    import cv2
    import numpy as np

    cols = min(4, len(frames))
    rows = (len(frames) + cols - 1) // cols
    cell_w, cell_h = 480, 300
    sheet = np.zeros((rows * cell_h, cols * cell_w, 3), dtype=np.uint8)

    for i, path in enumerate(frames):
        img = cv2.imread(str(path))
        if img is None:
            continue
        img = cv2.resize(img, (cell_w, cell_h - 28))
        r, c = divmod(i, cols)
        y0, x0 = r * cell_h, c * cell_w
        sheet[y0 + 28:y0 + cell_h, x0:x0 + cell_w] = img
        idx = int(path.stem)
        label = f"{idx:02d}"
        row = meta.get(idx)
        if row:
            label += f"  {row.get('wall','')}  {_delta_text(row)}"
        cv2.putText(sheet, label[:70], (x0 + 6, y0 + 19),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.rectangle(sheet, (x0, y0), (x0 + cell_w - 1, y0 + cell_h - 1),
                      (60, 60, 60), 1)

    out = run_dir / "review.jpg"
    cv2.imwrite(str(out), sheet)
    print(f"\n{len(frames)}청크 · {run_dir.name}")
    print(f"저장: {out}")
    for idx in sorted(meta):
        print(f"  {idx:02d}  {_delta_text(meta[idx])}")
    if not args.no_show:
        cv2.imshow(run_dir.name, sheet)
        print("\n창을 클릭하고 아무 키나 누르면 닫힙니다.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
