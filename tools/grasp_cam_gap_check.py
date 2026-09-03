#!/usr/bin/env python3
"""그리퍼캠 "닫은 뒤 틈 밝기" 신호를 실기에서 눈으로 확인하는 스모크 테스트.

grasp_cam_diff_check.py와 달리 기준 프레임이 필요 없다 — 그리퍼를 완전히
닫은 뒤 Enter만 누르면 된다. domain/task/grasp_cam_gap.py 모듈 docstring
참고 — ROI/threshold는 아직 미실측 임시치다.

쓰는 법:
    python3 tools/grasp_cam_gap_check.py
    1) 그리퍼를 완전히 닫고(물체 없이) Enter — 빈 상태 bright_ratio 확인
    2) 물체를 물리고 그리퍼를 완전히 닫은 뒤 Enter — 물린 상태 확인
    3) q + Enter로 종료. 그 사이엔 Enter만 눌러서 반복 확인 가능.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from domain.adapters.real.gripper_cam_reader import GripperCamGapConfirm  # noqa: E402


def main() -> int:
    confirm = GripperCamGapConfirm()
    print("그리퍼를 완전히 닫은 뒤(물체 유무 상관없이) Enter, q+Enter로 종료.")

    while True:
        line = input("> ").strip()
        if line.lower() == "q":
            return 0
        verdict = confirm.check()
        print(
            f"  bright_ratio={verdict.bright_ratio:.3f} "
            f"ratio_threshold={confirm.ratio_threshold:.3f} "
            f"mean_brightness={verdict.mean_brightness:.1f} "
            f"confirmed={verdict.confirmed}"
        )


if __name__ == "__main__":
    raise SystemExit(main())
