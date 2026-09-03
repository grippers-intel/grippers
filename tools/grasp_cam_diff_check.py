#!/usr/bin/env python3
"""그리퍼캠 밝기 diff 신호를 실기에서 눈으로 확인하는 스모크 테스트.

⚠️ 참고/로그용이다 — domain/task/grasp_cam_diff.py 모듈 docstring 참고,
이 신호는 이미 실측으로 무효였던 방식이라(빈 그리퍼가 물체를 문 상태보다
diff가 더 크게 나온 전례) GRASP 판정에 쓰면 안 된다. 실제 파지 판정은
Perception.confirm_grasp()(정면 뎁스 카메라)가 한다.

쓰는 법:
    python3 tools/grasp_cam_diff_check.py
    1) 그리퍼를 비운 채로 Enter — 기준 프레임을 잡는다
    2) 물체를 손에 쥐여 주거나 그리퍼로 물린 뒤 Enter — diff_score를 찍는다
    3) q + Enter 로 종료. 그 사이엔 Enter만 눌러서 반복 확인 가능.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from domain.adapters.real.gripper_cam_reader import GripperCamDiffConfirm  # noqa: E402


def main() -> int:
    confirm = GripperCamDiffConfirm()

    input("그리퍼를 비운 채로 Enter — 기준 프레임을 잡습니다> ")
    if not confirm.capture_reference():
        print("기준 프레임 캡처 실패 — /dev/gripper_cam 연결을 확인하세요")
        return 1
    print("기준 프레임 확보. 이제 물체를 그리퍼에 물리고 Enter, q+Enter로 종료.")

    while True:
        line = input("> ").strip()
        if line.lower() == "q":
            return 0
        verdict = confirm.check()
        print(
            f"  diff_score={verdict.diff_score:.2f} "
            f"threshold={confirm.threshold:.2f} "
            f"confirmed={verdict.confirmed} confidence={verdict.confidence:.3f}"
        )


if __name__ == "__main__":
    raise SystemExit(main())
