"""다중 프레임 합의 필터를 perception_node.py에 연결하는 다리.

세션 결정(2026-08-23): 이번 세션에서 새로 만든 multi_frame_consensus.py
(미터 단위, 미실측 상수, 미통합)를 폐기하고 tools/perception/consensus.py
(팀원 실기 튜닝본 — HANDOFF.md 검증: 산포 0.2~1.1px)를 그대로 재사용한다.
재구현하지 않는다 — 이 파일은 그 위에 얇은 게이트만 얹는다.

tools/는 설치되는 ROS2 패키지가 아니라 호스트/컨테이너 파일시스템에 직접
놓인 스크립트다(호스트 `/home/pi/docker/shared/grippers/tools` = 컨테이너
`/grippers/tools`, HANDOFF.md "환경 함정" 참고). 그래서 approach.py와 같은
방식(경로를 계산해 sys.path에 얹는 것)으로 가져온다 — 리포 루트 기준
상대 경로라 Mac 체크아웃에서도, Pi 컨테이너에서도 그대로 맞는다.
"""
from __future__ import annotations

import sys
from pathlib import Path

_THIS_FILE = Path(__file__).resolve()
# ⚠️ 2026-08-23 실기 확인: colcon 빌드 후에는 이 파일이 소스 트리
# (.../grippers/ros2_ws/src/...)가 아니라 설치 경로
# (/ros2_ws/install/grippers_perception/lib/python3.10/site-packages/...)에서
# 실행된다 — __file__ 기준 상대 경로(parents[4])로는 tools/perception을
# 못 찾는다(ModuleNotFoundError: consensus, 실기로 확인됨). 소스 트리에서
# 직접 돌리는 경우(로컬 pytest)를 위해 상대 경로도 먼저 시도하되, 실제
# 배포 환경(HANDOFF.md 환경 함정 — 호스트 `~/docker/shared/grippers` =
# 컨테이너 `/grippers`)의 절대 경로를 확실한 대안으로 둔다.
_CANDIDATE_TOOLS_DIRS = (
    _THIS_FILE.parents[4] / "tools" / "perception",  # 소스 트리에서 직접 실행
    Path("/grippers/tools/perception"),  # colcon install 이후 배포 환경
)
for _dir in _CANDIDATE_TOOLS_DIRS:
    if _dir.is_dir():
        if str(_dir) not in sys.path:
            sys.path.insert(0, str(_dir))
        break
else:
    raise ModuleNotFoundError(
        "tools/perception/consensus.py를 못 찾음 — 확인한 경로: "
        f"{[str(d) for d in _CANDIDATE_TOOLS_DIRS]}"
    )

from consensus import consensus  # noqa: E402  (tools/perception/consensus.py)

# tools/perception/floor_observer.py(팀원 실기 튜닝본)의 FloorObserver
# 기본값을 그대로 옮긴다 — HANDOFF.md "검증 완료된 것 > 인식"이 요약한
# conf 0.45 · k-of-n 0.6 · 순도 ≥0.80 · y≥290 · 산포 ≤40px 다섯 개는 이
# 기본값의 축약본이라, 거기 없는 두 개(MIN_SUPPORT_CONF·RELIABLE_CLASSES)
# 도 floor_observer.py 원본을 기준으로 함께 가져온다.
CONF_THRESHOLD = 0.45
K_OF_N_RATIO = 0.6
MIN_PURITY = 0.80
# ⚠️ 2026-08-23 실기 확인: 원본 290.0은 팀원이 더 가까운 거리(접근 막바지)
# 기준으로 튜닝한 값으로 보인다 — 이번 세션 SCAN 거리대(줄자 실측
# 0.66~1.13m, 축구공·나이트·룩·퀸)에서는 bbox 하단 y가 227~271px로 전부
# 290 미만이라, conf 0.79~0.92의 정상 검출까지 이 게이트 하나로 다
# 걸러졌다(scan_floor가 항상 빈 목록 반환). SCAN 거리대를 포함하도록
# 200으로 낮춘다 — 실측 최소값(227)보다 여유를 두되, 순도·평균신뢰도·
# 산포·허용목록 게이트는 그대로 둬 오검출은 그쪽에서 계속 걸러지게 한다.
# 사용자 결정(2026-08-23): 게이트 값을 SCAN 거리에 맞게 조정.
MIN_BOTTOM_Y_PX = 200.0
MAX_SPREAD_PX = 40.0
# 합의 후 트랙의 평균 신뢰도 하한 — 순도가 높아도(다수결이 압도적이어도)
# 평균 신뢰도 자체가 낮으면 여전히 못 믿는다(예: 매번 같은 오분류를 냄).
MIN_SUPPORT_CONF = 0.35
# 2026-08-23 실측(train-8): "box"는 60프레임 중 0회 검출, "star"는 신뢰도
# 0.31로 불안정했다 — 흰색 3D 프린팅 도형이 흰색 체스 기물과 형상·색이
# 겹치는 탓으로 추정. 데이터 보강 전까지 허용목록으로 막아 뒀었다.
#
# 2026-08-27 train-9로 재검증(tools/perception/floor_observer.py --frames
# 60): box 60/60·순도 1.00·신뢰 0.93·산포 0.2px, star 60/60·순도 1.00·
# 신뢰 0.95·산포 0.1px — 나머지 네 클래스보다도 깨끗하다. train-8의
# 검출력 한계였을 뿐 형상·색 겹침 자체가 원인은 아니었던 것으로 보인다.
# 이제 여섯 클래스 전부 허용목록에 둔다(floor_observer.py RELIABLE과 동일).
RELIABLE_CLASSES = ("knight", "queen", "rook", "soccer", "box", "star")


def confirmed_tracks(frames, n_frames):
    """frames: 프레임별 `[(raw_cls, conf, (x1,y1,x2,y2)), ...]` 리스트.

    호출자가 이미 CONF_THRESHOLD로 걸러서 넘겨야 한다(consensus() 자체는
    신뢰도 게이트를 모른다 — floor_observer.py와 같은 분담). 여기서는
    consensus()가 낸 Track 중 실기 검증된 게이트를 전부 통과한 것만
    돌려준다 — 하나라도 못 넘으면 오탐/불안정 클래스일 가능성이 높다는
    뜻이라 "모르면 제외" 원칙대로 뺀다."""
    tracks = consensus(frames, n_frames, min_ratio=K_OF_N_RATIO)
    confirmed = []
    for t in tracks:
        if t.label not in RELIABLE_CLASSES:
            continue
        if t.purity < MIN_PURITY:
            continue
        mean_conf = sum(t.confs) / len(t.confs)
        if mean_conf < MIN_SUPPORT_CONF:
            continue
        if t.spread > MAX_SPREAD_PX:
            continue
        _, bottom_y = t.center
        if bottom_y < MIN_BOTTOM_Y_PX:
            continue
        confirmed.append(t)
    return confirmed


def track_bbox_xyxy(track):
    """합의 필터가 낸 Track(바닥 접점 중앙값 + 폭·높이 중앙값)을
    perception_node.py의 `_approach_pose_m(class_name, bbox_xyxy)`가 받는
    bbox_xyxy 형태로 되돌린다. bottom_center()의 역연산이다."""
    cx, cy_bottom = track.center
    w, h = track.size
    return (cx - w / 2.0, cy_bottom - h, cx + w / 2.0, cy_bottom)
