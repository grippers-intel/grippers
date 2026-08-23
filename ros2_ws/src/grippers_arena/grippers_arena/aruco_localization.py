"""ArUco 기반 아레나 좌표계 확립 · 로봇 위치추정의 순수 수학 (미션 명세서
2026-08-23 파이프라인 02번).

"외부 카메라가 바닥 ArUco 마커를 읽어 아레나 좌표계를 세우고, 로봇 위
마커로 로봇 자세를 실시간 추적한다. SLAM도 Nav2도 필요 없다."

⚠️ `tools/a2/a2_homography.py`와 겹치지 않는다 — 그 도구는 **수동으로 클릭한
점**으로 호모그래피를 구하고 정확도를 실측 검증하는 오프라인 캘리브레이션
CLI다(mm 단위·공유 원점 등 좌표 규약은 그 도구의 README를 그대로 따른다).
여기는 그 대신 **ArUco 마커 코너를 자동 대응점으로 써서 호모그래피를 구하고,
로봇에 붙인 마커로 실시간 자세(x, y, θ)를 뽑는** 런타임 수학이다 — 무대
위에서 매 프레임 돌아가야 하니 사람이 CSV를 채우는 과정이 없어야 한다.

이 파일은 마커 검출(`cv2.aruco`)이나 카메라 입출력을 하지 않는다 — 검출된
마커 코너 픽셀 좌표를 **입력**으로 받는 순수 함수만 담아 rclpy·카메라 없이
pytest로 검증한다(grippers_perception의 hailo_scan_mapping.py와 같은
이유). 호모그래피 피팅(`fit_homography_dlt`)만 cv2가 필요해서 지연 import로
격리했다 — 나머지 함수는 numpy조차 안 쓴다.

⚠️ 아직 실기로 검증 안 됨: 마커 배치·크기·"어느 모서리가 로봇 정면인가"는
전부 팀이 실제로 마커를 붙인 뒤 확정해야 하는 값이다(아래 각 함수 docstring
참고). 아레나 실제 치수도 여기서 정하지 않는다 — 호출자가 `marker_layout`으로
넘긴다."""

import math

# ArUco 코너 순서(OpenCV `cv2.aruco.detectMarkers` 관례) — 마커 자신의 로컬
# 이미지 기준 [좌상단, 우상단, 우하단, 좌하단] 순서로 4개가 온다. 세계
# 좌표로 투영해도 이 순서(인덱스)는 유지된다.
CORNER_ORDER = ("top_left", "top_right", "bottom_right", "bottom_left")


def apply_homography(h, u, v):
    """3x3 호모그래피 `h`(행 우선 평탄화된 9개 값의 시퀀스)로 이미지 픽셀
    `(u, v)`를 바닥 평면 세계 좌표 `(x_mm, y_mm)`로 사상한다.

    표준 동차좌표 변환이다: `[x, y, w] = h @ [u, v, 1]`, 최종 좌표는
    `(x/w, y/w)`. `tools/a2/a2_homography.py`가 실측으로 검증하는 것과 같은
    호모그래피 계약이다 — 지시점은 항상 **바닥 평면 위의 점**이어야 한다
    (물체 중심이 아니라 바닥 접지선을 찍어야 하는 이유와 동일하게, 로봇
    마커도 바닥과 같은 높이가 아니면 원리적으로 오차가 생긴다 — 아래
    `robot_pose_from_marker_corners` 참고)."""
    h11, h12, h13, h21, h22, h23, h31, h32, h33 = h
    w = h31 * u + h32 * v + h33
    x = (h11 * u + h12 * v + h13) / w
    y = (h21 * u + h22 * v + h23) / w
    return x, y


def axis_aligned_marker_world_corners(center_mm, size_mm):
    """세계 좌표계 축에 나란히(회전 없이) 바닥에 붙인 정사각 마커의 4개
    모서리 세계 좌표를 `CORNER_ORDER`와 같은 순서로 반환한다.

    ⚠️ "축에 나란히"가 전제다 — 마커를 살짝 돌려 붙이면 이 함수로 만든
    기준점 자체가 틀린다. 아레나 고정 마커(모서리 4개)처럼 설치 시점에
    각도를 맞출 수 있는 경우에만 쓰고, 임의 각도로 놓일 수 있는 마커에는
    쓰지 않는다."""
    cx, cy = center_mm
    half = size_mm / 2.0
    return (
        (cx - half, cy + half),  # top_left    (세계 좌표는 y 위쪽이 +)
        (cx + half, cy + half),  # top_right
        (cx + half, cy - half),  # bottom_right
        (cx - half, cy - half),  # bottom_left
    )


def build_correspondences(detections, marker_layout):
    """검출된 마커 목록과 알려진 배치를 대응점 쌍으로 묶는다 — 호모그래피
    피팅(`fit_homography_dlt`)의 입력을 만드는 자리다.

    `detections`: `{marker_id: (코너 4개 이미지 픽셀 (u, v) 튜플, CORNER_ORDER
    순서)}`. `marker_layout`: `{marker_id: (코너 4개 세계 mm 좌표, 같은 순서)}`
    — 보통 `axis_aligned_marker_world_corners()`로 만든다.

    `marker_layout`에 없는 마커 ID는 조용히 건너뛴다 — "모르면 제외" 관례다.
    관객 오버레이용으로 놓아둔 장식 마커나, 아직 배치를 안 적어 넣은
    마커가 섞여 들어와도 호모그래피가 깨지지 않는다.

    반환: `(image_points, world_points)` — 순서가 맞춰진 두 리스트. 대응점이
    하나도 없으면 둘 다 빈 리스트다(호출자가 `fit_homography_dlt`에 그대로
    넘기면 점 부족으로 `None`을 받는다)."""
    image_points = []
    world_points = []
    for marker_id, image_corners in detections.items():
        world_corners = marker_layout.get(marker_id)
        if world_corners is None:
            continue
        image_points.extend(image_corners)
        world_points.extend(world_corners)
    return image_points, world_points


# 호모그래피를 유일하게 결정하는 데 필요한 최소 대응점 수(자유도 8, 점 1개당
# 방정식 2개). 그보다 적으면 cv2.findHomography 자체가 실패하거나 신뢰할 수
# 없는 해를 낸다 — 여기서 미리 걸러 "모르면 실패"로 접는다.
MIN_CORRESPONDENCES = 4


def fit_homography_dlt(image_points, world_points):
    """대응점으로 3x3 호모그래피를 피팅한다. `cv2.findHomography`(DLT +
    RANSAC)를 감싼 얇은 래퍼다 — 이 파일에서 cv2가 필요한 유일한 함수라
    지연 import로 격리했다(미설치 환경에서도 나머지 함수는 그대로 쓸 수
    있게).

    다음 중 하나면 **`None`**(다른 관측 포트와 같은 "모르면 실패" 관례):
    - cv2 미설치
    - 대응점이 `MIN_CORRESPONDENCES`보다 적음
    - `cv2.findHomography`가 해를 못 찾음(공선점 등)

    성공하면 `apply_homography()`가 바로 받는 평탄화된 9개 값 튜플을
    반환한다."""
    if len(image_points) < MIN_CORRESPONDENCES or len(world_points) < MIN_CORRESPONDENCES:
        return None

    try:
        import cv2
        import numpy as np
    except ImportError:
        return None

    src = np.array(image_points, dtype=np.float64)
    dst = np.array(world_points, dtype=np.float64)
    h, _mask = cv2.findHomography(src, dst, method=cv2.RANSAC)
    if h is None:
        return None
    return tuple(float(v) for v in h.flatten())


def robot_pose_from_marker_corners(h, image_corners, front_edge=(0, 1)):
    """로봇에 붙인 마커의 이미지 코너 4개(`CORNER_ORDER` 순서)로 로봇 자세
    `(x_mm, y_mm, theta_rad)`를 구한다 — 세계 좌표계 기준, 마커 중심을
    로봇 기준점으로 본다.

    `h`가 `None`(아직 아레나 호모그래피를 못 구함)이거나 코너가 정확히
    4개가 아니면 **`None`**.

    위치는 4개 코너를 각각 호모그래피로 투영한 세계 좌표의 **중심**이다.
    방위각은 `front_edge`가 가리키는 두 코너(기본값 (0, 1) = top_left→
    top_right)의 세계 좌표 중점 방향을 로봇 정면으로 보고, 마커 중심에서
    그 중점으로 향하는 벡터의 `atan2`다.

    ⚠️ `front_edge` 기본값은 **가정**이다 — 실제로 마커를 로봇 위에 어느
    방향으로 붙였는지에 따라 달라진다. 마커를 붙인 뒤 로봇을 세계 좌표계의
    알려진 방향으로 놓고 이 함수가 돌려주는 theta가 실제 정면과 맞는지
    확인해서, 안 맞으면 `front_edge`를 (1, 2)/(2, 3)/(3, 0) 중 실제 정면
    모서리로 바꿀 것 — 코드를 고치는 게 아니라 이 인자 하나만 바꾸면
    된다."""
    if h is None or len(image_corners) != 4:
        return None

    world_corners = [apply_homography(h, u, v) for u, v in image_corners]
    cx = sum(x for x, _y in world_corners) / 4.0
    cy = sum(y for _x, y in world_corners) / 4.0

    i, j = front_edge
    fx = (world_corners[i][0] + world_corners[j][0]) / 2.0
    fy = (world_corners[i][1] + world_corners[j][1]) / 2.0

    theta = math.atan2(fy - cy, fx - cx)
    return cx, cy, theta
