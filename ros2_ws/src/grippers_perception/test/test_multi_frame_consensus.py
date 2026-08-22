"""multi_frame_consensus.consensus_detections() 순수 로직 테스트.

rclpy/cv2 없이도 돈다 — perception_node.py 자체는 rclpy를 무조건 import해서
ROS2 없이는 임포트가 안 되지만, 이 로직은 그 파일에서 뽑아냈으므로(2026-08-23,
미션 명세서 파이프라인 04번) 여기서만 순수 pytest로 검증한다."""

from grippers_perception.multi_frame_consensus import (
    RawDetection,
    consensus_detections,
)


def test_object_seen_in_every_frame_is_kept_with_median_pose():
    """3프레임 모두에서 본 물체는 k_of_n=3을 통과하고, 위치는 중앙값이다."""
    frames = [
        [RawDetection("CHESS_PIECE", x=0.30, y=0.02, yaw_rad=0.10, confidence=0.85)],
        [RawDetection("CHESS_PIECE", x=0.31, y=0.00, yaw_rad=0.12, confidence=0.90)],
        [RawDetection("CHESS_PIECE", x=0.29, y=0.01, yaw_rad=0.11, confidence=0.82)],
    ]

    result = consensus_detections(frames, k_of_n=3, cluster_radius_m=0.05)

    assert len(result) == 1
    merged = result[0]
    assert merged.class_key == "CHESS_PIECE"
    assert merged.x == 0.30  # median([0.30, 0.31, 0.29])
    assert merged.y == 0.01  # median([0.02, 0.00, 0.01])
    assert merged.confidence == 0.90  # 멤버 중 최댓값
    assert merged.frames_seen == 3


def test_object_seen_below_k_of_n_is_dropped_as_noise():
    """오검출/일시적 노이즈처럼 1프레임에서만 본 물체는 버려진다."""
    frames = [
        [RawDetection("GABE", x=0.50, y=0.10, yaw_rad=0.0, confidence=0.85)],
        [],  # 2프레임째는 안 보임 — 진짜 물체라면 흔치 않은 패턴
        [],
    ]

    result = consensus_detections(frames, k_of_n=3, cluster_radius_m=0.05)

    assert result == []


def test_two_distinct_objects_of_same_class_stay_separate():
    """같은 클래스라도 cluster_radius_m 밖에 있으면 서로 다른 물체로 남는다."""
    frames = [
        [
            RawDetection("CHESS_PIECE", x=0.30, y=0.00, yaw_rad=0.0, confidence=0.8),
            RawDetection("CHESS_PIECE", x=0.90, y=0.50, yaw_rad=0.0, confidence=0.8),
        ],
        [
            RawDetection("CHESS_PIECE", x=0.31, y=0.01, yaw_rad=0.0, confidence=0.8),
            RawDetection("CHESS_PIECE", x=0.89, y=0.51, yaw_rad=0.0, confidence=0.8),
        ],
    ]

    result = consensus_detections(frames, k_of_n=2, cluster_radius_m=0.05)

    assert len(result) == 2
    positions = sorted((round(d.x, 2), round(d.y, 2)) for d in result)
    assert positions == [(0.3, 0.01), (0.9, 0.51)]


def test_different_classes_never_merge_even_when_close():
    """같은 자리에 겹쳐 보여도 class_key가 다르면 절대 합치지 않는다."""
    frames = [
        [
            RawDetection("CHESS_PIECE", x=0.30, y=0.00, yaw_rad=0.0, confidence=0.8),
            RawDetection("GABE", x=0.30, y=0.00, yaw_rad=0.0, confidence=0.8),
        ],
        [
            RawDetection("CHESS_PIECE", x=0.30, y=0.00, yaw_rad=0.0, confidence=0.8),
            RawDetection("GABE", x=0.30, y=0.00, yaw_rad=0.0, confidence=0.8),
        ],
    ]

    result = consensus_detections(frames, k_of_n=2, cluster_radius_m=0.05)

    assert {d.class_key for d in result} == {"CHESS_PIECE", "GABE"}
    assert len(result) == 2


def test_same_frame_never_double_counts_toward_frames_seen():
    """한 프레임 안 검출 2개는 서로 다른 물체로 취급돼야 한다 — 실수로
    같은 클러스터에 둘 다 들어가면 frames_seen이 부풀어 오검출이 통과한다."""
    frames = [
        [
            RawDetection("GABE", x=0.10, y=0.00, yaw_rad=0.0, confidence=0.8),
            RawDetection("GABE", x=0.12, y=0.00, yaw_rad=0.0, confidence=0.8),
        ]
    ]

    result = consensus_detections(frames, k_of_n=1, cluster_radius_m=0.05)

    assert len(result) == 2
    assert all(d.frames_seen == 1 for d in result)


def test_empty_frames_returns_empty_list():
    assert consensus_detections([], k_of_n=1) == []
    assert consensus_detections([[], [], []], k_of_n=1) == []
