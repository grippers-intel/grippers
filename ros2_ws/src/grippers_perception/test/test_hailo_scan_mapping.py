"""hailo_scan_mapping.object_class_for_hailo_id() 순수 로직 테스트.

rclpy/cv2/hailo_platform 없이도 돈다 — perception_node.py 자체는 rclpy를
무조건 import해서 ROS2 없이는 임포트가 안 되지만, 이 매핑 로직은 그 파일에서
뽑아냈으므로(2026-08-22) 여기서만 순수 pytest로 검증한다."""

import pytest
from grippers_perception.hailo_scan_mapping import (
    HAILO_CLASS_NAMES,
    HAILO_CLASS_TO_OBJECT_CLASS,
    object_class_for_hailo_id,
)


@pytest.mark.parametrize(
    "class_name,expected",
    [
        ("knight", "CHESS_PIECE"),
        ("queen", "CHESS_PIECE"),
        ("rook", "CHESS_PIECE"),
        ("soccer", "GABE"),
        ("star", "GABE"),
    ],
)
def test_known_classes_map_to_expected_object_class(class_name, expected):
    class_id = HAILO_CLASS_NAMES.index(class_name)
    assert object_class_for_hailo_id(class_id) == expected


@pytest.mark.parametrize("class_name", ["container", "box"])
def test_destination_box_classes_are_excluded(class_name):
    """container/box는 목적지 상자로 추정 — 바닥 스캔 후보에서 제외돼야 한다."""
    class_id = HAILO_CLASS_NAMES.index(class_name)
    assert object_class_for_hailo_id(class_id) is None


def test_out_of_range_class_id_returns_none():
    """HEF가 바뀌어 클래스 수가 달라져도 죽지 않고 제외로 접는다."""
    assert object_class_for_hailo_id(len(HAILO_CLASS_NAMES)) is None
    assert object_class_for_hailo_id(-1) is None


def test_every_mapped_class_name_is_a_real_hailo_class():
    """HAILO_CLASS_TO_OBJECT_CLASS의 키가 HAILO_CLASS_NAMES에 없는 이름으로
    오타 나는 걸 잡는다 — 오타가 나면 조용히 매핑이 안 먹는다."""
    assert set(HAILO_CLASS_TO_OBJECT_CLASS).issubset(set(HAILO_CLASS_NAMES))


def test_mapped_values_are_known_object_classes():
    """domain.values.ObjectClass는 GABE/CHESS_PIECE 둘뿐이다 — 이 문자열이
    그 두 값과 어긋나면 Ros2Perception이 만든 Detection이 도메인에서 조용히
    거부되거나 잘못 해석된다."""
    assert set(HAILO_CLASS_TO_OBJECT_CLASS.values()) <= {"GABE", "CHESS_PIECE"}
