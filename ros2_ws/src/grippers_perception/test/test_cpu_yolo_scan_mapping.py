"""cpu_yolo_scan_mapping.object_class_for_cpu_yolo_class_name() 순수 로직 테스트.

hailo_scan_mapping과 같은 이유로 rclpy 없이도 돈다."""

import pytest
from grippers_perception.cpu_yolo_scan_mapping import (
    CPU_YOLO_CLASS_NAMES,
    CPU_YOLO_CLASS_TO_OBJECT_CLASS,
    object_class_for_cpu_yolo_class_name,
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
    assert object_class_for_cpu_yolo_class_name(class_name) == expected


def test_box_class_is_excluded_pending_team_confirmation():
    assert object_class_for_cpu_yolo_class_name("box") is None


def test_unknown_class_name_returns_none():
    assert object_class_for_cpu_yolo_class_name("container") is None
    assert object_class_for_cpu_yolo_class_name("cube") is None


def test_every_mapped_class_name_is_a_real_model_class():
    assert set(CPU_YOLO_CLASS_TO_OBJECT_CLASS).issubset(set(CPU_YOLO_CLASS_NAMES))


def test_mapped_values_are_known_object_classes():
    assert set(CPU_YOLO_CLASS_TO_OBJECT_CLASS.values()) <= {"GABE", "CHESS_PIECE"}
