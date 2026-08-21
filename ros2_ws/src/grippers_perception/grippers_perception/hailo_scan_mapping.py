"""Hailo-10H scan_floor 검출 → domain ObjectClass 매핑 (순수 함수).

`perception_node.py`에서 이 매핑 로직만 뽑아냈다 — `perception_node.py`는
`rclpy`를 최상단에서 무조건 import해서 ROS2 없이는 아예 임포트가 안 되고,
그래서 이 매핑 로직도 지금까지 테스트가 하나도 없었다(PR #185 리뷰에서
`scan_floor` 안전 게이트 누락이 리뷰 전에 안 잡힌 이유 중 하나다). 이 파일은
`rclpy`/`cv2`/`hailo_platform`을 전혀 쓰지 않으므로 도메인 테스트와 같은
방식(순수 `pytest`)으로 검증할 수 있다 — `ros2_ws/src/grippers_perception/test/`
참고.

클래스 이름·순서는 HEF 컴파일 당시 metadata.yaml과 반드시 일치해야 한다.
HEF가 바뀌면 `HAILO_CLASS_NAMES`도 같이 바꿀 것.
"""

# metadata.yaml의 names 순서 — HEF가 바뀌면 같이 바꿀 것.
HAILO_CLASS_NAMES = ["container", "knight", "queen", "rook", "box", "soccer", "star"]

# knight/queen/rook은 CHESS_PIECE로, soccer/star는 GABE로 매핑한다.
# "container"/"box"는 목적지 상자로 추정해 바닥 스캔 후보에서 제외한다 —
# 확실하지 않은 매핑을 코드에 박지 않는다. "cube"는 애초에 학습 클래스에
# 없다(미해결, docs 참고).
HAILO_CLASS_TO_OBJECT_CLASS = {
    "knight": "CHESS_PIECE",
    "queen": "CHESS_PIECE",
    "rook": "CHESS_PIECE",
    "soccer": "GABE",
    "star": "GABE",
    # "container", "box": 목적지 상자로 추정 — 바닥 스캔 후보에서 제외.
}


def object_class_for_hailo_id(class_id: int) -> str | None:
    """Hailo 추론 출력의 class_id(=`HAILO_CLASS_NAMES` 인덱스)를 domain
    `ObjectClass` 이름 문자열("GABE"/"CHESS_PIECE")로 바꾼다.

    매핑이 없으면(범위 밖 class_id, 또는 `HAILO_CLASS_TO_OBJECT_CLASS`에
    의도적으로 안 넣은 클래스) **`None`** — 호출자가 바닥 스캔 후보에서
    제외해야 한다는 신호다. 다른 Perception 계약과 같은 "모르면 제외" 관례."""
    if not 0 <= class_id < len(HAILO_CLASS_NAMES):
        return None
    class_name = HAILO_CLASS_NAMES[class_id]
    return HAILO_CLASS_TO_OBJECT_CLASS.get(class_name)
