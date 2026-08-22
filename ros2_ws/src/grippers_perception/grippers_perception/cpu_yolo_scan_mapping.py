"""CPU YOLO(ultralytics) scan_floor 검출 → domain ObjectClass 매핑 (순수 함수).

`hailo_scan_mapping.py`와 같은 이유로 뽑아냈다 — `perception_node.py`는 rclpy를
무조건 import해서 ROS2 없이는 임포트가 안 되므로, 이 매핑 로직만 따로 둬야
`rclpy` 없이 순수 pytest로 검증할 수 있다.

⚠️ 2026-08-22: AI HAT+2(Hailo-10H)가 부팅마다 SOC 펌웨어 stage 2에서 멈추는
증상으로 하드웨어 고장(온보드 DDR 손상 의심 — Hailo 커뮤니티의 동일 증상
스레드가 그렇게 결론남, 이슈 #189 참고)이 확인돼, 새 보드 도착 전까지 CPU
YOLO(ultralytics, `260822_5k_640_hailo_model/weights/best.pt`)로 임시 대체한다.

이 모델은 `hailo_scan_mapping.py`가 쓰던 모델과 **클래스 구성이 다르다** —
"container"가 빠지고 6종(`knight/queen/rook/box/soccer/star`)이다. `box`가
이제 유일한 상자 후보 클래스로 보이지만 팀 확인 전이라 여전히 바닥 스캔
후보에서 제외해둔다(불확실한 매핑을 코드에 박지 않는다는 기존 원칙 유지).
"""

# dataset/data.yaml의 names 순서 — 모델이 바뀌면 같이 바꿀 것.
CPU_YOLO_CLASS_NAMES = ["knight", "queen", "rook", "box", "soccer", "star"]

# knight/queen/rook은 CHESS_PIECE로, soccer/star는 GABE로 매핑한다.
# "box"는 상자 후보로 추정되나 미확정이라 바닥 스캔 후보에서 제외한다.
# "cube"는 이번 모델에도 학습 클래스에 없다(기존과 동일한 미해결 갭).
CPU_YOLO_CLASS_TO_OBJECT_CLASS = {
    "knight": "CHESS_PIECE",
    "queen": "CHESS_PIECE",
    "rook": "CHESS_PIECE",
    "soccer": "GABE",
    "star": "GABE",
    # "box": 상자 후보로 추정 — 확정 전까지 바닥 스캔 후보에서 제외.
}


def object_class_for_cpu_yolo_class_name(class_name: str) -> str | None:
    """ultralytics 추론 결과의 클래스 이름을 domain `ObjectClass` 이름 문자열
    ("GABE"/"CHESS_PIECE")로 바꾼다.

    매핑이 없으면(모르는 이름, 또는 의도적으로 제외한 클래스) **`None`** —
    호출자가 바닥 스캔 후보에서 제외해야 한다는 신호다. hailo_scan_mapping의
    "모르면 제외" 관례와 동일하다."""
    return CPU_YOLO_CLASS_TO_OBJECT_CLASS.get(class_name)
