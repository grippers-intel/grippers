"""Ros2Perception — mission_orchestrator가 쓰는 Perception 포트 구현.
perception_node에 서비스로 말을 건다.

⚠️ domain.values ↔ ROS2 메시지 변환은 반드시 여기(그리고 이 파일의 형제
어댑터들)에서만 한다. domain.values 인스턴스를 ROS2 메시지 생성자 자리에
그대로 넘기면 안 된다 — geometry_msgs/Pose2D 같은 rclpy 메시지는 자기
필드 타입을 assert로 검사하므로, domain.values.Pose2D를 그 자리에 그대로
넘기면 런타임에 AssertionError가 난다. 필드 하나하나를 명시적으로
꺼내 옮긴다."""

from grippers_interfaces.srv import FindBox, MeasureOpening, MonitorClearance, ScanFloor

from domain.adapters.real._ros_call import SAFETY_TIMEOUT_SEC, call_service
from domain.adapters.real._ros_convert import box_observation_from_msg, box_observation_to_msg
from domain.ports.perception import Perception
from domain.values import (
    BoxColor,
    BoxObservation,
    Clearance,
    Detection,
    ObjectClass,
    Point3,
    ScanResult,
)


def _blind_clearance() -> Clearance:
    """관측하지 못했을 때 돌려줄 여유 공간. "모르면 멈춘다"가 monitor_clearance의
    계약이므로 거리는 0(= 장애물이 바로 앞), contact_risk는 True다.

    `Clearance` 는 frozen이 아니라 모듈 상수로 공유하면 호출자가 실수로 바꿀 수
    있으므로 매번 새로 만든다."""
    return Clearance(front_m=0.0, left_m=0.0, right_m=0.0, contact_risk=True)


def _detection_from_msg(msg) -> Detection:
    return Detection(
        track_id=msg.track_id,
        cls=ObjectClass[msg.cls],
        pose_m=Point3(x=msg.pose.x, y=msg.pose.y, z=msg.pose.z),
        dims_m=Point3(x=msg.dims.x, y=msg.dims.y, z=msg.dims.z),
        yaw_rad=msg.yaw_rad,
        confidence=msg.confidence,
    )


class Ros2Perception(Perception):
    def __init__(self, node):
        self._node = node
        self._scan_client = node.create_client(ScanFloor, "perception/scan_floor")
        self._find_box_client = node.create_client(FindBox, "perception/find_box")
        self._measure_opening_client = node.create_client(
            MeasureOpening, "perception/measure_opening"
        )
        self._clearance_client = node.create_client(
            MonitorClearance, "perception/monitor_clearance"
        )

    def scan_floor(self) -> ScanResult:
        """관측 결과. 서비스가 없거나 응답이 없으면 **`ScanResult.unavailable()`** —
        빈 목록으로 삼키지 않는다 (이슈 #194). 응답이 왔는데 검출이 0개인 것은
        정상 관측이므로 `observed(())` 다.

        `call_service()` 는 **서비스 부재와 응답 없음을 둘 다 `None`** 으로
        돌려주고 구분은 그쪽 경고 로그(`scan_floor: 서비스 없음 …` /
        `scan_floor: 응답 없음 …`)에만 남는다. 여기서 두 경우를 갈라 보려면
        공통 호출 API 를 바꿔야 하는데 그건 이 이슈의 범위를 넘는다 — 그래서
        `reason` 은 **아는 만큼만** 적고 상세는 그 로그를 가리킨다."""
        res = call_service(self._node, self._scan_client, ScanFloor.Request(), label="scan_floor")
        if res is None:
            return ScanResult.unavailable(
                "scan_floor 서비스가 응답하지 않음 (부재 또는 타임아웃) — "
                "상세는 call_service 경고 로그"
            )
        return ScanResult.observed(_detection_from_msg(d) for d in res.detections.detections)

    def find_box(self, color: BoxColor) -> BoxObservation | None:
        """찾지 못했거나 서비스가 응답하지 않으면 **None** — `TRANSPORT` 가
        대상을 보류 등록하고 `SCAN` 으로 복귀한다."""
        req = FindBox.Request(color=color.name)
        res = call_service(self._node, self._find_box_client, req, label="find_box")
        if res is None or not res.found:
            return None
        return box_observation_from_msg(res.box)

    def measure_opening(self, box: BoxObservation) -> float | None:
        """입구 폭(mm). 서비스가 없거나 응답이 없으면 **None**(해 없음 취급) —
        `POSE_PLAN` 이 `REJECT` 로 보낸다. 입구 폭을 모르는 채로 투입을 시도하면
        상자 테두리에 물체를 찍는다."""
        req = MeasureOpening.Request(box=box_observation_to_msg(box))
        res = call_service(self._node, self._measure_opening_client, req, label="measure_opening")
        if res is None:
            return None
        return res.opening_mm

    def monitor_clearance(self) -> Clearance:
        """여유 공간. 서비스가 없거나 응답이 없으면 **`contact_risk=True`** —
        "모르면 멈춘다"가 이 포트의 계약이다.
        타임아웃을 통과 신호로 두면 실제 장애물을 못 보고 밀고 지나가는 사고로
        직결된다.

        상한도 여기만 `SAFETY_TIMEOUT_SEC`(0.5초)로 짧다 — `INSERT` 중 반복
        호출되는 안전 판정이라, 일반 서비스와 같은 3초를 기다리면 베이스가
        움직이는 도중 3초간 판단이 멈춘다. 안전 장치가 오히려 위험 요인이 된다."""
        res = call_service(
            self._node,
            self._clearance_client,
            MonitorClearance.Request(),
            label="monitor_clearance",
            timeout_sec=SAFETY_TIMEOUT_SEC,
        )
        if res is None:
            return _blind_clearance()
        # MonitorClearance.srv에는 top 필드도 있지만 domain.values.Clearance에는
        # 없다 — 서버 쪽(perception_node, #9 범위 밖) 인터페이스는 그대로 두고
        # 여기서만 조용히 버린다.
        return Clearance(
            front_m=res.front,
            left_m=res.left,
            right_m=res.right,
            contact_risk=res.contact_risk,
        )
