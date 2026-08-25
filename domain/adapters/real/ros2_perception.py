"""Ros2Perception — mission_orchestrator가 쓰는 Perception 포트 구현.
perception_node에 서비스로 말을 건다.

⚠️ domain.values ↔ ROS2 메시지 변환은 반드시 여기(그리고 이 파일의 형제
어댑터들)에서만 한다. domain.values 인스턴스를 ROS2 메시지 생성자 자리에
그대로 넘기면 안 된다 — geometry_msgs/Pose2D 같은 rclpy 메시지는 자기
필드 타입을 assert로 검사하므로, domain.values.Pose2D를 그 자리에 그대로
넘기면 런타임에 AssertionError가 난다. 필드 하나하나를 명시적으로
꺼내 옮긴다."""

from grippers_interfaces.srv import (
    FindBox,
    MeasureOpening,
    MonitorClearance,
    ObserveTarget,
    ScanFloor,
)

from domain.adapters.real._ros_call import SAFETY_TIMEOUT_SEC, call_service
from domain.adapters.real._ros_convert import box_observation_from_msg, box_observation_to_msg
from domain.ports.perception import Perception
from domain.values import BoxObservation, Clearance, Destination, Detection, ObjectClass, Point3


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
        self._observe_client = node.create_client(ObserveTarget, "perception/observe_target")
        # GRASP 직전에 기억해 둔 목표 관측 (remember_target -> confirm_grasp)
        self._remembered: tuple[str, float, float] | None = None

    def scan_floor(self) -> list[Detection]:
        """검출 목록. 서비스가 없거나 응답이 없으면 **빈 목록** — `SELECT` 가
        고를 후보가 없어 `DONE` 으로 간다. 관측이 안 되는데 계속 도는 것보다
        미션을 끝내고 이유를 로그로 남기는 편이 낫다."""
        res = call_service(self._node, self._scan_client, ScanFloor.Request(), label="scan_floor")
        if res is None:
            return []
        return [_detection_from_msg(d) for d in res.detections.detections]

    def find_box(self, dest: Destination) -> BoxObservation | None:
        """찾지 못했거나 서비스가 응답하지 않으면 **None** — `TRANSPORT` 가
        대상을 보류 등록하고 `SCAN` 으로 복귀한다.

        ⚠️ FindBox.srv의 필드명은 아직 `color`다(_ros_convert.py 상단 경고와
        같은 이유로 와이어 인터페이스는 이번 변경 범위 밖) — Destination의
        이름("LEFT"/"RIGHT")을 그 문자열 필드에 담아 보낸다."""
        req = FindBox.Request(color=dest.name)
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

    # ---- 파지 확인: "그 자리에 있던 것이 사라졌는가" -----------------------
    #
    # 임계값 근거. 두 관측 사이에 로봇은 미세 전진으로 몇 cm 앞으로 간다 —
    # 물체가 그대로 있다면 더 **가까워져** bbox 높이가 오히려 커진다. 그래서
    # "여전히 있다"는 h_after >= h_before * STILL_THERE_H_RATIO 로 잡는다.
    # 비율을 1.0 이 아니라 0.8 로 두는 것은 관측 잡음과 살짝 밀린 경우까지
    # 실패로 보기 위해서다 — 이 방향의 오판(성공인데 실패라고 함)은 사람이
    # 확인하게 만들 뿐이지만, 반대 방향은 빈 그리퍼로 미션을 계속하게 한다.
    STILL_THERE_H_RATIO = 0.8

    def remember_target(self, raw_cls: str) -> bool:
        res = call_service(
            self._node, self._observe_client,
            ObserveTarget.Request(raw_cls=raw_cls), label="remember_target")
        if res is None or not res.found or res.h <= 0.0:
            self._remembered = None
            self._node.get_logger().warn(
                f"[remember_target] {raw_cls} 관측 실패 — 파지 확인을 쓸 수 없다")
            return False
        self._remembered = (raw_cls, float(res.h), float(res.x))
        self._node.get_logger().info(
            f"[remember_target] {raw_cls} h={res.h:.1f}px x={res.x:.1f}px")
        return True

    def confirm_grasp(self) -> bool:
        if self._remembered is None:
            self._node.get_logger().warn("[confirm_grasp] 기준 관측이 없다 — False")
            return False
        raw_cls, h_before, _x_before = self._remembered
        res = call_service(
            self._node, self._observe_client,
            ObserveTarget.Request(raw_cls=raw_cls), label="confirm_grasp")
        if res is None:
            self._node.get_logger().warn("[confirm_grasp] 관측 응답 없음 — False")
            return False
        if not res.found:
            self._node.get_logger().info(
                f"[confirm_grasp] {raw_cls} 가 정면에서 사라졌다 "
                f"(before h={h_before:.1f}px) — 파지 성공으로 본다")
            return True
        still_there = res.h >= h_before * self.STILL_THERE_H_RATIO
        self._node.get_logger().info(
            f"[confirm_grasp] {raw_cls} h={res.h:.1f}px (before {h_before:.1f}px) -> "
            + ("그대로 있다 — 파지 실패" if still_there
               else "멀리 있는 다른 개체 — 파지 성공으로 본다"))
        return not still_there
