"""Ros2Perception — mission_orchestrator가 쓰는 Perception 포트 구현.
perception_node에 서비스로 말을 건다.

⚠️ domain.values ↔ ROS2 메시지 변환은 반드시 여기(그리고 이 파일의 형제
어댑터들)에서만 한다. domain.values 인스턴스를 ROS2 메시지 생성자 자리에
그대로 넘기면 안 된다 — geometry_msgs/Pose2D 같은 rclpy 메시지는 자기
필드 타입을 assert로 검사하므로, domain.values.Pose2D를 그 자리에 그대로
넘기면 런타임에 AssertionError가 난다. 필드 하나하나를 명시적으로
꺼내 옮긴다."""

from grippers_interfaces.srv import MonitorClearance, ObserveTarget

from domain.adapters.real._ros_call import SAFETY_TIMEOUT_SEC, call_service
from domain.ports.perception import Perception
from domain.values import Clearance, TargetObservation


def _blind_clearance() -> Clearance:
    """관측하지 못했을 때 돌려줄 여유 공간. "모르면 멈춘다"가 monitor_clearance의
    계약이므로 거리는 0(= 장애물이 바로 앞), contact_risk는 True다.

    `Clearance` 는 frozen이 아니라 모듈 상수로 공유하면 호출자가 실수로 바꿀 수
    있으므로 매번 새로 만든다."""
    return Clearance(front_m=0.0, left_m=0.0, right_m=0.0, contact_risk=True)


class Ros2Perception(Perception):
    def __init__(self, node):
        self._node = node
        self._clearance_client = node.create_client(
            MonitorClearance, "perception/monitor_clearance"
        )
        self._observe_client = node.create_client(ObserveTarget, "perception/observe_target")
        # GRASP 직전에 기억해 둔 목표 관측 (remember_target -> confirm_grasp)
        self._remembered: tuple[str, float, float] | None = None

    # Pi 자기 뎁스캠이 알아볼 수 있는 raw 클래스들. perception_node의 YOLO
    # 라벨과 같아야 한다. 순서가 우선순위다 — 여러 개가 동시에 보이면
    # 앞쪽을 고른다.
    KNOWN_LABELS = ("queen", "knight", "rook", "box", "star", "soccer")

    def identify_target(self):
        """정면 물체의 raw 라벨. 못 찾으면 **None**.

        `ObserveTarget`은 "이 클래스가 보이나"를 묻는 서비스라 클래스를
        하나씩 물어본다. 서비스를 새로 만들지 않고 있는 것으로 푸는 쪽을
        택했다 — GRASP 진입 때 한 번만 도는 경로라 왕복 몇 번의 비용이
        인터페이스를 늘리는 비용보다 싸다.

        여러 개가 동시에 보이면 **가장 큰 것**을 고른다. 파지하러 내려가는
        거리에서는 목표가 화면에서 가장 크고, 배경에 걸친 다른 물체는 작게
        잡히기 때문이다.

        ⚠️ 첫 질문에만 `force_fresh=True`를 보낸다(2026-09-01, 실기 사고
        대응) — perception_node의 표본 캐시는 이 여섯 번 연속 질문이
        같은 순간을 공유하라고 있는 것이지, GRASP_ALIGN처럼 이 함수 자체가
        3초 안에 여러 번 다시 불리는 것까지 같은 표본으로 답하라는 뜻이
        아니다. 차체가 재직진하거나 servo 1로 고친 **뒤** 다시 부른
        판정 라운드가 낡은 캐시를 물려받으면 "지금은 보이는데 못 찾음"이
        된다 — 매 라운드의 첫 질문에서 캐시를 강제로 비워 그걸 막는다."""
        best, best_area = None, 0.0
        for index, label in enumerate(self.KNOWN_LABELS):
            res = call_service(
                self._node, self._observe_client,
                ObserveTarget.Request(raw_cls=label, force_fresh=(index == 0)),
                label="identify_target")
            if res is None or not res.found:
                continue
            area = float(res.h) * float(res.w)
            if area > best_area:
                best_area = area
                best = TargetObservation(
                    label=label,
                    forward_m=float(res.forward_m),
                    lateral_m=float(res.lateral_m),
                    metric_ok=bool(res.metric_ok),
                )
        return best

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
        # force_fresh=True — identify_target의 마지막 질문 이후로도 시간이
        # 흘렀을 수 있는, 별도의 판정 라운드다(같은 이유는 identify_target
        # docstring 참고).
        res = call_service(
            self._node, self._observe_client,
            ObserveTarget.Request(raw_cls=raw_cls, force_fresh=True),
            label="remember_target")
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
        # force_fresh=True — 팔이 파지를 시도한 **뒤**의 판정이라, remember_
        # target 때 찍은 낡은 캐시를 다시 받으면 "그대로 있다"를 자기 자신과
        # 비교하는 꼴이 된다(캐시 유효창 3초 안에서는 실제로 벌어질 수 있는
        # 경로다 — GRASP 시퀀스는 그보다 빠르다).
        res = call_service(
            self._node, self._observe_client,
            ObserveTarget.Request(raw_cls=raw_cls, force_fresh=True),
            label="confirm_grasp")
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
