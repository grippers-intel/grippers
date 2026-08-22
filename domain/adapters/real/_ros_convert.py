"""domain.values ↔ ROS2 메시지 변환 헬퍼. Ros2Perception과 Ros2MecanumBase가
둘 다 BoxObservation을 주고받아야 해서(find_box/measure_opening/align_to_box
전부 "이 상자" 하나를 지칭) 여기 한 곳에만 둔다 — 각 어댑터 파일에 흩어 두면
필드가 바뀔 때 두 곳을 같이 고쳐야 하는데 하나를 놓치기 쉽다.

⚠️ domain.values 인스턴스를 ROS2 메시지 생성자 자리에 그대로 넘기면 안 된다 —
rclpy 메시지는 필드 타입을 assert로 검사하므로 런타임 AssertionError가 난다.
"""

from geometry_msgs.msg import Pose2D as RosPose2D
from grippers_interfaces.msg import BoxObservation as RosBoxObservation

from domain.values import BoxObservation, Destination, Pose2D

# ⚠️ 2026-08-23: domain.values.BoxObservation.color(BoxColor)가 .dest(Destination)로
# 바뀌었지만, 와이어 메시지(BoxObservation.msg)의 필드명은 아직 `color`로 남아
# 있다 — .msg 필드명을 바꾸면 인터페이스 재빌드와 이걸 쓰는 다른 노드/브랜치가
# 전부 같이 바뀌어야 해서, 그 조율은 팀과 따로 하고 여기서는 같은 문자열 필드에
# Destination의 이름("LEFT"/"RIGHT")을 담는 것으로만 맞춘다.


def box_observation_from_msg(msg: RosBoxObservation) -> BoxObservation:
    return BoxObservation(
        dest=Destination[msg.color],
        pose_m=Pose2D(x=msg.pose.x, y=msg.pose.y, theta=msg.pose.theta),
        opening_mm=msg.opening_mm,
        long_axis_rad=msg.long_axis_rad,
    )


def box_observation_to_msg(box: BoxObservation) -> RosBoxObservation:
    return RosBoxObservation(
        color=box.dest.name,
        pose=RosPose2D(x=box.pose_m.x, y=box.pose_m.y, theta=box.pose_m.theta),
        opening_mm=box.opening_mm,
        long_axis_rad=box.long_axis_rad,
    )
