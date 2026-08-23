import rclpy
from rclpy.node import Node

from domain.adapters.real.ros2_mecanum_base import Ros2MecanumBase
from domain.adapters.real.ros2_perception import Ros2Perception
from domain.task.mission_task import Ports
from domain.task.states import ApproachState, ScanState, SelectState
from domain.values import MissionContext, MissionMode, MissionSpec, ObjectClass

rclpy.init()
node = Node("approach_test_node")
ports = Ports(
    base=Ros2MecanumBase(node),
    arm=None,
    perception=Ros2Perception(node),
    interpreter=None,
    estop=None,
)

spec = MissionSpec(mode=MissionMode.TIDY, target_cls=None, placement_rule={ObjectClass.CHESS_PIECE: {}}, raw_text="")
ctx = MissionContext(spec=spec)

state = ScanState(ctx)
print(f"SCAN -> ", end="")
state = state.execute(ports)
print(type(state).name)

if type(state).__name__ != "SelectState":
    print("SELECT까지 못 감, 종료")
else:
    print(f"SELECT -> ", end="")
    state = state.execute(ports)
    print(type(state).name)

    if type(state).__name__ == "ApproachState":
        target = state.target
        print(f"target track_id={target.track_id} cls={target.cls} pose={target.pose_m} yaw={target.yaw_rad}")
        print("APPROACH.execute() 호출 -- 실제 베이스 주행 시작")
        next_state = state.execute(ports)
        print(f"APPROACH -> {type(next_state).name}")
    else:
        print("APPROACH까지 못 감 (후보 없음 -> DONE)")

node.destroy_node()
rclpy.shutdown()
