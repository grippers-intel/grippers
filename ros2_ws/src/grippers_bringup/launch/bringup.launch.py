"""grippers_bringup — MentorPi 저수준 드라이버(controller/peripherals) 위에
grippers_base/arm/perception/mission 노드를 얹는다.
대회용 bringup.launch.py를 통째로 쓰지 않는 이유: start_app_launch(자율주행/트래킹)와
joystick_control_launch가 같은 /cmd_vel에 동시에 publish하면 grippers_mission과
경쟁 상태가 생기기 때문. 필요한 하위 launch만 골라서 재조합한다."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    compiled = os.environ.get("need_compile", "False")
    if compiled == "True":
        controller_package_path = get_package_share_directory("controller")
        peripherals_package_path = get_package_share_directory("peripherals")
    else:
        controller_package_path = "/home/ubuntu/ros2_ws/src/driver/controller"
        peripherals_package_path = "/home/ubuntu/ros2_ws/src/peripherals"

    use_fake_base = LaunchConfiguration("use_fake_base")
    use_fake_arm = LaunchConfiguration("use_fake_arm")
    use_fake_perception = LaunchConfiguration("use_fake_perception")
    use_fake_interpreter = LaunchConfiguration("use_fake_interpreter")
    scan_floor_enabled = LaunchConfiguration("scan_floor_enabled")
    record_bag = LaunchConfiguration("record_bag")
    bag_output = LaunchConfiguration("bag_output")
    arm_port = LaunchConfiguration("arm_port")

    # ⚠️ 2026-08-23: controller.launch.py를 그대로 쓰지 않는다 — HANDOFF.md
    # 실기 확인: 이 launch가 포함하는 imu_filter.launch.py가 `imu_calib`
    # 패키지 부재로 SIGINT를 내며 launch 전체를 죽인다. 팀원이 실기로 검증한
    # 우회로 그대로 odom_publisher.launch.py만 직접 포함한다 — 대신 EKF가
    # 없어 /odom이 비어 있다. 2026-08-26 팀 확정 이후 Pi에는 주행 판단이
    # 없으므로(Host가 속도를 직접 보낸다) 이 노드는 cmd_vel을 바퀴로
    # 내보내는 역할만 한다.
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_package_path, "launch/odom_publisher.launch.py")
        ),
        condition=UnlessCondition(use_fake_base),
    )
    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, "launch/depth_camera.launch.py")
        ),
        condition=UnlessCondition(use_fake_perception),
    )
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, "launch/lidar.launch.py")
        ),
        condition=UnlessCondition(use_fake_perception),
    )

    # use_fake_* 는 launch 인자 선언, 하드웨어 노드 guard, orchestrator 파라미터가
    # 모두 맞아야 한다. 하나라도 빠지면 "껐다고 믿었는데 실물이 돌아가는" 상태가
    # 되거나 real 어댑터의 서비스 서버가 뜨지 않는다.
    perception_node = Node(
        package="grippers_perception",
        executable="perception_node",
        output="screen",
        condition=UnlessCondition(use_fake_perception),
        parameters=[{"scan_floor_enabled": scan_floor_enabled}],
    )
    arm_driver_node = Node(
        package="grippers_arm",
        executable="arm_driver",
        output="screen",
        condition=UnlessCondition(use_fake_arm),
        parameters=[{"arm_port": arm_port}],
    )
    bag_recorder = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-a", "-o", bag_output],
        output="screen",
        condition=IfCondition(record_bag),
    )
    grippers_nodes = [
        Node(
            package="grippers_mission",
            executable="mission_orchestrator",
            output="screen",
            parameters=[
                {
                    "use_fake_base": use_fake_base,
                    "use_fake_arm": use_fake_arm,
                    "use_fake_perception": use_fake_perception,
                    "use_fake_interpreter": use_fake_interpreter,
                }
            ],
        ),
    ]

    return [
        controller_launch,
        depth_camera_launch,
        lidar_launch,
        perception_node,
        arm_driver_node,
        bag_recorder,
        *grippers_nodes,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_fake_base",
                default_value="true",
                description="true면 controller 없이 FakeBase 사용",
            ),
            DeclareLaunchArgument(
                "use_fake_arm",
                default_value="true",
                description="true면 SO-ARM101 하드웨어 없이 FakeArm 사용",
            ),
            DeclareLaunchArgument(
                "use_fake_perception",
                default_value="true",
                description="true면 카메라 하드웨어 없이 FakePerception 사용",
            ),
            DeclareLaunchArgument(
                "use_fake_interpreter",
                default_value="true",
                description="true면 language 노드 없이 ScriptedInterpreter 사용",
            ),
            DeclareLaunchArgument(
                "scan_floor_enabled",
                default_value="false",
                description="true면 perception_node의 scan_floor 안전 게이트를 연다 "
                "(perception_node.py SCAN_FLOOR_ENABLED_DEFAULT 경고 참고 — "
                "실기 SCAN→SELECT→APPROACH 경로를 확인할 때만 명시적으로 켤 것)",
            ),
            DeclareLaunchArgument(
                "arm_port",
                # ttyACM 번호는 USB 연결 순서에 따라 바뀔 수 있으므로 udev가 만드는
                # 안정적인 심볼릭 링크를 기본값으로 사용한다. MentorPi 베이스 보드는
                # /dev/rrc, SO-ARM101은 /dev/soarm 으로 구분한다.
                # arm_driver_node도 기동 시 베이스 보드 포트 충돌을 검사한다.
                default_value="/dev/soarm",
                description="SO-ARM101 시리얼 포트 (udev 기본값: /dev/soarm)",
            ),
            DeclareLaunchArgument(
                "record_bag",
                default_value="false",
                description="true면 ros2 bag record -a로 전체 토픽 녹화",
            ),
            DeclareLaunchArgument(
                "bag_output",
                default_value="/tmp/grippers_mission_bag",
                description="rosbag 출력 디렉터리",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
