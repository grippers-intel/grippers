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
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import UnlessCondition
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

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_package_path, "launch/controller.launch.py")
        ),
    )
    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, "launch/depth_camera.launch.py")
        ),
    )
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, "launch/lidar.launch.py")
        ),
    )

    use_fake_arm = LaunchConfiguration("use_fake_arm")
    use_fake_perception = LaunchConfiguration("use_fake_perception")
    use_fake_interpreter = LaunchConfiguration("use_fake_interpreter")
    arm_port = LaunchConfiguration("arm_port")

    # use_fake_* 는 세 지점이 모두 맞아야 실제로 동작한다: 여기서 선언
    # (generate_launch_description)하고, 하드웨어 노드를 UnlessCondition으로 끄고,
    # mission_orchestrator에 파라미터로 넘겨 어댑터 분기를 시킨다. 하나라도 빠지면
    # ROS2가 선언되지 않은 launch 인자를 조용히 버리기 때문에 "껐다고 믿었는데
    # 실물이 돌아가는" 상태가 된다.
    perception_node = Node(
        package="grippers_perception",
        executable="perception_node",
        output="screen",
        condition=UnlessCondition(use_fake_perception),
    )
    arm_driver_node = Node(
        package="grippers_arm",
        executable="arm_driver",
        output="screen",
        condition=UnlessCondition(use_fake_arm),
        parameters=[{"arm_port": arm_port}],
    )
    grippers_nodes = [
        Node(package="grippers_base", executable="base_driver", output="screen"),
        Node(
            package="grippers_mission",
            executable="mission_orchestrator",
            output="screen",
            parameters=[
                {
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
        *grippers_nodes,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
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
                "arm_port",
                # ttyACM 번호는 USB 연결 순서에 따라 바뀔 수 있으므로 udev가 만드는
                # 안정적인 심볼릭 링크를 기본값으로 사용한다. MentorPi 베이스 보드는
                # /dev/rrc, SO-ARM101은 /dev/soarm 으로 구분한다.
                # arm_driver_node도 기동 시 베이스 보드 포트 충돌을 검사한다.
                default_value="/dev/soarm",
                description="SO-ARM101 시리얼 포트 (udev 기본값: /dev/soarm)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
