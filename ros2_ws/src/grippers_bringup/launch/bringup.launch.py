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

    use_fake_perception = LaunchConfiguration("use_fake_perception")

    perception_node = Node(
        package="grippers_perception",
        executable="perception_node",
        output="screen",
        condition=UnlessCondition(use_fake_perception),
    )
    grippers_nodes = [
        Node(package="grippers_base", executable="base_driver", output="screen"),
        Node(package="grippers_arm", executable="arm_driver", output="screen"),
        Node(
            package="grippers_mission",
            executable="mission_orchestrator",
            output="screen",
            parameters=[{"use_fake_perception": use_fake_perception}],
        ),
    ]

    return [
        controller_launch,
        depth_camera_launch,
        lidar_launch,
        perception_node,
        *grippers_nodes,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_fake_perception",
                default_value="true",
                description="true면 카메라 하드웨어 없이 FakePerception 사용",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
