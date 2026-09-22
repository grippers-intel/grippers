"""vla_robot 전체 기동.

    ros2 launch vla_robot_bringup robot.launch.py
    ros2 launch vla_robot_bringup robot.launch.py config:=/path/robot.yaml use_policy:=false

설정은 전부 config(robot.yaml) 한 파일에 있다. 여기 인자는 "어떤 노드를 띄울지"만 정한다.

벤더 스택에서는 odom_publisher.launch.py 하나만 포함한다(cmd_vel -> 바퀴).
벤더 bringup.launch.py 를 통째로 쓰지 않는 이유: 자율주행/조이스틱 노드가 같은 cmd_vel 에
동시에 발행해 경쟁한다. controller.launch.py 는 imu_calib 부재로 launch 전체를 죽인
기록이 있다(기존 HANDOFF 2026-08-23).
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _vendor_base(context):
    path = LaunchConfiguration("vendor_odom_launch").perform(context)
    if not path:
        try:
            path = os.path.join(get_package_share_directory("controller"), "launch", "odom_publisher.launch.py")
        except PackageNotFoundError:
            path = "/home/ubuntu/ros2_ws/src/driver/controller/launch/odom_publisher.launch.py"
    if not os.path.exists(path):
        return [LogInfo(msg=f"[vla_robot] 벤더 odom_publisher launch 를 찾지 못했다: {path} — 차체가 움직이지 않는다")]
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(path))]


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory("vla_robot_bringup"), "config", "robot.yaml")
    config = LaunchConfiguration("config")
    params = [{"config_file": config}]

    def node(pkg, exe, flag):
        return Node(package=pkg, executable=exe, name=exe, output="screen", parameters=params,
                    condition=IfCondition(LaunchConfiguration(flag)), respawn=False)

    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config, description="robot.yaml 경로"),
        DeclareLaunchArgument("use_base", default_value="true", description="벤더 odom_publisher 포함"),
        DeclareLaunchArgument("vendor_odom_launch", default_value="",
                              description="벤더 odom_publisher.launch.py 경로 (비우면 자동)"),
        DeclareLaunchArgument("use_arm", default_value="true"),
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("use_policy", default_value="true"),
        DeclareLaunchArgument("use_mission", default_value="true"),
        OpaqueFunction(function=_vendor_base, condition=IfCondition(LaunchConfiguration("use_base"))),
        node("vla_robot_arm", "arm_driver_node", "use_arm"),
        node("vla_robot_vision", "gripper_cam_node", "use_camera"),
        node("vla_robot_policy", "vla_grasp_node", "use_policy"),
        node("vla_robot_mission", "pi_mission_node", "use_mission"),
    ])
