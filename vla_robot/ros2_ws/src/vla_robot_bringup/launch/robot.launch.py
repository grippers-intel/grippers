"""vla_robot 전체 기동.

    ros2 launch vla_robot_bringup robot.launch.py
    ros2 launch vla_robot_bringup robot.launch.py config:=/path/robot.yaml use_policy:=false

설정은 전부 config(robot.yaml) 한 파일에 있다. 여기 인자는 "어떤 노드를 띄울지"만 정한다.

## ⚠️ ros_robot_controller 는 기본으로 띄우지 않는다 (use_vendor_controller:=false)

MentorPi 는 **부팅 때 `ros_robot_controller` 를 자동 실행한다.** 여기서 하나를 더 띄우면
컨트롤러가 둘이 되고, 그러면 **바퀴가 안 돈다.** 더 나쁜 것은 `/odom_raw` 가 명령을 그대로
되읽는 추측항법이라 "정상적으로 돌고 있다"고 보고한다는 점이다 — 로그만 보면 멀쩡해 보인다.
팀이 이것으로 몇 시간을 태웠고(2026-09-07), 중복 컨트롤러를 없애자마자 바퀴가 돌았다.

그래서 벤더 `odom_publisher.launch.py` 를 통째로 포함하지 않는다 — 그 런치가
`ros_robot_controller.launch.py` 를 같이 띄우기 때문이다. 필요한 `odom_publisher` 노드만
직접 띄우고, 컨트롤러는 부팅 자동 실행분을 쓴다.

부팅 자동 실행분이 없는 환경(직접 정리했거나 서비스를 껐을 때)에서만
`use_vendor_controller:=true` 로 켠다. 기동 전에 `tools/ops/pi_preflight.sh` 로 몇 개가
떠 있는지 먼저 확인할 것.
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

#: 벤더 패키지를 소스 트리에서 찾을 때의 후보 경로 (need_compile=False 구성).
VENDOR_SRC_CANDIDATES = (
    "/home/ubuntu/ros2_ws/src/driver",
    "/ros2_ws/src/driver",
)


def _vendor_dir(package: str, relative: str) -> str:
    """설치된 share 를 먼저 보고, 없으면 소스 트리에서 찾는다. 없으면 빈 문자열."""
    try:
        path = os.path.join(get_package_share_directory(package), relative)
        if os.path.exists(path):
            return path
    except PackageNotFoundError:
        pass
    for root in VENDOR_SRC_CANDIDATES:
        path = os.path.join(root, package, relative)
        if os.path.exists(path):
            return path
    return ""


def _base(context):
    """odom_publisher 노드만 띄운다. 컨트롤러는 부팅 자동 실행분을 쓴다(위 주석 참고)."""
    params_file = _vendor_dir("controller", "config/calibrate_params.yaml")
    if not params_file:
        return [LogInfo(msg="[vla_robot] controller/config/calibrate_params.yaml 을 찾지 못했다 — "
                            "차체가 움직이지 않는다. 벤더 워크스페이스를 source 했는지 확인할 것")]
    return [Node(
        package="controller",
        executable="odom_publisher",
        name="odom_publisher",
        output="screen",
        parameters=[params_file, {
            "base_frame_id": "base_footprint",
            "odom_frame_id": "odom",
            "pub_odom_topic": True,
        }],
    )]


def _vendor_controller(context):
    launch_file = _vendor_dir("ros_robot_controller", "launch/ros_robot_controller.launch.py")
    if not launch_file:
        return [LogInfo(msg="[vla_robot] ros_robot_controller launch 를 찾지 못했다")]
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file))]


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory("vla_robot_bringup"), "config", "robot.yaml")
    config = LaunchConfiguration("config")
    params = [{"config_file": config}]

    def node(pkg, exe, flag):
        return Node(package=pkg, executable=exe, name=exe, output="screen", parameters=params,
                    condition=IfCondition(LaunchConfiguration(flag)), respawn=False)

    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config, description="robot.yaml 경로"),
        DeclareLaunchArgument("use_base", default_value="true",
                              description="벤더 odom_publisher 노드 (cmd_vel -> 바퀴)"),
        DeclareLaunchArgument("use_vendor_controller", default_value="false",
                              description="ros_robot_controller 도 띄운다. **부팅 자동 실행분이 없을 때만** "
                                          "— 둘이 되면 바퀴가 안 돈다(런치 파일 주석 참고)"),
        DeclareLaunchArgument("use_arm", default_value="true"),
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("use_policy", default_value="true"),
        DeclareLaunchArgument("use_mission", default_value="true"),
        OpaqueFunction(function=_vendor_controller,
                       condition=IfCondition(LaunchConfiguration("use_vendor_controller"))),
        OpaqueFunction(function=_base, condition=IfCondition(LaunchConfiguration("use_base"))),
        node("vla_robot_arm", "arm_driver_node", "use_arm"),
        node("vla_robot_vision", "gripper_cam_node", "use_camera"),
        node("vla_robot_policy", "vla_grasp_node", "use_policy"),
        node("vla_robot_mission", "pi_mission_node", "use_mission"),
    ])
