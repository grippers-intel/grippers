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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    use_fake_host = LaunchConfiguration("use_fake_host")
    host_ip = LaunchConfiguration("host_ip")
    scan_floor_enabled = LaunchConfiguration("scan_floor_enabled")
    record_bag = LaunchConfiguration("record_bag")
    bag_output = LaunchConfiguration("bag_output")
    arm_port = LaunchConfiguration("arm_port")
    use_vla = LaunchConfiguration("use_vla")
    policy_source = LaunchConfiguration("policy_source")
    policy_url = LaunchConfiguration("policy_url")
    policy_calibration_file = LaunchConfiguration("policy_calibration_file")
    gripper_cam_publish_hz = LaunchConfiguration("gripper_cam_publish_hz")

    # ⚠️ use_vla 를 끄면 그리퍼캠 발행도 **함께** 꺼져야 한다. perception_node 의
    # 기본값이 0.0(끔)이고 "켜기 전에는 이 노드의 동작이 전과 완전히 같다"가
    # 그쪽 주석의 약속이라, 여기서 무조건 10Hz 를 박으면 VLA 를 안 쓰는 실기에서도
    # 1280x720 캡처가 매초 돌아 4코어를 갉아먹는다.
    #
    # use_fake_* 주석이 말하는 함정과 같은 종류다 — 스위치 하나가 여러 자리에
    # 걸쳐 있으면 하나만 빠져도 "껐다고 믿었는데 도는" 상태가 된다. 그래서
    # 인자를 둘로 나누되 **묶는 계산을 여기 한 곳에 둔다.**
    gripper_cam_hz = ParameterValue(
        PythonExpression([
            gripper_cam_publish_hz, " if '", use_vla,
            "'.lower() in ('true', '1') else 0.0",
        ]),
        value_type=float,
    )

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
        parameters=[
            {
                "scan_floor_enabled": scan_floor_enabled,
                # 그리퍼캠을 토픽으로 내보낸다 — vla_inference 가 이걸 구독한다.
                # 카메라를 소유한 노드가 여기라서 여기서 발행해야 한다
                # (gripper_cam_publisher_node 를 따로 띄우면 같은 장치를 두 번 열어
                # Device or resource busy 다 — 그 노드 docstring 의 경고).
                "gripper_cam_publish_hz": gripper_cam_hz,
            }
        ],
    )
    # perception_node는 회전 보정된 스트림만 구독한다 — 이 노드가 없으면
    # 카메라가 뒤집힌 프레임에서 YOLO가 매 프레임 오검출을 낸다(2026-08-26
    # 인수인계서 §작업 규칙). 이전까지는 이 launch에서 빠져 있어 매번 손으로
    # 따로 띄워야 했다.
    depth_cam_rotate_node = Node(
        package="grippers_perception",
        executable="depth_cam_rotate_node",
        output="screen",
        condition=UnlessCondition(use_fake_perception),
    )
    arm_driver_node = Node(
        package="grippers_arm",
        executable="arm_driver",
        output="screen",
        condition=UnlessCondition(use_fake_arm),
        parameters=[
            {
                "arm_port": arm_port,
                # 빈 문자열이면 arm_driver 가 "미설정" 으로 로그만 남기고 넘어가고
                # ExecuteJointChunk(VLA 재생)만 거부된다 — classic 경로는 그대로다.
                # 그래서 use_vla 와 무관하게 항상 넘겨도 안전하다.
                "policy_calibration_file": policy_calibration_file,
            }
        ],
    )
    # VLA 파지 백엔드. 기본은 꺼져 있다 — 미션 FSM 의 classic GRASP 가 여전히
    # 기본 경로이고, remote 추론은 노트북과 네트워크에 의존하기 때문이다.
    #
    # ⚠️ 이 노드는 카메라를 직접 열지 않는다. perception_node 의
    # gripper_cam_publish_hz 로 나오는 토픽을 구독한다(위 gripper_cam_hz 참고).
    vla_inference_node = Node(
        package="grippers_vla",
        executable="vla_inference_node",
        output="screen",
        condition=IfCondition(use_vla),
        parameters=[
            {
                "policy_source": policy_source,
                "policy_url": policy_url,
            }
        ],
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
                    "use_fake_host": use_fake_host,
                    "host_ip": host_ip,
                }
            ],
        ),
    ]

    return [
        controller_launch,
        depth_camera_launch,
        lidar_launch,
        perception_node,
        depth_cam_rotate_node,
        arm_driver_node,
        vla_inference_node,
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
            # ⚠️ 이 둘이 없어서 실기 통합이 막혀 있었다(2026-08-28 확인).
            # host_ip 기본값이 작성자 개발 PC 주소라, 다른 사람이 Host를 띄우면
            # **명령은 가는데 보고는 남의 PC로 갔다.** 명령이 단방향이라 차는
            # 정상적으로 움직이고 Host만 아무것도 못 받는다 — 링크가 끊긴 것처럼
            # 보이지만 절반만 연결된 상태다.
            #
            # 근본 해법은 UdpHostLink가 **명령을 보낸 쪽으로** 보고하게 한 것이고
            # (같은 날 수정), 이 인자는 그것을 끄고 고정하고 싶을 때 쓴다.
            DeclareLaunchArgument(
                "use_fake_host",
                default_value="false",
                description="true면 UDP 없이 FakeHostLink 사용 (Host 없이 시험)",
            ),
            DeclareLaunchArgument(
                "host_ip",
                default_value="192.168.0.10",
                description="보고를 보낼 Host 주소의 **초기값**. 첫 명령이 오면 "
                            "그 명령을 보낸 주소로 자동으로 바뀐다",
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
            # ── VLA 파지 백엔드 ────────────────────────────────────────────
            #
            # 기본이 false 인 이유는 두 가지다. (1) 미션 FSM 의 classic GRASP 가
            # 여전히 기본 경로다. (2) policy_source 기본값이 remote 라, 켜면
            # **노트북의 policy_server 가 떠 있어야** 노드가 기동한다 — 없으면
            # health 에서 일찍 실패한다. 그 편이 파지 도중에 알게 되는 것보다 낫지만,
            # 시연 기본값이 네트워크에 의존해서는 안 된다.
            DeclareLaunchArgument(
                "use_vla",
                default_value="false",
                description="true면 vla_inference_node를 띄우고 그리퍼캠 발행을 켠다 "
                "(policy_source 기본 remote — 노트북 policy_server가 필요하다)",
            ),
            DeclareLaunchArgument(
                "policy_source",
                default_value="remote",
                description="local이면 Pi가 체크포인트를 들고 추론하고, remote면 "
                "policy_url의 policy_server에 맡긴다",
            ),
            DeclareLaunchArgument(
                "policy_url",
                default_value="http://192.168.0.2:8770",
                description="policy_source=remote일 때 추론 서버 주소",
            ),
            # ⚠️ 저장소 안의 이 파일이 기준이다 — 노트북의 lerobot 캘리브레이션
            # 캐시(~/.cache/huggingface/lerobot/.../grippers_arm.json)와 같은
            # 내용이고, 녹화 때 lerobot 이 실제로 읽은 것이 그쪽이다.
            # Pi 의 /shared/hf_cache 사본은 gripper range_max 가 2378 로 멈춘
            # 옛 것이니 쓰지 말 것(2026-09-05 확인).
            DeclareLaunchArgument(
                "policy_calibration_file",
                default_value="/grippers/host/vla/calibration/grippers_arm.json",
                description="정책 좌표계 캘리브레이션. 빈 문자열이면 "
                "ExecuteJointChunk(VLA 재생)만 거부되고 classic 경로는 그대로다",
            ),
            DeclareLaunchArgument(
                "gripper_cam_publish_hz",
                default_value="10.0",
                description="그리퍼캠 발행 주기. use_vla=false면 이 값과 무관하게 "
                "0(끔)이 된다 — launch_setup의 gripper_cam_hz 주석 참고",
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
