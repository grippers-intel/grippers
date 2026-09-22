cmake_minimum_required(VERSION 3.8)
project(vla_robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "srv/GetArmState.srv"
  "srv/SetGripper.srv"
  "action/ExecuteJointChunk.action"
  "action/MoveToPose.action"
  "action/RunVlaGrasp.action"
  DEPENDENCIES action_msgs builtin_interfaces
)

ament_package()
