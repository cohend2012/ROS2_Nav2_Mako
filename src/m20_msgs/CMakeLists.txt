cmake_minimum_required(VERSION 3.8)
project(m20_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotMode.msg"
  "msg/GaitRequest.msg"
  "msg/FailsafeStatus.msg"
  "msg/HealthReport.msg"
  "msg/MissionItem.msg"
  "msg/BehaviorRequest.msg"
  "msg/BehaviorStatus.msg"
  "msg/ControlAuthority.msg"
  "srv/SetMode.srv"
  "srv/SetControlAuthority.srv"
  "action/RunMission.action"
  DEPENDENCIES std_msgs geometry_msgs action_msgs
)
ament_package()
