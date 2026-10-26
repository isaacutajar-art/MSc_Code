cmake_minimum_required(VERSION 3.8)
project(hrc_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Measurement.msg" "msg/SensorStatus.msg" "msg/Detection.msg"
  "msg/Velocity.msg" "msg/SpeedFactor.msg" DEPENDENCIES std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
