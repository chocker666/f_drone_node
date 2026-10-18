cmake_minimum_required(VERSION 3.8)
project(drone_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# Dependencies
# ============================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Threads REQUIRED)


# ============================================================
# ROS2 interfaces
# ============================================================

set(INTER_MSG_FILES
  "msg/VehicleState.msg"
  "msg/TrajectorySetpoint.msg"
)

set(CTRL_MSG_FILES
  "msg/ControlTrajectory.msg"
  "msg/FixedTrajectory.msg"
  "msg/PIDParams.msg"
  "msg/ControlStatus.msg"
  "msg/ControlCommand.msg"
)

set(EXEC_ACTION_FILES
  "action/GoTo.action"
  "action/Takeoff.action"
  "action/Land.action"
  "action/Arm.action"
  "action/Disarm.action"
  "action/Stop.action"
)

set(EXEC_SRV_FILES
  "srv/SetArming.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${INTER_MSG_FILES}
  ${CTRL_MSG_FILES}
  ${EXEC_ACTION_FILES}
  ${EXEC_SRV_FILES}
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

# ============================================================
# Common include path
# ============================================================

set(DRONE_INCLUDE_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ============================================================
# Control algorithms
# ============================================================

add_library(drone_control_algorithms STATIC
  src/ctrl/algorithm/pid_controller.cpp
)

target_include_directories(drone_control_algorithms PUBLIC
  ${DRONE_INCLUDE_DIR}
)

target_link_libraries(drone_control_algorithms PUBLIC
  Eigen3::Eigen
)

# ============================================================
# Source groups
# ============================================================

set(INTER_SOURCES
  src/inter/px4_interface.cpp
  src/inter/frame_transforms.cpp
)

set(CTRL_SOURCES
  src/ctrl/flight_controller.cpp
  src/ctrl/trajectory_manager.cpp
  src/ctrl/trajectory_sampler/fixed_sampler.cpp
)

# ============================================================
# PX4 Interface executable
# ============================================================

add_executable(px4_interface_exec
  src/main/inter_main.cpp
  ${INTER_SOURCES}
)

target_include_directories(px4_interface_exec PUBLIC
  ${DRONE_INCLUDE_DIR}
)

ament_target_dependencies(px4_interface_exec
  rclcpp
  px4_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
)

target_link_libraries(px4_interface_exec
  Eigen3::Eigen
  "${cpp_typesupport_target}"
)

# ============================================================
# Flight Controller executable
# ============================================================

add_executable(flight_controller_exec
  src/main/ctrl_main.cpp
  ${CTRL_SOURCES}
)

target_include_directories(flight_controller_exec PUBLIC
  ${DRONE_INCLUDE_DIR}
)

ament_target_dependencies(flight_controller_exec
  rclcpp
  geometry_msgs
)

target_link_libraries(flight_controller_exec
  drone_control_algorithms
  "${cpp_typesupport_target}"
)

# ============================================================
# Combined Drone Node executable
# ============================================================

add_executable(drone_node_exec
  src/main/drone_node_main.cpp
  ${INTER_SOURCES}
  ${CTRL_SOURCES}
)

target_include_directories(drone_node_exec PUBLIC
  ${DRONE_INCLUDE_DIR}
)

ament_target_dependencies(drone_node_exec
  rclcpp
  px4_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
)

target_link_libraries(drone_node_exec
  drone_control_algorithms
  Eigen3::Eigen
  Threads::Threads
  "${cpp_typesupport_target}"
)

# ============================================================
# Attitude test executable
# ============================================================

add_executable(attitude_test
  test/inter/attitude_test.cpp
)

target_include_directories(attitude_test PUBLIC
  ${DRONE_INCLUDE_DIR}
)

ament_target_dependencies(attitude_test
  rclcpp
  px4_msgs
)

target_link_libraries(attitude_test
  Eigen3::Eigen
  "${cpp_typesupport_target}"
)


# ============================================================
# Motion Executor executable
# ============================================================

add_executable(motion_executor_exec
  src/main/exec_main.cpp
  src/exec/motion_executor.cpp
)

target_include_directories(motion_executor_exec PUBLIC
  ${DRONE_INCLUDE_DIR}
)

ament_target_dependencies(motion_executor_exec
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
)

target_link_libraries(motion_executor_exec
  "${cpp_typesupport_target}"
)


# ============================================================
# Install
# ============================================================

install(TARGETS
  drone_node_exec
  px4_interface_exec
  flight_controller_exec
  motion_executor_exec
  attitude_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

# ============================================================
# Testing and lint
# ============================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
