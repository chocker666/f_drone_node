cmake_minimum_required(VERSION 3.8)
project(drone_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Threads REQUIRED)




rosidl_generate_interfaces(${PROJECT_NAME}
  # inter part msg
  "msg/VehicleState.msg"
  "msg/TrajectorySetpoint.msg"
  # ctrl part msg
  "msg/ControlTrajectory.msg"
  "msg/FixedTrajectory.msg"
  "msg/PIDParams.msg"
  "msg/ControlStatus.msg"
  # others
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
)

# Control algorithms
add_library(drone_control_algorithms STATIC
  src/algorithm/pid_controller.cpp
)

target_include_directories(drone_control_algorithms PUBLIC
  include
)

target_link_libraries(drone_control_algorithms PUBLIC
  Eigen3::Eigen
)



# px4_interface exec
add_executable(px4_interface_exec
  src/px4_interface_main.cpp
  src/px4_interface.cpp
  src/frame_transforms.cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(px4_interface_exec
  Eigen3::Eigen
  "${cpp_typesupport_target}"
)

target_include_directories(px4_interface_exec PUBLIC
  include
)

ament_target_dependencies(px4_interface_exec
  rclcpp
  px4_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
)


# flight_controller exec
# add_executable(flight_controller_exec
#   src/flight_controller_main.cpp
#   src/flight_controller.cpp
#   src/algorithm/pid_controller.cpp
#   src/trajectory_sampler/fixed_sampler.cpp
#   src/trajectory_manager.cpp
# )
add_executable(flight_controller_exec
  src/flight_controller_main.cpp
  src/flight_controller.cpp
  src/trajectory_manager.cpp
  src/trajectory_sampler/fixed_sampler.cpp
)

target_link_libraries(flight_controller_exec
  drone_control_algorithms
  "${cpp_typesupport_target}"
)

target_include_directories(flight_controller_exec PUBLIC
  include
)

ament_target_dependencies(flight_controller_exec
  rclcpp
  geometry_msgs
)



# drone_node exec
# add_executable(drone_node_exec
#   src/main.cpp
#   src/px4_interface.cpp
#   src/flight_controller.cpp
#   src/frame_transforms.cpp
#   src/algorithm/pid_controller.cpp
#   src/trajectory_sampler/fixed_sampler.cpp
#   src/trajectory_manager.cpp
# )
add_executable(drone_node_exec
  src/main.cpp
  src/px4_interface.cpp
  src/flight_controller.cpp
  src/trajectory_manager.cpp
  src/trajectory_sampler/fixed_sampler.cpp
  src/frame_transforms.cpp
)

target_include_directories(drone_node_exec PUBLIC
  include
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


# attitude tf test exec
add_executable(attitude_test
  src/attitude_test.cpp
)

target_include_directories(attitude_test PUBLIC
  include
)

ament_target_dependencies(attitude_test
  rclcpp
  px4_msgs
)

target_link_libraries(attitude_test
  Eigen3::Eigen
  "${cpp_typesupport_target}"
)









install(TARGETS
  drone_node_exec
  px4_interface_exec
  flight_controller_exec
  attitude_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
