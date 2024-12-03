cmake_minimum_required(VERSION 3.8)
project(commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(self_msg REQUIRED)

add_executable(takeoff_mode_srv src/takeoff_srv.cpp)
ament_target_dependencies(takeoff_mode_srv px4_msgs rclcpp std_msgs self_msg)
install(TARGETS
  takeoff_mode_srv
  DESTINATION lib/${PROJECT_NAME})

add_executable(vehicle_command src/vehicle_command.cpp)
ament_target_dependencies(vehicle_command px4_msgs rclcpp std_msgs self_msg)
install(TARGETS
  vehicle_command
  DESTINATION lib/${PROJECT_NAME})

add_executable(service_test src/service_test.cpp)
ament_target_dependencies(service_test px4_msgs rclcpp std_msgs self_msg)
install(TARGETS
  service_test
  DESTINATION lib/${PROJECT_NAME})

add_executable(virtual_leader src/virtual_leader.cpp)
ament_target_dependencies(virtual_leader px4_msgs rclcpp std_msgs self_msg)
install(TARGETS
  virtual_leader 
  DESTINATION lib/${PROJECT_NAME})

add_executable(leader src/leader.cpp)
ament_target_dependencies(leader px4_msgs rclcpp std_msgs tf2 self_msg)
install(TARGETS
  leader 
  DESTINATION lib/${PROJECT_NAME})

add_executable(formation_control src/formation_control.cpp)
ament_target_dependencies(formation_control px4_msgs rclcpp std_msgs tf2 self_msg)
install(TARGETS
  formation_control
  DESTINATION lib/${PROJECT_NAME})

add_executable(offboard_control src/offboard_control.cpp)
ament_target_dependencies(offboard_control px4_msgs rclcpp std_msgs tf2 self_msg)
install(TARGETS
  offboard_control
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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
