cmake_minimum_required(VERSION 3.8)
project(observer)

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

add_executable(wind_observer src/wind_observer.cpp)
ament_target_dependencies(wind_observer px4_msgs rclcpp std_msgs tf2 self_msg)
install(TARGETS
  wind_observer
  DESTINATION lib/${PROJECT_NAME})
  
add_executable(wind_observer_g src/wind_observer_g.cpp)
ament_target_dependencies(wind_observer_g px4_msgs rclcpp std_msgs tf2 self_msg)
install(TARGETS
  wind_observer_g
  DESTINATION lib/${PROJECT_NAME})
  
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
