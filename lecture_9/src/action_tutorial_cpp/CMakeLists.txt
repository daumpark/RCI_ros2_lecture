cmake_minimum_required(VERSION 3.8)
project(action_tutorial_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_tutorial_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

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

add_executable(action_server src/fibonacci_action_server.cpp)
add_executable(action_client src/fibonacci_action_client.cpp)

ament_target_dependencies(action_server
  action_tutorial_interfaces
  rclcpp
  rclcpp_action
)

ament_target_dependencies(action_client
  action_tutorial_interfaces
  rclcpp
  rclcpp_action
)

install(TARGETS
  action_server
  action_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
