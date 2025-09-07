cmake_minimum_required(VERSION 3.8)
project(service_tutorial_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_srv REQUIRED)

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

add_executable(spawn_turtle_client src/turtle_spawn.cpp)
add_executable(pubvel_toggle src/velocity_toggle.cpp)
add_executable(pubvel_toggle_rate src/velocity_toggle_plus_service.cpp)

ament_target_dependencies(spawn_turtle_client
  rclcpp
  turtlesim
)

ament_target_dependencies(pubvel_toggle
  rclcpp
  std_srvs
  geometry_msgs
)
ament_target_dependencies(pubvel_toggle_rate
  rclcpp
  std_srvs
  geometry_msgs
  my_srv
)

install(TARGETS
  spawn_turtle_client
  pubvel_toggle
  pubvel_toggle_rate
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
