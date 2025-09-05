cmake_minimum_required(VERSION 3.8)
project(topic_tutorial_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(turtlesim REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

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

add_executable(pub_vel_node src/pubvel.cpp)
add_executable(sub_pose_node src/subpose.cpp)
add_executable(check_direction_node src/direction.cpp)

ament_target_dependencies(pub_vel_node
  rclcpp
  geometry_msgs
)
ament_target_dependencies(sub_pose_node
  rclcpp
  turtlesim
)
ament_target_dependencies(check_direction_node
  rclcpp
  turtlesim
  geometry_msgs
  std_msgs
)

install(TARGETS
  pub_vel_node
  sub_pose_node
  check_direction_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
