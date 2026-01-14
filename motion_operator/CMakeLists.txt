cmake_minimum_required(VERSION 3.8)
project(motion_operator)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(humanoid_interfaces REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(dynamixel_rdk_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

set(sources src/motion_operator.cpp src/motion_operator_node.cpp)

add_executable(motion_operator_node ${sources})
target_link_libraries(motion_operator_node ${rclcpp_LIBRARIES} yaml-cpp)
ament_target_dependencies(motion_operator_node rclcpp rclcpp_lifecycle humanoid_interfaces std_msgs dynamixel_rdk_msgs)

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

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  motion_operator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
