cmake_minimum_required(VERSION 3.8)
project(master_jo)

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
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(humanoid_interfaces REQUIRED)

include_directories(
  include
)

file(GLOB_RECURSE ALL_SRCS

  "src/master_jo_node.cpp"
  "src/forward.cpp"
  "src/player.cpp"
  "src/point.cpp"
  "include/master_controller_PID/pidcontroll.cpp"
)

add_executable(master_jo_node ${ALL_SRCS})
target_link_libraries(master_jo_node ${rclcpp_LIBRARIES})
ament_target_dependencies(master_jo_node rclcpp rclcpp_lifecycle std_msgs humanoid_interfaces)

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

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install the executables
install(TARGETS
  master_jo_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
