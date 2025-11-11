cmake_minimum_required(VERSION 3.8)
project(ik_walk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(humanoid_interfaces REQUIRED)
find_package(dynamixel_rdk_ros REQUIRED)
find_package(dynamixel_rdk_msgs REQUIRED)

# for Lifecycle
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rcl_lifecycle REQUIRED)
find_package(rcutils REQUIRED)

include_directories(
  include/ik_walk
  include/
  ${humanoid_interfaces_INCLUDE_DIRS}
)

set(headers
  include/ik_walk/ik_walk.hpp
  include/ik_walk/imu_position_control.hpp
  include/ik_walk/pid_control_float.hpp
  include/ik_walk/solve_kinematics.hpp
  include/ik_walk/walk_pattern.hpp
  include/ik_walk/zmp_position_control.hpp
)

set(sources 
  src/ik_walk.cpp  
  src/imu_position_control.cpp 
  src/pid_control_float.cpp 
  src/solve_kinematics.cpp 
  src/walk_pattern.cpp 
  src/zmp_position_control.cpp 
)
add_executable(ik_walk ${sources}) #${headers}

ament_target_dependencies(ik_walk
  rclcpp
  rclcpp_lifecycle
  std_msgs
  humanoid_interfaces
  dynamixel_rdk_msgs
  dynamixel_rdk_ros
)

install(TARGETS
  ik_walk
  DESTINATION lib/${PROJECT_NAME}
)
# install(DIRECTORY launch params
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
