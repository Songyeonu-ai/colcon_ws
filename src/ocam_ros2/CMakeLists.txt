cmake_minimum_required(VERSION 3.8)
project(ocam_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)


# Declare the executable
add_executable(ocam_ros2 src/oCam.cpp src/withrobot_camera.cpp src/withrobot_utility.cpp)

# Link libraries
target_link_libraries(ocam_ros2
  ${rclcpp_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Boost::system 
  Boost::thread
  -lv4l2 
  -ludev
)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


# Add dependencies
ament_target_dependencies(ocam_ros2
  rclcpp
  std_msgs
  cv_bridge
  sensor_msgs
  image_transport
  camera_info_manager
  OpenCV
  Boost
)

# Install targets
install(TARGETS
  ocam_ros2
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES config/camera.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
