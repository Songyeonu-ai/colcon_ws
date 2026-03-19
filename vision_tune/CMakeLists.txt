cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0071 NEW)
project(vision_tune)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_AUTOUIC_SEARCH_PATHS
  ${CMAKE_CURRENT_SOURCE_DIR}/ui
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED) #yaml 파일로 save하기 위함

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_library(vision_tune_vision_utils
  src/utils/vision_utils.cpp
)

# =========================
# Messages
# =========================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TuningValue.msg"
  "msg/ProcessResult.msg"
  "msg/PanTilt.msg"
  "msg/PanTiltMsgs.msg"
  "msg/PanTiltStatusMsgs.msg"
  "msg/BoundingBox.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

# =========================
# camera_node
# =========================
add_executable(pan_tilt_camera_node
  src/usb_cam/insta/pan_tilt_camera_node.cpp
  src/usb_cam/insta/usb_camera.cpp
)

ament_target_dependencies(pan_tilt_camera_node
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  cv_bridge
)

target_include_directories(pan_tilt_camera_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

target_link_libraries(pan_tilt_camera_node
  ${cpp_typesupport_target}
)

# =========================
# process_node
# =========================
add_executable(process_node
  src/process/process_node.cpp
)

ament_target_dependencies(process_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

target_include_directories(process_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

target_link_libraries(process_node
  vision_tune_vision_utils
  ${cpp_typesupport_target}
  ${OpenCV_LIBS}
)

# =========================
# ui_node
# =========================
add_executable(ui_node
  src/ui/ui_node.cpp
  src/ui/main_window.cpp
  include/vision_tune/ui/main_window.hpp
  include/vision_tune/ui/ui_node.hpp
  ui/main_window.ui
)

ament_target_dependencies(ui_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

target_include_directories(ui_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

target_link_libraries(ui_node
  vision_tune_vision_utils
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
  yaml-cpp
  ${cpp_typesupport_target}
  ${OpenCV_LIBS}
)

install(TARGETS
  pan_tilt_camera_node
  process_node
  ui_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY ui
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()