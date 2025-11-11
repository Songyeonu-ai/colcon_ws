cmake_minimum_required(VERSION 3.8)
project(humanoid_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}

  "msg/IkAngleSimMsg.msg"
  "msg/IkComMsg.msg"
  "msg/IkCoordMsg.msg"
  "msg/IkEndMsg.msg"
  "msg/IkLTCMsg.msg"
  "msg/IkPatternMsg.msg"

  "msg/CPWalk.msg"
  "msg/LCMsgs.msg"

  "msg/ImuMsg.msg"
  "msg/Master2IkMsg.msg"
  "msg/MotorMsg.msg"
  "msg/Tune2IkMsg.msg"
  "msg/ZmpMsg.msg"

  "msg/Robocupvision25.msg"
  "msg/Robocupvision25feature.msg"
  "msg/Master2vision25.msg"

  "msg/Robocuplocalization25.msg"
  "msg/Master2localization25.msg"

  "msg/Motionnum.msg"
  "msg/Motionend.msg"

  "msg/Master2udp.msg"
  "msg/Udp2master.msg"

  "msg/Gamecontroldata.msg"
  "msg/Gamecontrolreturndata.msg"
  "msg/Robocupcontroller.msg"

  "msg/MotionOperator.msg"
  
  "msg/Pidtuning.msg"

  DEPENDENCIES std_msgs
  DEPENDENCIES geometry_msgs
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
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
