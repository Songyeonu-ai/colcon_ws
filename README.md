# colcon_ws 
> for robit 20th intern project
---
## how to build
```bash
git clone https://github.com/ROBIT-KOR-teamHumanoid/colcon_ws.git
cw
colcon build --packages-select humanoid_interfaces dynamixel_rdk_msgs
sb
```
## how to run 
### dynamixel_rdk 
```bash
ros2 launch dynamixel_rdk_ros dynamixel_rdk.launch.py
ros2 lifecycle set dynamixel_rdk_node configure
ros2 lifecycle set dynamixel_rdk_node activate
```
### ik_walk
```bash
ros2 run ik_walk ik_walk
```
### camera
```bash
ros2 launch ocam_ros2 ocam_ros.launch.py
ros2 launch robocup_vision25 robocup_vision25_launch.py
```

### motion
```bash
ros2 launch motion_operator motion_operator.launch.py
```

### imu
```bash
ros2 launch ebimu e2box_imu_9dofv4.launch.py
```

### master 
```bash
ros2 launch robocup_master25 robocup_master25.launch.py
```
### local
```bash
ros2 run robocup_localization25 robocup_localization25
```
