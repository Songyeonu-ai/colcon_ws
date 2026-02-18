from dataclasses import dataclass
from typing import List

@dataclass
class PackageConfig:
    name: str
    executable: str
    pkg_type: str  # 'launch' or 'run'
    description: str = ""
    auto_start: bool = False

DEFAULT_PACKAGES: List[PackageConfig] = [ # rdk, ik, tune, ebimu, vision, local, motion, udp, web, gamecontroller
    PackageConfig(
        name='dynamixel_rdk_ros2', 
        executable='rdk.launch.py',
        pkg_type='launch',
        description='Dynamixel RDK ROS2 package',
        auto_start=False
    ),
    PackageConfig(
        name='ik_walk',
        executable='ik_walk',
        pkg_type='run',
        description='Inverse Kinematics Walk',
        auto_start=False
    ),
    PackageConfig(
        name='ebimu_v5',
        executable='e2box_imu_9dofv4.launch.py',
        pkg_type='launch',
        description='EBIMU V5 IMU Sensor',
        auto_start=False
    ),
    PackageConfig(
        name='robocup_vision',
        executable='robocup_vision.launch.py',
        pkg_type='launch',
        description='Vision',
        auto_start=False
    ),
    PackageConfig(
        name='udpcom',
        executable='udp_com.launch.py',
        pkg_type='launch',
        description='Udp Communication',
        auto_start=False
    ),
    PackageConfig(
        name='web_control_bridge',
        executable='web_control_bridge.launch.py',
        pkg_type='launch',
        description='Web Control Bridge',
        auto_start=False
    ),
    PackageConfig(
        name='gamecontroller',
        executable='gamecontroller',
        pkg_type='run',
        description='Game Controller',
        auto_start=False
    ),
    PackageConfig(
        name='robocup_localization25',
        executable='robocup_localization25',
        pkg_type='run',
        description='Localization',
        auto_start=False
    ),
    PackageConfig(
        name='robocup_master25',
        executable='robocup_master25.launch.py',
        pkg_type='launch',
        description='Master',
        auto_start=False
    ),
    PackageConfig(
        name='motion_operator',
        executable='motion_operator.launch.py',
        pkg_type='launch',
        description='Motion Operator',
        auto_start=False
    ),
    PackageConfig(
        name='tune_walk',
        executable='tune_walk',
        pkg_type='run',
        description='Walk Tuning',
        auto_start=False
    ),
]
