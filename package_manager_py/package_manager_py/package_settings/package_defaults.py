from dataclasses import dataclass
from typing import List, Dict

@dataclass
class PackageConfig:
    id: int
    name: str
    executable: str
    pkg_type: str  # 'launch' or 'run'
    description: str = ""
    auto_start: bool = False

DEFAULT_PACKAGES: Dict[int, PackageConfig] = {
    1: PackageConfig(1, 'dynamixel_rdk_ros2', 'rdk.launch.py', 'launch'),
    2: PackageConfig(2, 'ik_walk', 'ik_walk', 'run'),
    3: PackageConfig(3, 'ebimu_v5', 'e2box_imu_9dofv4.launch.py', 'launch'),
    4: PackageConfig(4, 'robocup_vision', 'robocup_vision.launch.py', 'launch'),
    5: PackageConfig(5, 'udpcom', 'udp_com.launch.py', 'launch'),
    6: PackageConfig(6, 'web_control_bridge', 'web_control_bridge.launch.py', 'launch'),
    7: PackageConfig(7, 'gamecontroller', 'gamecontroller', 'run'),
    8: PackageConfig(8, 'robocup_localization25', 'robocup_localization25', 'run'),
    9: PackageConfig(9, 'robocup_master25', 'robocup_master25.launch.py', 'launch'),
    10: PackageConfig(10, 'motion_operator', 'motion_operator.launch.py', 'launch'),
    11: PackageConfig(11, 'tune_walk', 'tune_walk', 'run'),
}
