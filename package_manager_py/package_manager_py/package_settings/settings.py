"""
Global settings for Package Manager
"""

# ========== Application Settings ==========
APP_NAME = "ROS2 Package Manager"
APP_VERSION = "0.1.0"

# ========== Package Configuration ==========
MAX_PACKAGES = 11  # Maximum number of packages to manage
DEFAULT_PACKAGE_COUNT = 11

# ========== Process Settings ==========
PROCESS_STOP_TIMEOUT = 5000  # milliseconds
PROCESS_KILL_TIMEOUT = 3000  # milliseconds
RESTART_DELAY = 5000  # milliseconds

# ========== UI Settings ==========
WINDOW_TITLE = "ROS2 Package Manager"
STATUS_UPDATE_ENABLED = True
AUTO_SCROLL_LOG = True

# ========== ROS2 Settings ==========
DEFAULT_NODE_NAME = 'package_manager_py_node'

# ========== Logging Settings ==========
LOG_TIMESTAMP_FORMAT = "HH:mm:ss"
LOG_MAX_LINES = 1000

# ========== Package GUI Settings ==========
# 패키지별 고정 사이즈
PACKAGE_GUI_SETTINGS = {
    "ebimu_v5": {
        "width": 409,
        "height": 588,
    },
    "gamecontroller": {
        "width": 465,
        "height": 337,
    },
    "robocup_localization25": {
        "width": 871,
        "height": 822,
    },
    "tune_walk": {
        "width": 1140,
        "height": 672,
    },
    # 기본값
    "default": {
        "width": 600,
        "height": 400,
    }
}
