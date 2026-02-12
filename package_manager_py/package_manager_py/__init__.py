"""
ROS2 Package Manager

A modular, reusable package manager for ROS2 applications with PyQt5 GUI.

Features:
- Start, stop, and restart ROS2 packages
- Real-time status monitoring
- Configurable via ROS2 parameters
- Clean separation of concerns
- Reusable components
"""

__version__ = "0.1.0"
__author__ = "YeonU"

from .core import RosNode, ProcessManager, PackageConfigManager
from .ui import MainWindow
from .package_settings.package_defaults import PackageConfig

__all__ = [
    'RosNode',
    'ProcessManager',
    'PackageConfigManager',
    'MainWindow',
    'PackageConfig',
]
