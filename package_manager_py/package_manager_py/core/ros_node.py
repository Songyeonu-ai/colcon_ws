"""
ROS2 Node for Package Manager
"""

import rclpy
from rclpy.node import Node
from PyQt5.QtCore import QObject, pyqtSignal, QThread

from ..package_settings import settings, DEFAULT_PACKAGES
from ..package_settings.package_defaults import PackageConfig
from ..core.process_manager import ProcessManager
from ..core.package_manager import PackageConfigManager
from ..utils.constants import LAUNCH, RUN
from ..package_settings.settings import RESTART_DELAY

package_count=settings.DEFAULT_PACKAGE_COUNT+1
count=0

class RosNode(Node, QObject):
    """
    ROS2 Node for managing packages
    
    Signals:
        status_message(str): Status update messages
        package_started(str): Package started (package_name)
        package_stopped(str): Package stopped (package_name)
        shutdown_signal(): Node shutdown signal
    """
    
    status_message = pyqtSignal(str)
    package_started = pyqtSignal(str)
    package_stopped = pyqtSignal(str)
    shutdown_signal = pyqtSignal()
    
    def __init__(self, node_name: str = None):
        """
        Initialize ROS2 node
        
        Args:
            node_name: Custom node name (optional)
        """
        # Initialize QObject first
        QObject.__init__(self)
        
        # Initialize ROS2 Node
        node_name = node_name or settings.DEFAULT_NODE_NAME
        Node.__init__(self, node_name)
        
        # Declare ROS2 parameters
        self._declare_parameters()
        
        # Initialize managers
        self.config_manager = PackageConfigManager(self)
        self.process_manager = ProcessManager(self)

        #Initialize Timer
        self.timer = self.create_timer(0.1, lambda: self.timer_callback)
        
        # Connect process manager signals
        self._connect_process_signals()

    def timer_callback(self):
        count += 1

    def _declare_parameters(self):
        """Declare ROS2 parameters for package configurations"""
        for i in range(1, package_count):  # Support up to 11 packages
            # Get default values
            default_config = DEFAULT_PACKAGES[i - 1] if i - 1 < len(DEFAULT_PACKAGES) else PackageConfig("", "", "launch")
            
            self.declare_parameters(
                namespace='',
                parameters=[
                    (f'packages.package{i}.name', default_config.name),
                    (f'packages.package{i}.executable', default_config.executable),
                    (f'packages.package{i}.type', default_config.pkg_type),
                    (f'packages.package{i}.description', default_config.description),
                    (f'packages.package{i}.auto_start', default_config.auto_start),
                ]
            )
    
    def _connect_process_signals(self):
        """Connect process manager signals to node signals"""
        self.process_manager.process_started.connect(
            lambda pkg: self._on_package_started(pkg)
        )
        self.process_manager.process_stopped.connect(
            lambda pkg: self._on_package_stopped(pkg)
        )
        self.process_manager.process_error.connect(
            lambda pkg, msg: self._on_package_error(pkg, msg)
        )
    
    # ========== Package Control Methods ==========
    
    def start_package(self, package_name: str, executable: str, pkg_type: int):
        """
        Start a package
        
        Args:
            package_name: Package name
            executable: Executable/launch file name
            pkg_type: LAUNCH or RUN
        """
        command = "ros2"
        arguments = []
        
        if pkg_type == LAUNCH:
            arguments = ["launch", package_name, executable]
        else:  # RUN
            arguments = ["run", package_name, executable]
        
        success, msg = self.process_manager.start_process(
            package_name, command, arguments
        )
        
        self.status_message.emit(msg)
    
    def stop_package(self, package_name: str):
        """
        Stop a package
        
        Args:
            package_name: Package name
        """
        success, msg = self.process_manager.stop_process(package_name)
        self.status_message.emit(msg)

    def start_all(self):
        all_pkgs = self.config_manager.get_all_packages()
        for i, config in all_pkgs.items():
            if config and config.name and config.name != "tune_walk":
                pkg_type = LAUNCH if config.pkg_type == "launch" else RUN
                self.start_package(config.name, config.executable, pkg_type)
                QThread.msleep(500) #패키지 켜질 시간

    def tune_start(self):
        all_pkgs = self.config_manager.get_all_packages()
        for i, config in all_pkgs.items():
            if config and config.name and (config.name == "dynamixel_rdk_ros2" or config.name == "ik_walk" or config.name == "tune_walk" or config.name == "ebimu_v5" or config.name == "robocup_localization25)"):
                pkg_type = LAUNCH if config.pkg_type == "launch" else RUN
                self.start_package(config.name, config.executable, pkg_type)
                QThread.msleep(500)

    def without_UDP(self):
        all_pkgs = self.config_manager.get_all_packages()
        for i, config in all_pkgs.items():
            if config and config.name and config.name != "udpcom" and config.name != "tune_walk":
                pkg_type = LAUNCH if config.pkg_type == "launch" else RUN
                self.start_package(config.name, config.executable, pkg_type)
                QThread.msleep(500)
    
    def restart_package(self, package_name: str, executable: str, pkg_type: int):
        """
        Restart a package
        
        Args:
            package_name: Package name
            executable: Executable/launch file name
            pkg_type: LAUNCH or RUN
        """
        self.status_message.emit(f"Restarting package: {package_name}")
        self.stop_package(package_name) #여기까지는 잘 작동..
        
        QThread.msleep(RESTART_DELAY)
        self.start_package(package_name, executable, pkg_type)  
        
    
    def start_package_by_index(self, index: int):
        """
        Start package by configuration index
        
        Args:
            index: Package index (1-8)
        """
        config = self.config_manager.get_package(index)
        
        if config is None or not config.name:
            self.status_message.emit(f"Package {index} not configured")
            return
        
        pkg_type = LAUNCH if config.pkg_type == "launch" else RUN
        self.start_package(config.name, config.executable, pkg_type)
    
    def is_package_running(self, package_name: str) -> bool:
        """Check if a package is running"""
        return self.process_manager.is_running(package_name)
    
    # ========== Convenience Methods ==========
    
    def get_package_info(self, index: int) -> PackageConfig:
        """Get package configuration by index"""
        return self.config_manager.get_package(index)
    
    def get_all_packages(self) -> dict:
        """Get all package configurations"""
        return self.config_manager.get_all_packages()
    
    # ========== Signal Handlers ==========
    
    def _on_package_started(self, package_name: str):
        """Handle package started event"""
        self.status_message.emit(f"✓ Package '{package_name}' started successfully")
        self.package_started.emit(package_name)
    
    def _on_package_stopped(self, package_name: str):
        """Handle package stopped event"""
        self.status_message.emit(f"● Package '{package_name}' stopped")
        self.package_stopped.emit(package_name)
    
    def _on_package_error(self, package_name: str, error_msg: str):
        """Handle package error event"""
        self.status_message.emit(f"✗ {package_name}: {error_msg}")
    
    # ========== Cleanup ==========
    
    def shutdown_node(self):
        """Shutdown node and cleanup resources"""
        # Stop all running processes
        self.process_manager.stop_all()
        
        # Emit shutdown signal
        self.shutdown_signal.emit()
        
        # Cleanup ROS2
        self.destroy_node()
        rclpy.shutdown()
