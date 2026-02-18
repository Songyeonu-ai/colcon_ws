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
    status_message = pyqtSignal(str)
    package_started = pyqtSignal(str)
    package_stopped = pyqtSignal(str)
    shutdown_signal = pyqtSignal()
    
    def __init__(self, node_name: str = None):
        QObject.__init__(self)

        node_name = node_name or settings.DEFAULT_NODE_NAME
        Node.__init__(self, node_name)

        self._declare_parameters()

        self.config_manager = PackageConfigManager(self)
        self.process_manager = ProcessManager(self)

        self.timer = self.create_timer(0.1, lambda: self.timer_callback)
        
        self._connect_process_signals()

    def timer_callback(self):
        count += 1

    def _declare_parameters(self):
        for i in range(1, package_count):  #11 packages
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
        command = "ros2"
        arguments = []
        
        if pkg_type == LAUNCH:
            arguments = ["launch", package_name, executable]
        else:
            arguments = ["run", package_name, executable]
        
        success, msg = self.process_manager.start_process(
            package_name, command, arguments
        )
        
        self.status_message.emit(msg)
    
    def stop_package(self, package_name: str):
        success, msg = self.process_manager.stop_process(package_name)
        self.status_message.emit(msg)

    def start_all(self):
        all_pkgs = self.config_manager.get_all_packages()
        for i, config in all_pkgs.items():
            if config and config.name and config.name != "tune_walk":
                pkg_type = LAUNCH if config.pkg_type == "launch" else RUN
                self.start_package(config.name, config.executable, pkg_type)
                QThread.msleep(500) #패키지 켜질 대기시간

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
        self.status_message.emit(f"Restarting package: {package_name}")
        self.stop_package(package_name)
        
        QThread.msleep(RESTART_DELAY)
        self.start_package(package_name, executable, pkg_type)
        
    
    def start_package_by_index(self, index: int):
        config = self.config_manager.get_package(index)
        
        if config is None or not config.name:
            self.status_message.emit(f"Package {index} not configured")
            return
        
        pkg_type = LAUNCH if config.pkg_type == "launch" else RUN
        self.start_package(config.name, config.executable, pkg_type)
    
    def is_package_running(self, package_name: str) -> bool:
        return self.process_manager.is_running(package_name)
    
    # ========== Convenience Methods ==========
    
    def get_package_info(self, index: int) -> PackageConfig:
        return self.config_manager.get_package(index)
    
    def get_all_packages(self) -> dict:
        return self.config_manager.get_all_packages()
    
    # ========== Signal Handlers ==========
    
    def _on_package_started(self, package_name: str):
        self.status_message.emit(f"✓ Package '{package_name}' started successfully")
        self.package_started.emit(package_name)
    
    def _on_package_stopped(self, package_name: str):
        self.status_message.emit(f"● Package '{package_name}' stopped")
        self.package_stopped.emit(package_name)
    
    def _on_package_error(self, package_name: str, error_msg: str):
        self.status_message.emit(f"✗ {package_name}: {error_msg}")
    
    # ========== Cleanup ==========
    
    def shutdown_node(self):
        self.process_manager.stop_all()
        
        self.shutdown_signal.emit()
        
        self.destroy_node()
        rclpy.shutdown()
