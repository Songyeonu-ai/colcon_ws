import rclpy
from rclpy.node import Node
from web_control_bridge.msg import NodeManagerMsg, Pm2WebBridgeMsg, ListStructure
from PyQt5.QtCore import QObject, pyqtSignal, QThread, QTimer

from ..package_settings import settings, DEFAULT_PACKAGES
from ..package_settings.package_defaults import PackageConfig
from ..core.process_manager import ProcessManager
from ..core.package_manager import PackageConfigManager
from ..core.runtime_state_manager import RuntimeStateManager, PackageState
from ..utils.constants import LAUNCH, RUN
from ..package_settings.settings import RESTART_DELAY

package_count=settings.DEFAULT_PACKAGE_COUNT+1

class RosNode(Node, QObject):
    status_message = pyqtSignal(str)
    package_started = pyqtSignal(int)
    package_stopped = pyqtSignal(int)
    shutdown_signal = pyqtSignal()
    
    def __init__(self, node_name: str = None):
        QObject.__init__(self)

        node_name = node_name or settings.DEFAULT_NODE_NAME
        Node.__init__(self, node_name)

        self._declare_parameters()

        self.config_manager = PackageConfigManager(self)
        self.process_manager = ProcessManager(self)
        self.state_manager = RuntimeStateManager()

        for pkg_id in self.config_manager.get_all_packages():
            self.state_manager.register(pkg_id)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.pm2web_pub = self.create_publisher(Pm2WebBridgeMsg, '/packagestate', 10)
        self.timer = self.create_timer(0.1, self._pm2web_pub)
        self.create_subscription(NodeManagerMsg, '/nodemanager', self._nodemanager_callback, 10)
        self.TUNE_SET = {1,2,3,8,11}
        self.count = 0
        
        self._connect_process_signals()

    def timer_callback(self):
        self.count += 1

    def _pm2web_pub(self):
        msg = Pm2WebBridgeMsg()
        runtime_states = self.process_manager.get_runtime_states()

        for pkg_id, state_str in runtime_states.items():
            item = ListStructure()

            item.id = int(pkg_id)

            if state_str == "RUNNING":
                item.state = PackageState.RUNNING      # 2
            elif state_str == "STOPPED":
                item.state = PackageState.STOPPED      # 0
            elif state_str == "STARTING":
                item.state = PackageState.STARTING     # 1
            elif state_str == "STOPPING":
                item.state = PackageState.STOPPING     # 3
            else:
                item.state = PackageState.ERROR        # 4

            msg.packages.append(item)

        self.pm2web_pub.publish(msg)


    def _nodemanager_callback(self, msg: NodeManagerMsg):

        if msg.id != 3:
            return

        if msg.action == "start":
            self.start_package(msg.package_id)

        elif msg.action == "stop":
            self.stop_package(msg.package_id) 


    def _declare_parameters(self):
        for i in range(1, package_count):
            default_config = DEFAULT_PACKAGES.get(i)

            if default_config is None:
                default_config = PackageConfig(i, "", "", "launch")

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
    
    def start_package(self, package_id: int):
        if not self.state_manager.can_start(package_id):
            self.status_message.emit(f"Package {package_id} already running")
            return

        self.state_manager.set_starting(package_id)
        
        config = self.config_manager.get_package(package_id)
        if not config:
            self.status_message.emit(f"Package {package_id} not found")
            return

        command = "ros2"

        if config.pkg_type == "launch":
            arguments = ["launch", config.name, config.executable]
        else:
            arguments = ["run", config.name, config.executable]

        success, msg = self.process_manager.start_process(
            package_id, command, arguments
        )

        self.status_message.emit(msg)
    
    def stop_package(self, package_id: int):
        if not self.state_manager.can_stop(package_id):
            return

        self.state_manager.set_stopping(package_id)
        success, msg = self.process_manager.stop_process(package_id)
        self.status_message.emit(msg)

    def start_all(self):
        for pkg_id, config in self.config_manager.get_all_packages().items():
            if pkg_id == 11:  # tune_walk 제외
                continue
            self.start_package(pkg_id)
            QThread.msleep(500) #패키지 켜질 대기시간


    def tune_start(self):
        for pkg_id in self.TUNE_SET:
            self.start_package(pkg_id)
            QThread.msleep(500)

    def without_UDP(self):
        for pkg_id in self.config_manager.get_all_packages():
            if pkg_id in (5,11):  # udpcom, tune_walk 제외
                continue
            self.start_package(pkg_id)
            QThread.msleep(500)
    
    def restart_package(self, package_id: int):
        self.status_message.emit(f"Restarting package: {package_id}")

        self.stop_package(package_id)
        QThread.msleep(RESTART_DELAY)
        self.start_package(package_id)
        
    
    def start_package_by_index(self, index: int):
        self.start_package(index)
    
    def is_package_running(self, package_id: int) -> bool:
        return self.process_manager.is_running(package_id)
    
    # ========== Convenience Methods ==========
    
    def get_package_info(self, index: int) -> PackageConfig:
        return self.config_manager.get_package(index)
    
    def get_all_packages(self) -> dict:
        return self.config_manager.get_all_packages()
    
    # ========== Signal Handlers ==========
    
    def _on_package_started(self, package_id: int):
        self.state_manager.set_running(package_id)
        self.status_message.emit(f"✓ Package {package_id} started")
        self.package_started.emit(package_id)
    
    def _on_package_stopped(self, package_id: int):
        self.state_manager.set_stopped(package_id)
        self.status_message.emit(f"● Package {package_id} stopping...")
        self.package_stopped.emit(package_id)
    
    def _on_package_error(self, package_id: int, error_msg: str):
        self.state_manager.set_error(package_id, error_msg)
        self.status_message.emit(f"✗ {package_id}: {error_msg}")
    
    # ========== Cleanup ==========
    
    def shutdown_node(self):
        self.process_manager.stop_all()
        
        self.shutdown_signal.emit()
        
        self.destroy_node()
        rclpy.shutdown()