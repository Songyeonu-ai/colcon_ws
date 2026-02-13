"""
Main Window for Package Manager
"""

from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QTextEdit, QLabel, QPushButton, QSplitter
)
from PyQt5.QtGui import QIcon, QTextCursor, QGuiApplication
from PyQt5.QtCore import QTime, Qt

from ..package_settings import settings
from ..utils.constants import LAUNCH, RUN
from .widgets.package_control import PackageControlPanel


class MainWindow(QMainWindow):
    """
    Main window for ROS2 Package Manager
    
    Features:
    - Package control (start/stop/restart)
    - Status log display
    - Real-time package status updates
    """
    
    def __init__(self, ros_node, parent=None):
        """
        Initialize main window
        
        Args:
            ros_node: ROS2 node instance
            parent: Parent widget
        """
        super().__init__(parent)
        
        screen_geo = QGuiApplication.primaryScreen().availableGeometry()
        x = screen_geo.width() - self.width() - 50 # 우측 여백 50
        y = screen_geo.height() - self.height() - 50 # 하단 여백 50
        self.move(x, y)

        self.ros_node = ros_node
        
        # Setup UI
        self._setup_window()
        self._setup_ui()
        self._connect_signals()
        
        # Initial status message
        self._log_message("Package Manager initialized. Ready to control packages.")
    
    def _setup_window(self):
        """Setup main window properties"""
        self.setWindowTitle(settings.WINDOW_TITLE)
        self.setMinimumSize(1000, 800)
        
        # Set window icon if available
        try:
            icon = QIcon("://ros-icon.png")
            self.setWindowIcon(icon)
        except:
            pass
    
    def _setup_ui(self):
        """Setup user interface"""
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        main_layout = QVBoxLayout(central_widget)
        
        # Title label
        title_label = QLabel(f"{settings.WINDOW_TITLE} v{settings.APP_VERSION}")
        title_label.setStyleSheet("font-size: 18px; font-weight: bold; padding: 10px;")
        title_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title_label)
        
        # Create splitter for resizable sections
        splitter = QSplitter(Qt.Vertical)
        
        # Package control panel
        packages = self.ros_node.get_all_packages()
        self.control_panel = PackageControlPanel(packages)
        splitter.addWidget(self.control_panel)
        
        # Status log section
        log_widget = self._create_log_widget()
        splitter.addWidget(log_widget)
        
        # Set initial splitter sizes (60% controls, 40% log)
        splitter.setSizes([600, 400])
        
        main_layout.addWidget(splitter)
        
        # Control buttons at bottom
        button_layout = self._create_button_layout()
        main_layout.addLayout(button_layout)
    
    def _create_log_widget(self) -> QWidget:
        """Create status log widget"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Log label
        log_label = QLabel("Status Log:")
        log_label.setStyleSheet("font-weight: bold; font-size: 12px;")
        layout.addWidget(log_label)
        
        # Log text edit
        self.status_log = QTextEdit()
        self.status_log.setReadOnly(True)
        self.status_log.setMinimumHeight(150)
        self.status_log.setStyleSheet("""
            QTextEdit {
                background-color: #f5f5f5;
                font-family: 'Courier New', monospace;
                font-size: 11px;
            }
        """)
        layout.addWidget(self.status_log)
        
        return widget
    
    def _create_button_layout(self) -> QHBoxLayout:
        """Create bottom button layout"""
        layout = QHBoxLayout()
        
        # Clear log button
        clear_btn = QPushButton("Clear Log")
        clear_btn.clicked.connect(self.status_log.clear)
        clear_btn.setStyleSheet("padding: 8px; font-size: 12px;")
        
        # Stop all button
        stop_all_btn = QPushButton("Stop All Packages")
        stop_all_btn.clicked.connect(self._stop_all_packages)
        stop_all_btn.setStyleSheet("""
            QPushButton {
                background-color: #f44336;
                color: white;
                padding: 8px;
                font-size: 12px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #da190b;
            }
        """)

        start_all_btn = QPushButton("Start All Packages")
        start_all_btn.clicked.connect(self._start_all_packages)
        start_all_btn.setStyleSheet("""
            QPushButton {
                background-color: #0cad2f;
                color: white;                padding: 8px;
                font-size: 12px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #09661d;
            }
        """)

        without_U_btn = QPushButton("without UDP")
        without_U_btn.clicked.connect(self._without_UDP_packages)
        without_U_btn.setStyleSheet("""
            QPushButton {
                background-color: #a236f4;
                color: white;                padding: 8px;
                font-size: 12px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #920bda;
            }
        """)

        tune_btn = QPushButton("Tunning Walk")
        tune_btn.clicked.connect(self._tune_packages)
        tune_btn.setStyleSheet("""
            QPushButton {
                background-color: #f4f436;
                color: black;
                padding: 8px;
                font-size: 12px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #dada0b;
            }
        """)
        
        layout.addWidget(clear_btn)
        layout.addStretch()
        layout.addWidget(stop_all_btn)
        layout.addWidget(start_all_btn)
        layout.addWidget(without_U_btn)
        layout.addWidget(tune_btn)
        
        return layout
    
    def _connect_signals(self):
        """Connect signals and slots"""
        # ROS node signals
        self.ros_node.shutdown_signal.connect(self.close)
        self.ros_node.status_message.connect(self._on_status_message)
        self.ros_node.package_started.connect(self._on_package_started)
        self.ros_node.package_stopped.connect(self._on_package_stopped)
        
        # Control panel signals
        self.control_panel.start_package.connect(self._on_start_package)
        self.control_panel.stop_package.connect(self._on_stop_package)
        self.control_panel.restart_package.connect(self._on_restart_package)
    
    # ========== Slot Methods ==========
    
    def _on_start_package(self, index: int):
        """Handle start package request"""
        self.ros_node.start_package_by_index(index)
    
    def _on_stop_package(self, index: int):
        """Handle stop package request"""
        config = self.ros_node.get_package_info(index)
        if config:
            self.ros_node.stop_package(config.name)
    
    def _on_restart_package(self, index: int):
        """Handle restart package request"""
        config = self.ros_node.get_package_info(index)
        if config:
            pkg_type = LAUNCH if config.pkg_type == "launch" else RUN
            self.ros_node.restart_package(config.name, config.executable, pkg_type)
    
    def _on_status_message(self, message: str):
        """Handle status message from ROS node"""
        self._log_message(message)
    
    def _on_package_started(self, package_name: str):
        """Handle package started event"""
        self.control_panel.update_package_status(package_name, True)
    
    def _on_package_stopped(self, package_name: str):
        """Handle package stopped event"""
        self.control_panel.update_package_status(package_name, False)
    
    def _stop_all_packages(self):
        """Stop all running packages"""
        self._log_message("Stopping all packages...")
        self.ros_node.process_manager.stop_all()

    def _start_all_packages(self):
        """Start all Debugging packages"""
        self._log_message("Starting all Debugging packages...")
        self.ros_node.start_all()

    def _tune_packages(self):
        """Tunning packages"""
        self._log_message("Tunning walk packages...")
        self.ros_node.tune_start()

    def _without_UDP_packages(self):
        """Start all running packages without UDP"""
        self._log_message("Starting all packages without UDP...")
        self.ros_node.without_UDP()
    
    # ========== Helper Methods ==========
    
    def _log_message(self, message: str):
        """
        Add message to status log with timestamp
        
        Args:
            message: Message to log
        """
        timestamp = QTime.currentTime().toString(settings.LOG_TIMESTAMP_FORMAT)
        formatted_message = f"[{timestamp}] {message}"
        
        self.status_log.append(formatted_message)
        
        # Auto-scroll to bottom if enabled
        if settings.AUTO_SCROLL_LOG:
            cursor = self.status_log.textCursor()
            cursor.movePosition(QTextCursor.End)
            self.status_log.setTextCursor(cursor)
    
    # ========== Event Handlers ==========
    
    def closeEvent(self, event):
        """Handle window close event"""
        # Cleanup will be handled by main.py
        super().closeEvent(event)
