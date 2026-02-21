from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QTextEdit, QLabel, QPushButton, QSplitter, QTabWidget
)
from PyQt5.QtGui import QIcon, QTextCursor, QGuiApplication
from PyQt5.QtCore import QTime, Qt

from ..package_settings import settings
from ..utils.constants import LAUNCH, RUN
from .widgets.package_control import PackageControlPanel


class MainWindow(QMainWindow):
    def __init__(self, ros_node, parent=None):
        super().__init__(parent)
        
        screen_geo = QGuiApplication.primaryScreen().availableGeometry()
        x = screen_geo.width() - self.width() - 50 # 우측 여백 50
        y = screen_geo.height() - self.height() - 50 # 하단 여백 50
        self.move(x, y)

        self.ros_node = ros_node

        self._setup_window()
        self._setup_ui()
        self._connect_signals()

        self._log_message("Package Manager initialized. Ready to control packages.")
    
    def _setup_window(self):
        self.setWindowTitle(settings.WINDOW_TITLE)
        self.setMinimumSize(600, 400)

        try:
            icon = QIcon("://ros-icon.png")
            self.setWindowIcon(icon)
        except:
            pass
    
    def _setup_ui(self):#셋업 UI 구성
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout(central_widget)

        title_label = QLabel(f"{settings.WINDOW_TITLE} v{settings.APP_VERSION}")
        title_label.setStyleSheet("font-size: 18px; font-weight: bold; padding: 10px;")
        title_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title_label)

        self.tabs = QTabWidget()

        self.tab1 = QWidget()
        self._setup_tab1()
        self.tabs.addTab(self.tab1, "Package Control")

        # self.tab2 = QWidget()
        # self._setup_tab2()
        # self.tabs.addTab(self.tab2, "Monitoring")

        main_layout.addWidget(self.tabs)

        button_layout = self._create_button_layout()
        main_layout.addLayout(button_layout)

    def _setup_tab1(self):
        splitter = QSplitter(Qt.Vertical)

        packages = self.ros_node.get_all_packages()
        self.control_panel = PackageControlPanel(packages)
        splitter.addWidget(self.control_panel)

        log_widget = self._create_log_widget()
        splitter.addWidget(log_widget)

        splitter.setSizes([600, 400])
        self.tab1_layout = QVBoxLayout(self.tab1)
        self.tab1_layout.addWidget(splitter)

    def _setup_tab2(self):
        layout = QVBoxLayout(self.tab2)
        label_1 = QLabel("rdk features coming soon...") #rdk --> rclcpp로 모든 로그 출력
        label_1.setAlignment(Qt.AlignCenter)
        label_1.setStyleSheet("font-size: 14px; color: gray;")
        
        label_2 = QLabel("ik features coming soon...") #ik --> cout로 모든 로그 출력
        label_2.setAlignment(Qt.AlignCenter)
        label_2.setStyleSheet("font-size: 14px; color: gray;")
        
        label_3 = QLabel("ebimu features coming soon...") #ebimu --> 섞어씀
        label_3.setAlignment(Qt.AlignCenter)
        label_3.setStyleSheet("font-size: 14px; color: gray;")
        
        label_4 = QLabel("vision features coming soon...") #vision --> 섞어씀
        label_4.setAlignment(Qt.AlignCenter)
        label_4.setStyleSheet("font-size: 14px; color: gray;")
        
        label_5 = QLabel("udp features coming soon...") #udp --> rclcpp로 모든 로그 출력
        label_5.setAlignment(Qt.AlignCenter)
        label_5.setStyleSheet("font-size: 14px; color: gray;")
        
        label_6 = QLabel("web bridge features coming soon...") #web bridge --> rclcpp로 모든 로그 출력
        label_6.setAlignment(Qt.AlignCenter)
        label_6.setStyleSheet("font-size: 14px; color: gray;")
        
        label_7 = QLabel("gamecontroller features coming soon...") #gamecontroller --> cout로 모든 로그 출력
        label_7.setAlignment(Qt.AlignCenter)
        label_7.setStyleSheet("font-size: 14px; color: gray;")
        
        label_8 = QLabel("localization features coming soon...") #localization --> cout로 모든 로그 출력
        label_8.setAlignment(Qt.AlignCenter)
        label_8.setStyleSheet("font-size: 14px; color: gray;")

        label_9 = QLabel("master features coming soon...") #master --> cout, rclcpp 섞어씀 cout진짜많이쓰고 rclcpp는 node.cpp에서만 씀
        label_9.setAlignment(Qt.AlignCenter)
        label_9.setStyleSheet("font-size: 14px; color: gray;")

        label_10 = QLabel("motion features coming soon...") #motion --> 섞어씀
        label_10.setAlignment(Qt.AlignCenter)
        label_10.setStyleSheet("font-size: 14px; color: gray;")

        label_11 = QLabel("tune features coming soon...") #tune --> cout로 모든 로그 출력
        label_11.setAlignment(Qt.AlignCenter)
        label_11.setStyleSheet("font-size: 14px; color: gray;")

        layout.addWidget(label_1)
        layout.addWidget(label_2)
        layout.addWidget(label_3)
        layout.addWidget(label_4)
        layout.addWidget(label_5)
        layout.addWidget(label_6)
        layout.addWidget(label_7)
        layout.addWidget(label_8)
        layout.addWidget(label_9)
        layout.addWidget(label_10)
        layout.addWidget(label_11)

    def _create_log_widget(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        log_label = QLabel("Status Log:")
        log_label.setStyleSheet("font-weight: bold; font-size: 12px;")
        layout.addWidget(log_label)
        
        self.status_log = QTextEdit()
        self.status_log.setReadOnly(True)
        self.status_log.setMinimumHeight(100)
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
        layout = QHBoxLayout()
        
        clear_btn = QPushButton("Clear Log")
        clear_btn.clicked.connect(self.status_log.clear)
        clear_btn.setStyleSheet("padding: 8px; font-size: 12px;")
        
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
                color: white;                
                padding: 8px;
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
                color: white;                
                padding: 8px;
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
        self.ros_node.shutdown_signal.connect(self.close)
        self.ros_node.status_message.connect(self._on_status_message)
        self.ros_node.package_started.connect(self._on_package_started)
        self.ros_node.package_stopped.connect(self._on_package_stopped)

        self.control_panel.start_package.connect(self._on_start_package)
        self.control_panel.stop_package.connect(self._on_stop_package)
        self.control_panel.restart_package.connect(self._on_restart_package)
    
    # ========== Slot Methods ==========
    
    def _on_start_package(self, index: int):
        self.ros_node.start_package_by_index(index)
    
    def _on_stop_package(self, index: int):
        config = self.ros_node.get_package_info(index)
        if config:
            self.ros_node.stop_package(index)
    
    def _on_restart_package(self, index: int):
        config = self.ros_node.get_package_info(index)
        if config:
            pkg_type = LAUNCH if config.pkg_type == "launch" else RUN
            self.ros_node.restart_package(index, config.executable, pkg_type)
    
    def _on_status_message(self, message: str):
        self._log_message(message)
    
    def _on_package_started(self, package_id: int):
        self.control_panel.update_package_status(package_id, True)
    
    def _on_package_stopped(self, package_id: int):
        self.control_panel.update_package_status(package_id, False)
    
    def _stop_all_packages(self):
        self._log_message("Stopping all packages...")
        self.ros_node.process_manager.stop_all()

    def _start_all_packages(self):
        self._log_message("Starting all Debugging packages...")
        self.ros_node.start_all()

    def _tune_packages(self):
        self._log_message("Tunning walk packages...")
        self.ros_node.tune_start()

    def _without_UDP_packages(self):
        self._log_message("Starting all packages without UDP...")
        self.ros_node.without_UDP()
    
    # ========== Helper Methods ==========
    
    def _log_message(self, message: str):
        timestamp = QTime.currentTime().toString(settings.LOG_TIMESTAMP_FORMAT)
        formatted_message = f"[{timestamp}] {message}"
        
        self.status_log.append(formatted_message)
        
        if settings.AUTO_SCROLL_LOG:
            cursor = self.status_log.textCursor()
            cursor.movePosition(QTextCursor.End)
            self.status_log.setTextCursor(cursor)
    
    # ========== Event Handlers ==========
    
    def closeEvent(self, event):
        super().closeEvent(event)
