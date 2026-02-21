#다시 쓸 수 있는 위젯들
from PyQt5.QtWidgets import (
    QWidget, QHBoxLayout, QPushButton, QLabel, QVBoxLayout, QGroupBox
)
from PyQt5.QtCore import pyqtSignal, Qt
from ...utils.constants import COLOR_RUNNING, COLOR_STOPPED, SYMBOL_RUNNING, SYMBOL_STOPPED


class PackageControlWidget(QWidget):
    start_clicked = pyqtSignal(int)  # package_index
    stop_clicked = pyqtSignal(int)
    restart_clicked = pyqtSignal(int)
    
    def __init__(self, package_index: int, package_name: str, description: str = "", parent=None):
        super().__init__(parent)
        
        self.package_index = package_index
        self.package_name = package_name
        self.description = description
        self.is_running = False
        
        self._setup_ui()
        self._connect_signals()
        self._update_status_display()
    
    def _setup_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        
        self.status_label = QLabel()
        self.status_label.setMinimumWidth(300)
        self.status_label.setStyleSheet("font-size: 14px;")

        self.start_btn = QPushButton("Start")
        self.start_btn.setMinimumWidth(80)
        self.start_btn.setStyleSheet("background-color: #4CAF50; color: white;")
        
        self.stop_btn = QPushButton("Kill")
        self.stop_btn.setMinimumWidth(80)
        self.stop_btn.setStyleSheet("background-color: #f44336; color: white;")
        self.stop_btn.setEnabled(False)
        
        self.restart_btn = QPushButton("Restart")
        self.restart_btn.setMinimumWidth(80)
        self.restart_btn.setStyleSheet("background-color: #FF9800; color: white;")
        self.restart_btn.setEnabled(False)

        layout.addWidget(self.status_label)
        layout.addStretch()
        layout.addWidget(self.start_btn)
        layout.addWidget(self.stop_btn)
        layout.addWidget(self.restart_btn)
    
    def _connect_signals(self):
        self.start_btn.clicked.connect(
            lambda: self.start_clicked.emit(self.package_index)
        )
        self.stop_btn.clicked.connect(
            lambda: self.stop_clicked.emit(self.package_index)
        )
        self.restart_btn.clicked.connect(
            lambda: self.restart_clicked.emit(self.package_index)
        )
    
    def set_running(self, is_running: bool):
        self.is_running = is_running
        self._update_status_display()
        self._update_button_states()
    
    def _update_status_display(self):
        if self.is_running:
            symbol = SYMBOL_RUNNING
            color = COLOR_RUNNING
            status = "Running"
        else:
            symbol = SYMBOL_STOPPED
            color = COLOR_STOPPED
            status = "Stopped"
        
        text = f"{symbol} Package {self.package_index}: {self.package_name}"
        if self.description:
            text += f" ({self.description})"
        
        self.status_label.setText(text)
        self.status_label.setStyleSheet(f"color: {color}; font-weight: bold; font-size: 14px;")
    
    def _update_button_states(self):
        self.start_btn.setEnabled(not self.is_running)
        self.stop_btn.setEnabled(self.is_running)
        self.restart_btn.setEnabled(self.is_running)


class PackageControlPanel(QGroupBox):
    start_package = pyqtSignal(int)
    stop_package = pyqtSignal(int)
    restart_package = pyqtSignal(int)
    
    def __init__(self, packages: dict, parent=None):
        super().__init__("Package Control", parent)
        
        self.package_widgets = {}
        self._setup_ui(packages)
    
    def _setup_ui(self, packages):
        layout = QVBoxLayout(self)
        
        for index, config in packages.items():
            if config.name:
                widget = PackageControlWidget(
                    index, 
                    config.name, 
                    config.description
                )
                
                widget.start_clicked.connect(self.start_package.emit)
                widget.stop_clicked.connect(self.stop_package.emit)
                widget.restart_clicked.connect(self.restart_package.emit)
                
                self.package_widgets[index] = widget
                layout.addWidget(widget)
        
        layout.addStretch()
    
    def update_package_status(self, package_id: int, is_running: bool):
        if package_id in self.package_widgets:
            self.package_widgets[package_id].set_running(is_running)
    
    def get_widget(self, package_id: int) -> PackageControlWidget:
        return self.package_widgets.get(package_id)
