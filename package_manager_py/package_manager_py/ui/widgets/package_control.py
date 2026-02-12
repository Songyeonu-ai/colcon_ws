"""
Reusable Package Control Widget
"""

from PyQt5.QtWidgets import (
    QWidget, QHBoxLayout, QPushButton, QLabel, QVBoxLayout, QGroupBox
)
from PyQt5.QtCore import pyqtSignal, Qt
from ...utils.constants import COLOR_RUNNING, COLOR_STOPPED, SYMBOL_RUNNING, SYMBOL_STOPPED


class PackageControlWidget(QWidget):
    """
    Reusable widget for controlling a single package
    
    Signals:
        start_clicked(int): Emitted when start button clicked
        stop_clicked(int): Emitted when stop button clicked
        restart_clicked(int): Emitted when restart button clicked
    """
    
    start_clicked = pyqtSignal(int)  # package_index
    stop_clicked = pyqtSignal(int)
    restart_clicked = pyqtSignal(int)
    
    def __init__(self, package_index: int, package_name: str, description: str = "", parent=None):
        """
        Initialize package control widget
        
        Args:
            package_index: Package index (1-8)
            package_name: Package name
            description: Package description (optional)
            parent: Parent widget
        """
        super().__init__(parent)
        
        self.package_index = package_index
        self.package_name = package_name
        self.description = description
        self.is_running = False
        
        self._setup_ui()
        self._connect_signals()
        self._update_status_display()
    
    def _setup_ui(self):
        """Setup user interface"""
        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        
        # Status label (shows running state)
        self.status_label = QLabel()
        self.status_label.setMinimumWidth(300)
        self.status_label.setStyleSheet("font-size: 14px;")
        
        # Control buttons
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
        
        # Add widgets to layout
        layout.addWidget(self.status_label)
        layout.addStretch()
        layout.addWidget(self.start_btn)
        layout.addWidget(self.stop_btn)
        layout.addWidget(self.restart_btn)
    
    def _connect_signals(self):
        """Connect button signals"""
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
        """
        Update running state
        
        Args:
            is_running: True if package is running
        """
        self.is_running = is_running
        self._update_status_display()
        self._update_button_states()
    
    def _update_status_display(self):
        """Update status label based on running state"""
        if self.is_running:
            symbol = SYMBOL_RUNNING
            color = COLOR_RUNNING
            status = "Running"
        else:
            symbol = SYMBOL_STOPPED
            color = COLOR_STOPPED
            status = "Stopped"
        
        # Format label text
        text = f"{symbol} Package {self.package_index}: {self.package_name}"
        if self.description:
            text += f" ({self.description})"
        
        self.status_label.setText(text)
        self.status_label.setStyleSheet(f"color: {color}; font-weight: bold; font-size: 14px;")
    
    def _update_button_states(self):
        """Update button enabled/disabled states"""
        self.start_btn.setEnabled(not self.is_running)
        self.stop_btn.setEnabled(self.is_running)
        self.restart_btn.setEnabled(self.is_running)


class PackageControlPanel(QGroupBox):
    """
    Panel containing multiple PackageControlWidgets
    
    Signals:
        start_package(int): Request to start package
        stop_package(int): Request to stop package  
        restart_package(int): Request to restart package
    """
    
    start_package = pyqtSignal(int)
    stop_package = pyqtSignal(int)
    restart_package = pyqtSignal(int)
    
    def __init__(self, packages: dict, parent=None):
        """
        Initialize package control panel
        
        Args:
            packages: Dictionary of {index: PackageConfig}
            parent: Parent widget
        """
        super().__init__("Package Control", parent)
        
        self.package_widgets = {}
        self._setup_ui(packages)
    
    def _setup_ui(self, packages):
        """Setup user interface"""
        layout = QVBoxLayout(self)
        
        # Create widget for each package
        for index, config in packages.items():
            if config.name:  # Only create widget if package is configured
                widget = PackageControlWidget(
                    index, 
                    config.name, 
                    config.description
                )
                
                # Connect signals
                widget.start_clicked.connect(self.start_package.emit)
                widget.stop_clicked.connect(self.stop_package.emit)
                widget.restart_clicked.connect(self.restart_package.emit)
                
                # Store and add to layout
                self.package_widgets[config.name] = widget
                layout.addWidget(widget)
        
        layout.addStretch()
    
    def update_package_status(self, package_name: str, is_running: bool):
        """
        Update status of a package
        
        Args:
            package_name: Package name
            is_running: Running state
        """
        if package_name in self.package_widgets:
            self.package_widgets[package_name].set_running(is_running)
    
    def get_widget(self, package_name: str) -> PackageControlWidget:
        """Get widget for a specific package"""
        return self.package_widgets.get(package_name)
