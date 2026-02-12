"""
Main entry point for Package Manager
"""

import sys
import rclpy
from threading import Thread
from PyQt5.QtWidgets import QApplication

from package_manager_py.core import RosNode
from package_manager_py.ui import MainWindow


def ros_spin_thread(ros_node):
    """
    ROS2 spin function for background thread
    
    Args:
        ros_node: ROS2 node instance
    """
    rclpy.spin(ros_node)


def main(args=None):
    """Main application entry point"""
    
    app = QApplication(sys.argv)
    rclpy.init(args=args)
    ros_node = RosNode()
    
    # Create main window
    window = MainWindow(ros_node)
    window.show()
    
    # Start ROS2 in background thread
    ros_thread = Thread(
        target=ros_spin_thread,
        args=(ros_node,),
        daemon=True
    )
    ros_thread.start()
    
    # Run Qt event loop
    exit_code = app.exec_()
    
    # Cleanup
    ros_node.shutdown_node()
    
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
