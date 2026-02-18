import sys
import rclpy
from threading import Thread
from PyQt5.QtWidgets import QApplication

from package_manager_py.core import RosNode
from package_manager_py.ui import MainWindow


def ros_spin_thread(ros_node):
    rclpy.spin(ros_node)


def main(args=None):
    app = QApplication(sys.argv)
    rclpy.init(args=args)
    ros_node = RosNode()
    
    window = MainWindow(ros_node)
    window.show()
    
    ros_thread = Thread(
        target=ros_spin_thread,
        args=(ros_node,),
        daemon=True
    )
    ros_thread.start()

    exit_code = app.exec_()
    
    ros_node.shutdown_node()
    
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
