import sys

import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication

from robocup_tc_interfaces.msg import ObNavUiCommand, ObNavPlannerState

from .window import MainWindow


class UiNode(Node):
    def __init__(self):
        super().__init__("ui_node")


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    node = UiNode()
    window = MainWindow(node)
    window.show()

    exit_code = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)