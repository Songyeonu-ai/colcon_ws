import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ...utils.geometry import hz_to_ms

from robocup_tc_interfaces.msg import (
    ObNavObstacleInfo,
    ObNavUiCommand,
    ObNavPlannerState,
)

from .plan_logic import TaskPlanner
from ...utils.ros2_interface import (
    obstacle_msg_to_observation,
    planner_output_to_state_msg,
)


class TaskPlannerNode(Node):
    def __init__(self):
        super().__init__("task_planner_node")
        self.timer_ = self.create_timer(hz_to_ms(25), self._on_timer)

    def _task_plan_tick(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = TaskPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()