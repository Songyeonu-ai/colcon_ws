from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision_tune',
            executable='camera_node',
            output='screen'
        ),
        Node(
            package='vision_tune',
            executable='process_node',
            output='screen'
        ),
        Node(
            package='vision_tune',
            executable='ui_node',
            output='screen'
        ),
    ])