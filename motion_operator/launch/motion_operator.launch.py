import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('motion_operator'),
        'config',
        'config.yaml'
    )

    motion_operator_node = Node(
        package='motion_operator',
        executable='motion_operator_node',
        name='motion_operator_node',
        output='screen',
        emulate_tty=True,
        parameters=[config_dir]
    )

    return LaunchDescription([motion_operator_node])
