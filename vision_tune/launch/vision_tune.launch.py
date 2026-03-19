from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('vision_tune'),
        'config',
        'camera_config.yaml'
    )

    info_dir = os.path.join(
        get_package_share_directory('vision_tune'),
        'config',
        'camera_info_config.yaml'
    )

    param_dir = os.path.join(
        get_package_share_directory('vision_tune'),
        'config',
        'ui_config.yaml'
    )

    with open(config_dir, 'r') as file:
        config_params = yaml.safe_load(file)
        camera_name = config_params['/**']['ros__parameters']['camera_name']

    node_name = f'pan_tilt_camera_node_{camera_name}'

    pan_tilt_camera_node = Node(
        package='vision_tune',
        executable='pan_tilt_camera_node',
        name=node_name,
        output='screen',
        parameters=[config_dir, info_dir]
    )

    process_node = Node(
        package='vision_tune',
        executable='process_node',
        name='process_node',
        output='screen',
        parameters=[param_dir]
    )

    ui_node = Node(
        package='vision_tune',
        executable='ui_node',
        name='ui_node',
        output='screen',
        parameters=[param_dir]
    )

    return LaunchDescription([
        pan_tilt_camera_node,
        process_node,
        ui_node,
    ])