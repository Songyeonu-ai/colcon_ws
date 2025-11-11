import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='ocam_ros2',
            executable='ocam_ros2',
            name='ocam_ros2',
            output='screen',
            parameters=[
                {"resolution": 2},
                {"frame_rate": 15.0},
                {"exposure": 180},
                {"gain": 110},
                {"wb_blue": 250},
                {"wb_red": 130},
                {"auto_exposure": False},
                {"show_image": True},
            ]
        )
    ])
