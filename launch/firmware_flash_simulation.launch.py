from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dexi_tools',
            executable='firmware_flash_node',
            name='firmware_flash_node',
            output='screen',
            parameters=[
                {'simulation_mode': True}
            ],
        ),
    ])