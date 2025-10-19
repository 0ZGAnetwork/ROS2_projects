from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='generator',
            name='generator_node',
            output='screen',
        ),
        Node(
            package='my_package',
            executable='scaler',
            name='scaler_node',
            output='screen',
        ),
        Node(
            package='my_package',
            executable='visualtools',
            name='visualtools_node',
            output='screen',
        ),
    ])