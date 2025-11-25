
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),

        Node(
            package='robot_monitoring',
            executable='state_monitor',
            name='state_monitor',
            output='screen',
        ),
        Node(
            package='robot_monitoring',
            executable='error_handler',
            name='error_handler',
            output='screen',
        ),
        # Node(
        #     package='robot_monitoring',
        #     executable='sensor1',
        #     name='sensor1',
        #     output='screen',
        # ),
        # Node(
        #     package='robot_monitoring',
        #     executable='sensor2',
        #     name='sensor2',
        #     output='screen',
        # ),
    ])