from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='differential_robot_control',
            executable='wheel_diff_controller_cpp',
            name='wheel_diff_controller',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('differential_robot_control'),
                    'config',
                    'controllers.yaml'
                ])
            ],
            output='screen'
        )
    ])