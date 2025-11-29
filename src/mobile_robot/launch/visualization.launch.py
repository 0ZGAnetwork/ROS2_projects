from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os

package_share = get_package_share_directory('mobile_robot')

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT','1'),

        Node(
            package='mobile_robot',
            executable='controller',
            name='controller',
            output='screen',
            parameters=[os.path.join(package_share, 'config', 'controller.yaml')]
        ),

        Node(
            package='mobile_robot',
            executable='robot_monitor_sim',
            name='robot_monitoring_sim',
            output='screen',
            parameters=[os.path.join(package_share, 'config', 'simulator.yaml')]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(package_share, 'rviz', 'mobile_robot_visual.rviz')]
        ),
    ])
    
