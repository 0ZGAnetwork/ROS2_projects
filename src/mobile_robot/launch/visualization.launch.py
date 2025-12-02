from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os

#/home/ozga/ros2_ws/src/mobile_robot/rviz
package_share = os.getcwd()          # ROS2_WS
src = os.path.join(package_share, "src")    # src
mobile_robot = os.path.join(src,"mobile_robot")     # project mobile_robot
config_path = os.path.join(mobile_robot,"config") 
launch_path = os.path.join(mobile_robot,"launch")
rviz_config = os.path.join(mobile_robot, 'rviz', 'mobile_robot_visual.rviz')

print("config:", config_path)
print("launch:", launch_path)
print("rviz_config:", launch_path)

#home/ozga/ros2_ws/install/mobile_robot/config/controller.yaml


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT','1'),

        Node(
            package='mobile_robot',
            executable='controller',
            name='controller',
            output='screen',
            parameters=[os.path.join(mobile_robot, 'config', 'controller.yaml')]
        ),
        
        Node(
            package='mobile_robot',
            executable='robot_monitor_sim',
            name='robot_monitor_sim',
            output='screen',
            parameters=[os.path.join(mobile_robot, 'config', 'simulator.yaml')]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            #/home/ozga/ros2_ws/src/mobile_robot/rviz
            # arguments=['-d', os.path.join(package_share, 'rviz', 'mobile_robot_visual.rviz')]
            arguments=['-d', rviz_config]
        ),
    ])
    
