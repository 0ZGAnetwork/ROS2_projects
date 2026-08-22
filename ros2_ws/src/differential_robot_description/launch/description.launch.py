from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('urdf_launch'),
                'launch',
                'description.launch.py'
            ]),
            launch_arguments={
                'urdf_package': 'differential_robot_description',
                'urdf_package_path': PathJoinSubstitution([
                    'urdf',
                    'differential_robot.urdf.xacro'
                ])
            }.items()
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=[
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('differential_robot_description'),
                    'config',
                    'differential_robot.rviz'
                ])
            ]
        )

    ])