from setuptools import find_packages, setup

package_name = 'mobile_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/visualization.launch.py']),
        ('share/' + package_name + '/config', ['config/controller.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ozga',
    maintainer_email='ozgaapawell@gmail.com',
    description='Mobile_Robot',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller = mobile_robot.controller:main',
            'robot_monitor_sim = mobile_robot.robot_monitor_sim:main',
        ],
    },
)
