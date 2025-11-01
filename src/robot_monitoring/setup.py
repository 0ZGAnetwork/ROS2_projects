from setuptools import find_packages, setup

package_name = 'robot_monitoring'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/sensors.yaml']),
        #('share/' + package_name + '/launch', ['launch/stary_system.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ozga',
    maintainer_email='ozgaapawell@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sensor1 = robot_monitoring.sensor1:main',
            'sensor2 = robot_monitoring.sensor2:main',
            'state_monitor = robot_monitoring.state_monitor:main',
            'error_handler = robot_monitoring.error_handler:main',
        ],
    },
)
