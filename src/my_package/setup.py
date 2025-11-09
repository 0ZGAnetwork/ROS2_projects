from setuptools import find_packages, setup

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/generator_params.yaml']),
        ('share/' + package_name + '/config', ['config/scaler_params.yaml']),
        ('share/' + package_name + '/launch',['launch/my.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ozga',
    maintainer_email='ozga@todo.todo',
    description='my package',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'generator = my_package.generator:main',
        'visualtools = my_package.VisualTools:main',
        'scaler = my_package.scaler:main',
        ],
    },
)
