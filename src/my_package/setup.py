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
<<<<<<< HEAD
        ('share/' + package_name + '/config', ['config/generator_params.yaml']),
        ('share/' + package_name + '/config', ['config/scaler_params.yaml']),
        ('share/' + package_name + '/launch',['launch/my.launch.py'])
=======
>>>>>>> 287fb1b (INitial commit with working talker package)
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ozga',
    maintainer_email='ozga@todo.todo',
<<<<<<< HEAD
    description='my package',
    license='MIT',
=======
    description='TODO: Package description',
    license='TODO: License declaration',
>>>>>>> 287fb1b (INitial commit with working talker package)
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
<<<<<<< HEAD
        'generator = my_package.generator:main',
        'visualtools = my_package.VisualTools:main',
        'scaler = my_package.scaler:main',
=======
		'talker = my_package.talker:main',
>>>>>>> 287fb1b (INitial commit with working talker package)
        ],
    },
)
