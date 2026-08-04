from setuptools import find_packages, setup

package_name = 'entry_level'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pawel',
    maintainer_email='ozgaapawell@gmail.com',
    description='Minimal publisher and subscriber',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'publisher1 = entry_level.publisher1:main',
            'subscriber1 = entry_level.subscriber1:main'
        ],
    },
)
