from setuptools import find_packages, setup

package_name = 'mg400_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/common/config', [
            'mg400_controller/common/config/alarmController.json',
            'mg400_controller/common/config/alarmServo.json',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='th0sun',
    maintainer_email='m.hassunofficial@gmail.com',
    description='ROS2 teleoperation controller for Dobot MG400.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vr_teleop_node = mg400_controller.vr_teleop_node:main',
            'monitor_gui = mg400_controller.monitor_gui:main',
        ],
    },
)
