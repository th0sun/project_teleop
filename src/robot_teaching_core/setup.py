from setuptools import find_packages, setup

package_name = 'robot_teaching_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='th0sun',
    maintainer_email='m.hassunofficial@gmail.com',
    description=(
        'Robot-neutral core: canonical program IR, capability profile, '
        'kinematics seam, adapter-API contracts. Must not import from '
        'robot-specific adapters.'
    ),
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [],
    },
)
