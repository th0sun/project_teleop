# /project_teleop_ws/src/dobot_mg400/mg400_bringup/launch/main.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # 1. หาไฟล์โมเดล (URDF/Xacro)
    pkg_name = 'mg400_description'
    file_subpath = 'urdf/mg400.urdf.xacro'
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    
    # 2. แปลง Xacro เป็น XML
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 3. path ของไฟล์ rviz
    rviz_config = os.path.join(
        get_package_share_directory('mg400_description'),
        'rviz',
        'mg400_simulation.rviz'
    )

    return LaunchDescription([

        # 4. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_raw}]
        ),

        # 5. Rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
