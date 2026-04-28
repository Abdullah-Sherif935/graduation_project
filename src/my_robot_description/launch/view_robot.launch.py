import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    pkg_name = 'my_robot_description'
    pkg_share = get_package_share_directory(pkg_name)

    # URDF/XACRO file
    xacro_file = os.path.join(pkg_share, 'urdf', 'final2.xml')

    doc = xacro.process_file(xacro_file)
    robot_description = {"robot_description": doc.toxml()}

    # 1. تحديد مسار ملف إعدادات RViz
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'view_robot.rviz')

    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Joint State Publisher GUI
    jsp = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        # 2. تمرير ملف الإعدادات كأرجومنت عشان يفتح عليه أوتوماتيك
        arguments=['-d', rviz_config_path] 
    )

    return LaunchDescription([
        rsp,
        jsp,
        rviz
    ])