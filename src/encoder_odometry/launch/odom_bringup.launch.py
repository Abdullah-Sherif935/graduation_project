import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # جلب مسار ملف الـ RViz الذي قمنا بحفظه
    pkg_share = get_package_share_directory('encoder_odometry')
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'odom_config.rviz')

    return LaunchDescription([
        # 1. نود الأودومتري
        Node(
            package='encoder_odometry',
            executable='encoder_odom_node',
            name='encoder_odometry_node',
            parameters=[{'port': '/dev/ttyACM0'}],
            output='screen'
        ),

        # 2. نود RViz مع تمرير مسار ملف الإعدادات
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path], # هذا السطر هو السر
            output='screen'
        )
    ])