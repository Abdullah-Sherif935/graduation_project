import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. تحديد مسار ملف لانش الكاميرا (XML)
    astra_camera_path = get_package_share_directory('astra_camera')
    camera_launch_file = os.path.join(astra_camera_path, 'launch', 'astra_pro.launch.xml')

    return LaunchDescription([
        
        # أ- تشغيل الكاميرا
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(camera_launch_file)
        ),

        # ب- تعريف التحويلات الثابتة (Static TFs) بناءً على مقاساتك
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_lidar',
            arguments=['0.08', '0.005', '0.28', '0', '0', '0', 'base_link', 'lidar_link']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_camera',
            arguments=['0.26', '-0.235', '0.90', '0', '0', '0', 'base_link', 'camera_link']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_arm',
            arguments=['0.32', '0.04', '0.81', '0', '0', '0', 'base_link', 'arm_base_link']
        ),

        # ج- تشغيل كود الـ YOLO
        Node(
            package='lever_vision_model',
            executable='lever_detector',  # شيل الـ .py من هنا
            name='lever_pose_detector',
            output='screen'
        ),
    ])
