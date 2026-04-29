from setuptools import setup
import os
from glob import glob

# تعريف الاسم هنا مرة واحدة
package_name = 'lever_vision_model'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # السطر ده هو اللي بينقل ملفات الـ launch لفولدر الـ install
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abdullah',
    maintainer_email='abdullah@todo.todo',
    description='Lever Detection Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # هنا بنرص كل الأوامر ورا بعض في نفس القائمة
            'lever_detector = lever_vision_model.lever_detector:main',
            'vision_moveit_coordinator = lever_vision_model.vision_moveit_coordinator:main',
        ],
    },
)
