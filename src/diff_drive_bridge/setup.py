from setuptools import setup

package_name = 'diff_drive_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mohamed',
    maintainer_email='test@test.com',
    description='Bridge from /cmd_vel (Twist) to /wheel_cmd ("R L") for hoverboard robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_to_wheel = diff_drive_bridge.cmd_vel_to_wheel:main',
        ],
    },
)
