from setuptools import setup

package_name = 'wheel_teleop'

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
    maintainer='abdullah',
    maintainer_email='test@test.com',
    description='Keyboard teleop for hoverboard, publishes /wheel_cmd as "R L".',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop = wheel_teleop.keyboard_teleop:main',
        ],
    },
)
