from setuptools import find_packages, setup

package_name = 'tb3_straight_avoid'

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
    maintainer='Vinay Hulibandi',
    maintainer_email='vinayhulibandi735@gmail.com',
    description='TurtleBot3 straight line motion with obstacle avoidance and yaw correction',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'straight_avoid = tb3_straight_avoid.straight_line_avoid:main',
            'straight_avoid_2 = tb3_straight_avoid.straight_line_avoid_2:main',
        ],
    },
)
