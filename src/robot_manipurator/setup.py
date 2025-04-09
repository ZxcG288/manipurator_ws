from setuptools import find_packages, setup

package_name = 'robot_manipurator'

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
    maintainer='golf1234_pc',
    maintainer_email='kangval.work@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "talker = robot_manipurator.talker1:main",
            "listener = robot_manipurator.listener1:main",
            "project1 = robot_manipurator.project1:main",
            "servo_test1 = robot_manipurator.servo_test1:main",
            "subscriber_node = robot_manipurator.subscriber:main",
            "subscriber1_node = robot_manipurator.subscriber1:main",
            "web_test1_node = robot_manipurator.web_test1:main",
            "real_robot = robot_manipurator.real_robot:main",
            "pick_place_controller = robot_manipurator.pick_place_controller:main",
        ],
    },
)
