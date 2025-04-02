from setuptools import find_packages, setup

package_name = 'camera2_ros'

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
    maintainer='g288-pc',
    maintainer_email='g288-pc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "camera2_node = camera2_ros.camera2_ros:main",
        "sub_yolo = camera2_ros.sub_yolo:main",
        "arduino_comunication = camera2_ros.arduino_comunication:main",
        "box_dectection_pick = camera2_ros.box_dectection_pick:main",
        "box_dectection_place = camera2_ros.box_dectection_place:main",
        "pub_yolo = camera2_ros.pub_yolo:main",
        "lane_detection = camera2_ros.lane_detection:main",
        ],
    },
)
