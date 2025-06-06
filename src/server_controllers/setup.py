from setuptools import find_packages, setup

package_name = 'server_controllers'

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
    maintainer_email='kangval.work@egmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "node_manager = server_controllers.node_manager:main",
            "node1 = server_controllers.node1:main",
            "node2 = server_controllers.node2:main"
        ],
    },
)
