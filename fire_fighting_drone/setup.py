from setuptools import find_packages, setup

package_name = 'fire_fighting_drone'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/mission.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/params.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ashish',
    maintainer_email='ashish@todo.todo',
    description=(
        'Flight control, trajectory generation/following, and YOLO-based '
        'human detection nodes for the Fire-Fighting-Drone project (ROS 2 Jazzy).'
    ),
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flight_controller = fire_fighting_drone.flight_controller:main',
            'trajectory_follower = fire_fighting_drone.trajectory_follower:main',
            'object_detection = fire_fighting_drone.object_detection:main',
        ],
    },
)
