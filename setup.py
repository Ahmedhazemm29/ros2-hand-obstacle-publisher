from setuptools import find_packages, setup

package_name = 'hand_obstacle_publisher'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ahmed Hazem',
    maintainer_email='a214.shams@gmail.com',
    description=(
        'Human hand / body detections to live MoveIt2 collision objects: '
        'subscribes to pixel-space detections, builds 3D obstacles and '
        'publishes atomic planning-scene diffs.'),
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hand_obstacle_publisher = '
            'hand_obstacle_publisher.hand_to_collision:main',
        ],
    },
)
