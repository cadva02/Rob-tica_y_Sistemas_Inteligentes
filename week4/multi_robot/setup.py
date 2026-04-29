import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'multi_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.stl'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adrian',
    maintainer_email='a01735818@tec.mx',
    description='Multi-robot simulation with kinematic models',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'kinematic_simulator = multi_robot.kinematic_simulator:main',
            'dead_reckoning_localization = multi_robot.localization:main',
            'point_stabilizer = multi_robot.point_stabilizer:main',
            'setpoint_generator = multi_robot.setpoint_generator:main',
            'joint_state_publisher = multi_robot.joint_state_publisher:main',
        ],
    },
)
