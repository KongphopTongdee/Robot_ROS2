from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'diff_drive_3_legs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # os.path.join( 'share', package_name, 'launch' ) => sets the destination in the install folder
    # glob( os.path.join( 'launch','*launch.[pxy][yma]*' ) ) => Finds all files in the launch directory ending in .launch.py, .launch.xml, .launch.yaml
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add function to call path of launch file
        ( os.path.join( 'share', package_name, 'launch' ), glob( os.path.join( 'launch','*launch.[pxy][yma]*' ) ) ),
        # Add function to call path of xacro file
        (os.path.join('share', package_name, 'description'),glob( os.path.join('description','*.xacro'))),
        # Add function to call path of config rviz2 file
        (os.path.join('share', package_name, 'config'),glob( os.path.join('config','*.rviz'))),
        # Add function to call calculate mesh
        (os.path.join('share', package_name, 'meshes'),glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kongphop666',
    maintainer_email='kongphop.working@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "test_node_only = diff_drive_3_legs.test_file:main",
            "control_robot_node = diff_drive_3_legs.controller_robot_diff_drive:main",
            "odometry_node = diff_drive_3_legs.odometry:main",
            "joint_state_publisher_node = diff_drive_3_legs.joint_state_publisher_myself:main",
        ],
    },
)
