from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'control_robot_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add function to call path of launch file
        ( os.path.join( 'share', package_name, 'launch' ), glob( os.path.join( 'launch','*launch.[pxy][yma]*' ) ) ),
        # Add function to call path of xacro file
        (os.path.join('share', package_name, 'description'),glob( os.path.join('description','*.xacro'))),
        # Add function to call calculate mesh
        (os.path.join('share', package_name, 'meshes'),glob('meshes/*')),
        # Add function to call config world gazebo file
        (os.path.join('share', package_name, 'config'),glob(os.path.join( 'config', 'gazebo_world/*.world' ))),
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
    # Input the new create node in entry_points( for using ros2 run ... function )
    # In console_scripts, the input was => "[name of ros2 execution] = [package name].[new python file name(new file node name)]:[function to run]" 
    entry_points={
        'console_scripts': [
            "test_node = control_robot_tutorial.my_first_node:main",
            "draw_circle_node = control_robot_tutorial.turtle_draw_circle:main",
            "pose_subscribe_node = control_robot_tutorial.pose_subscriber:main",
            "turtle_controller = control_robot_tutorial.turtle_controller:main",
        ],
    },
)
