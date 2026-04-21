# Import os library
import os

# Import get package path
from ament_index_python.packages import get_package_share_directory

# Import launch file
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration

# Create function to call launch
def generate_launch_description():
    # Call the launch file state_publisher_node
    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ os.path.join( 
            get_package_share_directory( 'control_robot_tutorial' ), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time':"true"}.items()
    )
    
    # Get the gazebo world path
    gazebo_world_path = os.path.join(
        get_package_share_directory( 'control_robot_tutorial' ),'world','obstacles.world'
    )
    world = LaunchConfiguration( 'world', default=gazebo_world_path )

    # Create gazebo node
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join( 
            get_package_share_directory('gazebo_ros'), 'launch','gazebo.launch.py' )]),
            launch_arguments={'gz_args':world}.items(),
    )

    # Create spawn entity node 
    argumentSpawnEntity = [ "-topic", "robot_description", "-entity", "my_bot" ]
    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments= argumentSpawnEntity,
        output="screen",
    )
    argumentRviz = [ "-d", "src/control_robot_tutorial/config/drive_bot.rviz" ]
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=argumentRviz,
    )

    rplidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        output="screen",
        parameters=[{
            "serial_port": "/dev/ttyUSB0",
            "frame_id" : "laser_link",
            "angle_compensate" : True,
            "scan_mode" : "Standard"
        }]
    )

    diff_drive_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        # output="screen",
    )

    joint_board_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_board"],
        # output="screen",
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_node,
        spawn_entity_node,
        # rviz2_node,
        # rplidar_node,
        diff_drive_spawner_node,
        joint_board_spawner_node,
    ])
