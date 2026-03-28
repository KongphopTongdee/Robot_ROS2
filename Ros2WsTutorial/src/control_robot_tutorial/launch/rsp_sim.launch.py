# ----------Import the nessary library----------
# Import the system library
import os

# Import the xacro library
import xacro

# Import launch library
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Import get package directory
from ament_index_python.packages import get_package_share_directory

# Create the call launch function
def generate_launch_description():
    # Specify the name of the package and path to xacro file within the package
    pkg_name = "control_robot_tutorial"

    # Use xacro to process the file
    xacro_file = os.path.join( get_package_share_directory( pkg_name ), 'description', 'example_robot_clip7.urdf.xacro' )
    robot_description_raw = xacro.process_file( xacro_file ).toxml()

    # Create robot state publisher node 
    parames = { 'robot_description': robot_description_raw, 'use_sim_time': True }
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[parames]
    )

    # Create gazebo node by using launch file
    gazebo_node = IncludeLaunchDescription(                                                 # IncludeLaunchDescription = call the launch function
        PythonLaunchDescriptionSource([os.path.join(                                        # PythonLaunchDescriptionSource = call the launch python file
            get_package_share_directory( 'gazebo_ros' ), 'launch'), '/gazebo.launch.py'     
        ]),
        )                                                   

    # Create spawn entity node with arguments
    argumentSpawn = [ "-topic", "robot_description", "-entity", "robot" ]
    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=argumentSpawn,
        output="screen",
    )

    # Create rviz2 node to visualize result
    argumentRviz2 = [ "-d", "src/control_robot_tutorial/config/visualize_rviz2_config_clip7AndClip8.rviz" ]
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=argumentRviz2,
        output="screen",
    )

    # Launch! (Run the node without configuration)
    return LaunchDescription([
        gazebo_node,
        node_robot_state_publisher,
        spawn_entity_node,
        rviz2_node
    ])
