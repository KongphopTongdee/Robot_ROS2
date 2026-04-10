from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
import os 
import xacro
# Import the launch library

# Create function to call the launch file
def generate_launch_description():

    # Create path to call the URDF file
    pkg_path = os.path.join( get_package_share_directory( 'diff_drive_3_legs' ) )
    xacro_file = os.path.join( pkg_path, 'description', 'main.urdf.xacro' )
    robot_description_config = xacro.process_file( xacro_file )

    # Create path to call the config file
    rviz_config_file = os.path.join( pkg_path, "config", "config_visualize_robot.rviz" )

    # Create the object launchDescription
    ld = LaunchDescription()

    # Coding node to call here
    # Create node robot_state_publisher
    params = { 'robot_description': robot_description_config.toxml() }
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",                                                        # output the result into the terminal
        parameters=[params]                                                     # call robot description
    )

    # Test only publish to rviz2
    paramsRviz2 = [ '-d', rviz_config_file ]
    node_rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments= paramsRviz2,
        output="screen",
    )

    # Test only with joint_state_publisher_gui
    node_joint_state_publisher_gui = Node( 
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    # Call the all launch node in the file.launch.xml
    # If you want to run micro-ros please insert this code in robot_bring_up.launch.xml: <node pkg="micro_ros_agent" exec="micro_ros_agent" name="micro_ROS" args="serial --dev /dev/ttyUSB0" />
    robot_ros_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('diff_drive_3_legs'),
                "launch/robot_bring_up.launch.xml"
            )
        )
    )

    # Add the action to call node launch 
    ld.add_action( node_robot_state_publisher )
    ld.add_action( robot_ros_launch )
    # ld.add_action( node_joint_state_publisher_gui )
    ld.add_action( node_rviz2 )

    return ld
