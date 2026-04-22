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


    # ---------- Coding launch file to call here ----------
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


    # ---------- Coding node to call here ----------
    # Create node robot_state_publisher
    paramOfRobotDescription = { 'robot_description': robot_description_config.toxml() }
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",                                                        # output the result into the terminal
        parameters=[paramOfRobotDescription]                                                     # call robot description
    )

    # Create node of rplidar sensor c1
    paramOfRplidarC1 = {
          "channel_type": "serial",
            # Need to be setting with -ls /dev/serial/by-path/ for all continue usage port 
            # This was serial by-path of the first port
          "serial_port": "/dev/serial/by-path/pci-0000:00:14.0-usb-0:4:1.0-port0",            
          "serial_baudrate": 460800,
          "frame_id": "odom",
          "inverted": False,
          "angle_compensate": True,
          "scan_mode": "Standard",
    }
    rplidar_c1_node = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        output="screen",
        parameters=[paramOfRplidarC1]
    )

    # Test only publish to rviz2
    paramsRviz2 = [ '-d', rviz_config_file ]
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments= paramsRviz2,
        output="screen",
    )

    # Test only with joint_state_publisher_gui
    joint_state_publisher_gui_node = Node( 
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )



    # Add the action to call node launch 
    return LaunchDescription([
        robot_state_publisher_node,
        robot_ros_launch,
        # joint_state_publisher_gui_node,
        rviz2_node,
        rplidar_c1_node,
    ])
