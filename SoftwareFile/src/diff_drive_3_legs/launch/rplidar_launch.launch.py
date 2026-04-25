import os
from launch_ros.actions import Node
from launch import LaunchDescription
# Import the launch library


# Create function to call the launch file
def generate_launch_description():

    # Create node of rplidar sensor c1
    paramOfRplidarC1 = {
          "channel_type": "serial",
            # Need to be setting with -ls /dev/serial/by-path/ for all continue usage port 
            # This was serial by-path of the first port in msi notebook : /dev/serial/by-path/pci-0000:00:14.0-usb-0:4:1.0-port0
            # This was serial by-path of the first port in raspberry pi 5 : /dev/serial/by-path/platform-xhci-hcd.1-usb-0:1:1.0-port0
          "serial_port": "/dev/serial/by-path/platform-xhci-hcd.1-usb-0:1:1.0-port0",            
          "serial_baudrate": 460800,
          "frame_id": "cylinderRplidar_link",
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

    return LaunchDescription([
        rplidar_c1_node,
    ])