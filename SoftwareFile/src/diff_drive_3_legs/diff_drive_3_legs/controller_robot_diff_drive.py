#!/usr/bin/env python3
# Interpreter line define to use python3( get the shebang to use python run )

# Package ROS2 with python
import rclpy
# Package ROS2 for create Node
from rclpy.node import Node

# ---------- Imoport library ----------
# Need to add the package below in the package.xml file
# Package for message type twist
from geometry_msgs.msg import Twist
# Package for message type int8
from std_msgs.msg import Int8
# Package for provides access to system-specific parameters and functions (used for keyboard input)
import sys
# Package for control terminal I/O settings (needed for raw keyboard input)
import termios
# Package for change the terminal mode to RAW mode
import tty

class ControlRobotTeleop( Node ):
    def __init__( self ):
        # Inherit from Node again( super() ). Create the node name in "__name__"
        super().__init__( "controller_teleop_node" )

        # Create publisher for publish mode control robot => condition in self.create_publisher( type of variable, name of the topic, queue size )
        self.publisherModeControl = self.create_publisher( Int8, "/micro_ros_mode_robot", 10 )

        # Display the mode of robot
        self.get_logger().info( "Set the robot to mode 2: Robot Control Teleop" )

        # Create publisher for publish cmd_vel => condition in self.create_publisher( type of variable, name of the topic, queue size )
        self.publisherVelocity = self.create_publisher( Twist, "/micro_ros_cmd_vel", 10 )

        # Publish every x times => parameter in self.create_timer( time, callback function )
        self.timerVelocity = self.create_timer( 0.5, self.timer_callback )

        # Display the debug of start publish
        self.get_logger().info( " Start publish velocity control... " )
        # Display the keyboard
        self.get_logger().info( "Control Your Robot" )
        self.get_logger().info( "---------------------------" )
        self.get_logger().info( "w : forward" )
        self.get_logger().info( "s : backward" )
        self.get_logger().info( "a : turn left" )
        self.get_logger().info( "d : turn right" )


    # Create funciton to recieve keys from keyboard
    def get_key( self ):

        # Get the file descriptor for standard input (keyboard)
        fd = sys.stdin.fileno()
        

        # Save the current terminal settings so we can restore them later
        old_settings = termios.tcgetattr(fd)

        try:
            # Set terminal to RAW mode
            # This allows reading a key immediately without pressing Enter
            tty.setraw(fd)
            
            # Read exactly one character from the keyboard
            key = sys.stdin.read(1)

        finally:
            # Restore the original terminal settings
            # Important so the terminal works normally after the program
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        # Return the pressed key
        return key
        

    def timer_callback( self ):
        # Create the storage variable mode robot
        mode_robot = Int8()

        # Assign mode of contorl robot( 2: robot control teleop )
        mode_robot.data = 2

        # Publish the mode to topic
        self.publisherModeControl.publish( mode_robot )

        # Create the storage variable cmd_vel
        cmd_vel_variable = Twist()

        # Create start varialbe of standing still
        cmd_vel_variable.linear.x = 0.0
        cmd_vel_variable.angular.z = 0.0

        # Coding for reading the key!!
        # Read one key from the keyboard
        key = self.get_key()

        # Function for checking the press key
        if( ( key == "w" ) or ( key == "W" ) ):
            cmd_vel_variable.linear.x = 1.0
        elif( ( key == "s" ) or ( key == "S" ) ):
            cmd_vel_variable.linear.x = -1.0
        elif( ( key == "a" ) or ( key == "A" ) ):
            cmd_vel_variable.angular.z = 1.0
        elif( ( key == "d" ) or ( key == "D" ) ):
            cmd_vel_variable.angular.z = -1.0
        else:
            cmd_vel_variable.linear.x = 0.0
            cmd_vel_variable.angular.z = 0.0

        # Display the debug of controlling
        self.get_logger().info( f"The value of linear: { cmd_vel_variable.linear.x } and angular: { cmd_vel_variable.angular.z }" )
        
        # Publish the cmd_vel to topic
        self.publisherVelocity.publish( cmd_vel_variable )



def main( args = None ):
    # Inital ros2 to comunicate and feature by using parameter from args(Start Comunication)
    rclpy.init( args = args )
    
    # Coding here
    # Create the variable to store NodeClass( It was use OOP )
    nodeToCall = ControlRobotTeleop()
    
    # Loop the excution on the terminal. In condition of spin pass variable to continues
    rclpy.spin( nodeToCall )
    
    # Destroy and end everything inside the node( End communication )
    nodeToCall.destroy_node()
    rclpy.shutdown()
