#!/usr/bin/env python3
# Interpreter line define to use python3( get the shebang to use python run )

import rclpy
from rclpy.node import Node
# Need to add the package geometry_msgs in the package.xml file
from geometry_msgs.msg import Twist

# Create the NodeClass( only for programe purpose ) which inherit( can access all function in rclpy.node ros2 ) from the rclpy
class DrawCircleNode( Node ):
    def __init__( self ):
        # Inherit from Node again( super() ). Create the node name in "__name__"
        super().__init__("draw_circle_node")

        # Create the publisher => parameter in self.create_publisher( type of variable, name of the topic, queue size )
        self.cmd_vel_pub_ = self.create_publisher( Twist, "/turtle1/cmd_vel", 10 )

        # Publish every x times => parameter in self.create_timer( time, callback function )
        self.timer_ = self.create_timer( 0.5, self.send_velocity_command )

        # Debug in terminal this code has been run
        self.get_logger().info( "Draw circle node has been started" )

    def send_velocity_command( self ):
        # Create storage variable of Twist
        msg = Twist()

        # Make turtle motivation into the circle in 2D, Need to move straight with linear.x and rotate with angular.z
        msg.linear.x = 2.0
        msg.angular.z = 1.0

        # Publish the msg 
        self.cmd_vel_pub_.publish( msg )

def main( args = None ):
    # Inital ros2 to comunicate and feature by using parameter from args(Start Comunication)
    rclpy.init( args = args )

    # Coding here
    # Create the variable to store NodeClass( It was use OOP )
    nodeCall = DrawCircleNode()

    # Loop the excution on the terminal. In parameter of spin pass variable to continues
    rclpy.spin( nodeCall )

    # Destroy and end everything inside the node( End communication )
    nodeCall.destroy_node()
    rclpy.shutdown()
