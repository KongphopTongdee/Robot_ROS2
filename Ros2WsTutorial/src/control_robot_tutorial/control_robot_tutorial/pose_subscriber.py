#!/usr/bin/env python3
# Interpreter line define to use python3( get the shebang to use python run )

import rclpy
from rclpy.node import Node
# Need to add the package turtlesim in the package.xml file
from turtlesim.msg import Pose

# Create the NodeClass( only for programe purpose ) which inherit( can access all function in rclpy.node ros2 ) from the rclpy
class PoseSubscriberNode( Node ):
    def __init__( self ):
        # Inherit from Node again( super() ). Create the node name in "__name__"
        super().__init__( "pose_subscriber_node" )
        
        # Create the subscriber => parameter in self.create_subscriber( type of variable, name of the topic, callback function, queue size )
        self.pose_subscriber_ = self.create_subscription( Pose, "/turtle1/pose", self.pose_callback, 10 )

    # Create callback function => with the parameter receive msg(type of msg was pose)
    def pose_callback( self, msg: Pose ):
        # Get only postion X and Y
        xPosition = msg.x
        yPosition = msg.y
        
        # Display the output of position X and Y
        self.get_logger().info( f"Get value from turtle X: { str( xPosition ) } ,Y: { str( yPosition ) } ")

def main( args = None ):
    # Inital ros2 to comunicate and feature by using parameter from args(Start Comunication)
    rclpy.init( args = args )
    
    # Coding here
    # Create the variable to store NodeClass( It was use OOP )
    nodeCall = PoseSubscriberNode()
    
    # Loop the excution on the terminal. In parameter of spin pass variable to continues
    rclpy.spin( nodeCall )
    
    # Destroy and end everything inside the node( End communication )
    nodeCall.destroy_node()
    rclpy.shutdown()
