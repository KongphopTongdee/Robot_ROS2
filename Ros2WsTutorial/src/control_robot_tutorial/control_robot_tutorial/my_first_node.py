#!/usr/bin/env python3
# Interpreter line define to use python3( get the shebang to use python run )

import rclpy
from rclpy.node import Node

# Create the NodeClass( only for programe purpose ) which inherit( can access all function in rclpy.node ros2 ) from the rclpy
class MyNode( Node ):
    def __init__( self ):
        # Inherit from Node again( super() ). Create the node name in "__name__"
        super().__init__("first_node")

        # Create counter of iteral spin
        self.counter_ = 0

        # Display of ROS2 function( in terminal ) once times
        self.get_logger().info( "Hello ROS2, once times" )

        # Display of ROS2 function( in terminal ) every xx seconds
        self.timer = self.create_timer( 1.0, self.timer_callback )

    def timer_callback( self ):
        self.get_logger().info( "Hello ROS2, everytime " + str( self.counter_ ) )
        self.counter_ += 1

def main( args = None ):
    # Inital ros2 to comunicate and feature by using parameter from args(Start Comunication)
    rclpy.init( args = args )

    # Coding here
    # Create the variable to store NodeClass( It was use OOP )
    node = MyNode()

    # Loop the excution on the terminal. In parameter of spin pass variable to continues
    rclpy.spin( node )
    # Destroy and end everything inside the node( End communication )
    rclpy.shutdown()


# Use for run execution this file only couldn't import file from another node
if __name__ == '__main__':
    main()