#!/usr/bin/env python3
# Interpreter line define to use python3( get the shebang to use python run )

import rclpy
from rclpy.node import Node
# Need to add the package turtlesim in the package.xml file
# .msg for send data
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
# .srv for recieve client request
from turtlesim.srv import SetPen

from functools import partial

class TurtleControllerNode( Node ):
    def __init__( self ):
        # Inherit from Node again( super() ). Create the node name in "__name__"
        super().__init__( "turtle_controller" )

        # Create variable to store the position of turtle( use when turtle cross the middle area )
        self.previous_x_ = 0

        # Create publisher for publish cmd_vel => condition in self.create_publisher( type of variable, name of the topic, queue size )
        self.cmd_vel_publisher_ = self.create_publisher( Twist, "/turtle1/cmd_vel", 10 )

        # Create subscriber for recieve position X and Y => condition in self.create_subscriber( type of variable, name of the topic, callback function, queue size )
        self.pose_subscriber_ = self.create_subscription( Pose, "/turtle1/pose", self.pose_callback, 10 )

        # Create debug while running
        self.get_logger().info( "Turtle contoller has been started." )

    # Create the function to publish cmd_vel when subscribe the pose
    def pose_callback( self, position: Pose ):
        # Create storage cmd_vel variable
        cmd_vel_Variable = Twist()

        # Check when the turtle will hit the wall 
        if( ( position.x > 9.0 ) or ( position.x < 2.0 ) or ( position.y > 9.0 ) or ( position.y < 2.0 ) ):
            # Create U-turn turtle
            cmd_vel_Variable.linear.x = 1.0
            cmd_vel_Variable.angular.z = 0.9
        else :
            # Create turtle go straight
            cmd_vel_Variable.linear.x = 5.0
            cmd_vel_Variable.angular.z = 0.0

        # Publish the value in cmd_vel
        self.cmd_vel_publisher_.publish( cmd_vel_Variable )

        # If the position of turtle was on the right change the color of trace( pen )
        # meaning ( self.previous_x_ <= 5.5 ) was if turtle on the left side 
        if( ( position.x > 5.5) and ( self.previous_x_ <= 5.5 ) ):
            self.previous_x_ = position.x
            self.get_logger().info( "Set color to red!" )
            self.call_set_pen_serviece( 255, 0, 0, 3, 0 )
        # meaning ( self.previous_x_ > 5.5 ) was if turtle on the right side 
        elif( ( position.x <= 5.5) and ( self.previous_x_ > 5.5 ) ):
            self.previous_x_ = position.x
            self.get_logger().info( "Set color to green!" )
            self.call_set_pen_serviece( 0, 255, 0, 3, 0 )


    # Create function for recieve the client request set_pen
    def call_set_pen_serviece( self, red, green, blue, width, off ):
        # Create object to represent the client => parameter in the self.create_client( srvice type,  )
        client = self.create_client( SetPen, "/turtle1/set_pen" )

        # Create waiting service if there wasn't any serive show up on time( avoid error without service )
        while not client.wait_for_service( 1.0 ):
            # Use warning to change the article into yellow color
            self.get_logger().warn( "Waiting for service..." )

        # Create store variable ( type SetPen ) from the request( service call )
        request = SetPen.Request()
        # Assign value from the request into variable
        request.r = red
        request.g = green
        request.b = blue
        request.width = width
        request.off = off

        # Call service immediately( if using .call => there gonna wait until service has replied and while using spin it gonna block the thread example the response will nerver recieve)
        #                         ( if using .call_async => service immediately reply)
        future = client.call_async( request )

        # When the service has replied call the self.callback_set_pen function
        future.add_done_callback( partial( self.callback_set_pen ) )

    # This function will run if the service replied the response
    def callback_set_pen( self, future ):
        # Avoid the error
        try:
            # Send response back if it wasn't error
            response = future.result()
        # If the was the error, assume this error was exception and display the error out
        except Exception as e:
            self.get_logger().error( "Service call failed: %r" %( e, ) )

def main( args = None ):
    # Inital ros2 to comunicate and feature by using parameter from args(Start Comunication)
    rclpy.init( args = args )

    # Coding here
    # Create the variable to store NodeClass( It was use OOP )
    nodeCall = TurtleControllerNode()
    
     # Loop the excution on the terminal. In condition of spin pass variable to continues
    rclpy.spin( nodeCall )
    
    # Destroy and end everything inside the node( End communication )
    nodeCall.destroy_node()
    rclpy.shutdown
