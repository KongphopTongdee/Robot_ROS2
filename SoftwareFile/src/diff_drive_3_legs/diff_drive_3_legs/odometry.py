#!/usr/bin/env python3
#
# Copyright (C) 2012 Jon Stephan.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# ----------------------------------
# Portions of this code borrowed from the arbotix_python diff_controller.
#
# diff_controller.py - controller for a differential drive
# Copyright (c) 2010-2011 Vanadium Labs LLC.  All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of Vanadium Labs LLC nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Package ROS2 with python
import rclpy
# Package ROS2 for create Node 
from rclpy.node import Node

# Package mathmatics
from math import sin, cos, pi 

# Package for transform board caster ( including /tf and /tf_static )
from tf2_ros import TransformBroadcaster

# Package storage variable 
# Store the velocity vector
from geometry_msgs.msg import Twist
# Store the odemetry information 
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
# Store the int (storage capacity of pulse encoder)
from std_msgs.msg import Int16

# Declare the global variable 
NS_TO_SEC = 1000000000

# Create the node 
class DiffDriveTF( Node ):
    def __init__( self ): 
        # Inherit from Node again( super() ). Create the node name in "__name__"
        super().__init__( "diff_drive_tf" )   

        # Output the statement
        self.get_logger().info( "Complete robot bringup, waiting for publish odom..." )

        # Create function for subscription left/right value from encoder
        self.subscriptionLWheel_encoder = self.create_subscription( Int16, "/left_wheel_encoder", self.lWheel_callback, 10 )
        self.subscriptionsRwheel_encoder = self.create_subscription( Int16, "/right_wheel_encoder", self.rWheel_callback, 10 )

        # Create function for publish odometry
        self.publisherOdometry = self.create_publisher( Odometry, "/odometry", 10 )
        # Create timer for publisher 
        self.rate_hz = self.declare_parameter( "rate_hz", 10.0 ).value
        self.timer_ = self.create_timer( 1.0 / self.rate_hz , self.update )

        # Create odometry and transform variable 
        self.odom_broadcaster = TransformBroadcaster( self )
        self.base_frame_id = self.declare_parameter( "base_frame_id", "base_link" ).value
        self.odom_frame_id = self.declare_parameter( "odom_frame_id", "odom" ).value

        # Deaclare the storage variable
        # Storage the left/right encoder
        self.left_encoder = None 
        self.right_encoder = None 
        # Store the position of X and Y 
        self.xPosition = 0.0
        self.yPosition = 0.0
        # Store the previous time stamp
        self.previous_time = self.get_clock().now()


    def update( self ):
        # Update the current time 
        currentTime = self.get_clock().now()
        # Create the sample time 
        sample_time = currentTime - self.previous_time
        # Update the current time to previous time
        self.previous_time = currentTime
        # Convert nanosec into sec
        sample_time = sample_time.nanoseconds / NS_TO_SEC

        # Calculate odometry here!!!



        # Declare the quaternion variable
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(  )
        quaternion.w = cos(  )

        # Declare transform stamped variable
        transformStampedMsg = TransformStamped()
        transformStampedMsg.header.stamp = self.get_clock().now().to_msg()
        transformStampedMsg.header.frame_id = self.odom_frame_id
        transformStampedMsg.child_frame_id = self.base_frame_id
        transformStampedMsg.transform.translation.x = self.xPosition
        transformStampedMsg.transform.translation.y = self.yPosition
        transformStampedMsg.transform.translation.z = 0.0
        transformStampedMsg.transform.rotation.x = quaternion.x
        transformStampedMsg.transform.rotation.y = quaternion.y
        transformStampedMsg.transform.rotation.z = quaternion.z
        transformStampedMsg.transform.rotation.w = quaternion.w

        # Boardcaster the /tf and /tf_static
        self.odom_broadcaster.sendTransform( transformStampedMsg )

        # Decleare the odometry variable 
        odom = Odometry()
        odom.header.stamp = currentTime.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = 
        odom.pose.pose.position.y =
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.twist.twist.linear.x =
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 
        self.publisherOdometry.publish( odom )


    def lWheel_callback( self ):
        pass


    def rWheel_callback( self ):
        pass












# Create main function to call 
def main( args = None ):
    # Inital ros2 to comunicate and feature by using parameter from args(Start Comunication)
    rclpy.init( args = args )
    
    # Coding here
    # Create the variable to store NodeClass( It was use OOP )
    nodeToCall = DiffDriveTF()
    
    # Loop the excution on the terminal. In condition of spin pass variable to continues
    rclpy.spin( nodeToCall )
    
    # Destroy and end everything inside the node( End communication )
    nodeToCall.destroy_node()
    rclpy.shutdown()


# If there wasn't any call from another file, this won't be error.
if __name__ == "__main__":
    main()
