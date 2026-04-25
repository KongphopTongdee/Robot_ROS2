# !/usr/bin/env python3
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
from math import sin, cos

# Package for transform board caster ( including /tf and /tf_static )
from tf2_ros import TransformBroadcaster

# Package storage variable 
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


        # ---------- Subscription ----------
        # Create function for subscription left/right value from encoder ( only the counter )
        self.subscription_speed_left = self.create_subscription( Int16, "/micro_ros_left_enc_tick_data", self.lWheel_callback, 10 )
        self.subscription_speed_right = self.create_subscription( Int16, "/micro_ros_right_enc_tick_data", self.rWheel_callback, 10 )


        # ---------- Publisher ----------
        # Create function for publish odometry
        self.publisherOdometry = self.create_publisher( Odometry, "odom", 10 )
        # Create timer for publisher 
        self.rate_hz = self.declare_parameter( "rate_hz", 10.0 ).value
        self.timer_ = self.create_timer( 1.0 / self.rate_hz , self.update )


        # Create odometry and transform variable 
        self.odom_broadcaster = TransformBroadcaster( self )
        self.base_frame_id = self.declare_parameter( "base_frame_id", "base_link" ).value
        self.odom_frame_id = self.declare_parameter( "odom_frame_id", "odom" ).value


        # ---------- Necessary variable ----------
        # Declare the storage pulse per 1 meter value
        # self.ticks_meter = float( self.declare_parameter( 'ticks_meter', 5136 ).value )
        self.ticks_meter = float( self.declare_parameter( 'ticks_meter', 1984 ).value )

        # Storage the left/right raw tick data from encoder( for calculate the position of odometry )
        self.left_encoder_accumulate_data = 0.0 
        self.right_encoder_accumulate_data = 0.0 
        self.prev_left_encoder_data = None
        self.prev_right_encoder_data = None

        # Declare the limit of encoder( refer from the maximum pulse in one sec = 1984 pulse/sec )
        self.encoder_min = self.declare_parameter( 'encoder_min', -2048 ).value
        self.encoder_max = self.declare_parameter( 'encoder_max', 2048 ).value
        # self.encoder_min = self.declare_parameter( 'encoder_min', -32768 ).value
        # self.encoder_max = self.declare_parameter( 'encoder_max', 32768 ).value

        # Declare the wrap boundary from encoder 
        self.encoder_low_wrap = self.declare_parameter( 'wheel_low_wrap', ( self.encoder_max - self.encoder_min )*0.3 + self.encoder_min ).value
        self.encoder_high_wrap = self.declare_parameter( 'wheel_high_wrap', ( self.encoder_max - self.encoder_min )*0.7 + self.encoder_min ).value

        # Declare the storage round of rotation of the encoder
        self.left_iteration = 0
        self.right_iteration = 0
        self.left_prev_tick_encoder = 0
        self.right_prev_tick_encoder = 0

        # Store the position of X and Y 
        self.xPosition = 0.0
        self.yPosition = 0.0
        self.zPosition = 0.0

        # Store the previous time stamp
        self.previous_time = self.get_clock().now()

        # Robot parameter( declare in context for ros2 )
        self.width_robot = float( self.declare_parameter( 'width_robot', 0.398 ).value )
        # self.wheel_diameter = float( self.declare_parameter( 'wheel_diameter', 0.123 ).value )
        # self.pulse_per_revolution = float( self.declare_parameter( 'pulse_per_revolution', 480 ).value )

        # Declare store velocity of wheel left and right value
        self.distance_left = 0.0
        self.distance_right = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0 
        self.forward_distance_robot = 0.0
        self.rotation_robot = 0.0


    def update( self ):
        # Update the current time 
        currentTime = self.get_clock().now()
        # Create the sample time 
        sample_time = currentTime - self.previous_time
        # Update the current time to previous time
        self.previous_time = currentTime
        # Convert nanosec into sec
        sample_time = sample_time.nanoseconds / NS_TO_SEC


        # ---------- Calculate odometry here ----------
        # Check if there wasn't any value in self.prev_left_encoder_data
        # Equation for calculate velocity ; v = ( deltaTick )
        if( self.prev_left_encoder_data is not None ):
            self.distance_left = ( self.left_encoder_accumulate_data - self.prev_left_encoder_data ) / ( self.ticks_meter )
            self.distance_right = ( self.right_encoder_accumulate_data - self.prev_right_encoder_data ) / ( self.ticks_meter )
        self.prev_left_encoder_data = self.left_encoder_accumulate_data
        self.prev_right_encoder_data = self.right_encoder_accumulate_data


        # Calculate forward velocity of robot ( From equation distance = ( distance_r + distance_l ) / 2 )
        self.forward_distance_robot = ( self.distance_right + self.distance_left ) / 2 
        # Calculate rotation velocity of robot small angel ( From equation w = ( distance_r - distance_l ) / L ) ; L = distance between left and right wheel 
        rotation_robot = ( self.distance_right - self.distance_left ) / self.width_robot
        # Calculate acceration in one point using divide by time method ( other method was calculate acceration infinite time by using differential )
        self.linear_velocity = self.forward_distance_robot / sample_time
        self.angular_velocity = rotation_robot / sample_time

        # Check if there was velocity of robot
        if ( self.forward_distance_robot != 0 ):
            # Calculate new position in x and y ordinate
            newX = cos( rotation_robot ) * self.forward_distance_robot
            newY = -sin( rotation_robot ) * self.forward_distance_robot
            # Update the final position of robot
            self.xPosition = self.xPosition + ( cos( self.rotation_robot ) * newX - sin( self.rotation_robot ) * newY )
            self.yPosition = self.yPosition + ( sin( self.rotation_robot ) * newX + cos( self.rotation_robot ) * newY )
        if ( rotation_robot != 0 ):
            self.rotation_robot = self.rotation_robot + rotation_robot

        # ---------- Assign the odometry value here ----------
        # Declare the quaternion variable
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin( self.rotation_robot / 2 )
        quaternion.w = cos( self.rotation_robot / 2 )

        # Declare transform stamped variable
        transformStampedMsg = TransformStamped()
        transformStampedMsg.header.stamp = self.get_clock().now().to_msg()
        transformStampedMsg.header.frame_id = self.odom_frame_id
        transformStampedMsg.child_frame_id = self.base_frame_id
        transformStampedMsg.transform.translation.x = self.xPosition
        transformStampedMsg.transform.translation.y = self.yPosition
        transformStampedMsg.transform.translation.z = self.zPosition
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
        odom.pose.pose.position.x = self.xPosition
        odom.pose.pose.position.y = self.yPosition
        odom.pose.pose.position.z = self.zPosition
        odom.pose.pose.orientation = quaternion
        odom.twist.twist.linear.x = self.linear_velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.angular_velocity
        self.publisherOdometry.publish( odom )


    # Create the subscription left_speed_pulse
    def lWheel_callback( self, msg ):
        # Create variable for encoder raw data
        encoder_tick_data = msg.data

        # ---------- Coding wrap boundary here ----------
        # Check if the value was more than limit convert into stack and continue count.
        if( ( encoder_tick_data < self.encoder_low_wrap ) and ( self.left_prev_tick_encoder > self.encoder_high_wrap ) ):
            self.left_iteration += 1
        # Check if the value was less than limit convert into stack and continue count.
        if( ( encoder_tick_data > self.encoder_high_wrap ) and ( self.left_prev_tick_encoder < self.encoder_low_wrap ) ):
            self.left_iteration -= 1

        # Assign the accumulate value here using wrap boundary.
        self.left_encoder_accumulate_data = ( 1.0 * ( encoder_tick_data + self.left_iteration * ( self.encoder_max - self.encoder_min ) ) )
        # Store the previous value.
        self.left_prev_tick_encoder = encoder_tick_data

    # Create the subscription right_speed_pulse
    def rWheel_callback( self, msg ):
        encoder_tick_data = msg.data

        # ---------- Coding wrap boundary here ----------
        # Check if the value was more than limit convert into stack and continue count.
        if( ( encoder_tick_data < self.encoder_low_wrap ) and ( self.right_prev_tick_encoder > self.encoder_high_wrap ) ):
            self.right_iteration += 1
        # Check if the value was less than limit convert into stack and continue count.
        if( ( encoder_tick_data > self.encoder_high_wrap ) and ( self.right_prev_tick_encoder < self.encoder_low_wrap ) ):
            self.right_iteration -= 1

        # Assign the accumulate value here using wrap boundary.
        self.right_encoder_accumulate_data = ( 1.0 * ( encoder_tick_data + self.right_iteration * ( self.encoder_max - self.encoder_min ) ) )
        # Store the previous value.
        self.right_prev_tick_encoder = encoder_tick_data


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



# Relationship: Pulse <-> RPM <-> m/s
# PPR = pulse per revolution
# R = wheel radius
# 1.RPM = ( (pulse/sec) / PPR ) * 60
# 2.v = ( RPM / 60 ) * ( 2 * pi * R )
# 3.v = ( (pulse/sec) / PPR ) * ( 2 * pi * R )


