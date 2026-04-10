#!/usr/bin/env python3
# Interpreter line define to use python3( get the shebang to use python run )

# Package ROS2 with python
import rclpy
# Package ROS2 for create Node
from rclpy.node import Node
# Package mathematic calculation
from math import pi

# ---------- Imoport library ----------
# Need to add the package below in the package.xml file
# Package for message type int16
from std_msgs.msg import Int16

# Package for message joint state publisher
from sensor_msgs.msg import JointState


class ControlRobotTeleop( Node ):
    def __init__( self ):
        # Inherit from Node again( super() ). Create the node name in "__name__"
        super().__init__( "joint_state_publisher_node" )

        # Display the mode of robot
        self.get_logger().info( "Sending the joint state of the wheel" )


        # ---------- Publisher ----------
        # Create publisher for publish joint state publisher => condition in self.create_publisher( type of variable, name of the topic, queue size )
        self.publisherJointState = self.create_publisher( JointState, "/joint_states", 10 )
        # Publish every x times => parameter in self.create_timer( time, callback function ) 
        self.rate_hz = self.declare_parameter( "rate_hz", 10.0 ).value
        self.timerPublihser = self.create_timer( 1.0 / self.rate_hz , self.timer_callback )


        # ---------- Subscription ----------
        # Create function for subscription left/right value from encoder ( only the counter )
        self.subscription_speed_left = self.create_subscription( Int16, "/micro_ros_left_enc_tick_data", self.lWheel_callback, 10 )
        self.subscription_speed_right = self.create_subscription( Int16, "/micro_ros_right_enc_tick_data", self.rWheel_callback, 10 )


        # ---------- Necessary variable ----------
        self.left_encoder_accumulate_data = 0.0 
        self.right_encoder_accumulate_data = 0.0 

        # Declare the limit of encoder( refer from the maximum pulse in one sec = 1984 pulse/sec )
        self.encoder_min = self.declare_parameter( 'encoder_min', -2048 ).value
        self.encoder_max = self.declare_parameter( 'encoder_max', 2048 ).value

        # Declare the wrap boundary from encoder 
        self.encoder_low_wrap = self.declare_parameter( 'wheel_low_wrap', ( self.encoder_max - self.encoder_min )*0.3 + self.encoder_min ).value
        self.encoder_high_wrap = self.declare_parameter( 'wheel_high_wrap', ( self.encoder_max - self.encoder_min )*0.7 + self.encoder_min ).value

        # Declare the storage round of rotation of the encoder
        self.left_iteration = 0
        self.right_iteration = 0
        self.left_prev_tick_encoder = 0
        self.right_prev_tick_encoder = 0

        # Declare the storage pulse per 1 meter value
        self.pulse_per_revolution = 480
        

    def timer_callback( self ):
        # Create the currTime
        currentTime = self.get_clock().now()


        # ---------- Calculation here ----------
        # Calculate the angle of left and right wheel
        wheel_left_angle = ( self.left_encoder_accumulate_data / self.pulse_per_revolution ) * 2 * pi
        wheel_right_angle = ( self.right_encoder_accumulate_data / self.pulse_per_revolution ) * 2 * pi


        # ---------- Publish the value ----------
        # Create the JointState object
        msg = JointState()
        # Assign the time in JointState object
        msg.header.stamp = currentTime.to_msg()
        # Assign the name in JointState object
        msg.name = [ "lwheel_joint","rwheel_joint","cylinderRplidar_joint" ]
        # Assign the position in JointState object
        msg.position = [ wheel_left_angle, wheel_right_angle, 0.0 ]
        # Publish the output value 
        self.publisherJointState.publish( msg )


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
