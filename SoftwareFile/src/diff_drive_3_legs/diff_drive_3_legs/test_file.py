# !/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class TestNode( Node ):
    def __init__( self ):
        super().__init__( "test_node_only" )
        # self.counter_ = 0.0
        # self.test_publish_ = self.create_publisher( Twist, "/micro_ros_arduino_twist_subscriber", 10 )
        # self.timer_of_publish = self.create_timer( 1.0, self.publisher_callback )
        # self.get_logger().info( " Start publish value... " )

        self.test_subscriber_ = self.create_subscription( String, "/micro_ros_error_status", self.subscription_callback, 10  )
        self.get_logger().info( "Start subscribe value..." )

    # def publisher_callback( self ):
    #     robotVelocity = Twist()
    #     robotVelocity.linear.x = self.counter_
    #     robotVelocity.angular.z = self.counter_

    #     self.test_publish_.publish( robotVelocity )
        
    #     self.get_logger().info( f"Now publish value : { str( self.counter_ ) }" )

    #     self.counter_ += 1.0

    def subscription_callback( self, msg: String ):
        recieveMsg = msg.data

        self.get_logger().info( f" Message from micro_ros_error_status: { recieveMsg } " )

def main( args = None ):
    rclpy.init( args = args )
    nodeToCall = TestNode()
    rclpy.spin( nodeToCall )
    nodeToCall.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()

# ------------------------------------------------------------------------------------------

