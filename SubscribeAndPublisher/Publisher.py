import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Import Node : define th Topic to send
# Import Twist : storage cmd_vel( linear.x and angular.z )

class CmdVelPublisher( Node ):
    def __init__( self ):
        super().__init__( 'cmd_vel_publisher' )
        # definition of .create_publisher( msg_type, topic_name, qos_profile )
        self.StoreValuePublish = self.create_publisher( Twist, 'cmd_vel', 10 )
        # definition of .create_timer( iteration time, function for publish )
        self.timer = self.create_timer( 0.5, self.publish_cmd )

    def publish_cmd( self ):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.2
        # publish this message
        self.StoreValuePublish.publish( msg )
        # check with log while running this python file
        self.get_logger().info( 'Complete publish cmd_vel' )

def main( args = None ):
    rclpy.init( args = args )
    node = CmdVelPublisher()
    rclpy.spin( node )
    node.destroy_node()
    rclpy.shutdown()