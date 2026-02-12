import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Import Node : define th Topic to send
# Import Twist : storage cmd_vel( linear.x and angular.z )

class CmdVelSubscriber( Node ):
    def __init__( self ):
        super().__init__( 'cmd_vel_subscriber' )
        self.SubscribeValue = self.create_subscription( 
            Twist,
            'cmd_vel',
            self.listener_callback,
            10 )
        
    def listener_callback( self, msg ):
        linear = msg.linear.x
        angular = msg.angular.z

        self.get_logger().info(
            f' Value of linear was { linear }, Value of angular was { angular } '
        )
        print( f" Value of linear was { linear }, Value of angular was { angular }" )

def main( args = None ):
    rclpy.init( args = args )
    node = CmdVelSubscriber()
    rclpy.spin( node )
    node.destroy_node()
    rclpy.shutdown()