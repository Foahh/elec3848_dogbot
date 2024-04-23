import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist

class StampAdder(Node):
    def __init__(self):
        super().__init__("StampAdderNode")
        
        self.publisher = self.create_publisher(TwistStamped, "/dogbot_base_controller/cmd_vel", 10)
        self.subscription = self.create_subscription(Twist, "/cmd_vel_raw", self.callback, 10)
        
    def callback(self, msg):
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.twist = msg
        self.publisher.publish(twist_stamped)
        
    
def main(args=None):
    rclpy.init(args=args)
    node = StampAdder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    

