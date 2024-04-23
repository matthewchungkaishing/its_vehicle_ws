import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimpleTeleopPublisher(Node):
    def __init__(self):
        super().__init__('simple_teleop_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    simple_teleop_publisher = SimpleTeleopPublisher()
    try:
        rclpy.spin(simple_teleop_publisher)
    except KeyboardInterrupt:
        print("Terminating node")
        simple_teleop_publisher.destroy_node()

if __name__ == "__main__":
    main()
