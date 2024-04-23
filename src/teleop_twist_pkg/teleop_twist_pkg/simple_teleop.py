import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput.keyboard import Key, Listener, KeyCode

# This dictionary maps keys to specific movement commands.
key_mapping = {
    'w': (1.0, 0.0),  # Move forward
    's': (-1.0, 0.0), # Move backward
    'a': (0.0, 1.0),  # Turn left
    'd': (0.0, -1.0), # Turn right
    'stop': (0.0, 0.0) # Stop
}

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.last_key_pressed = 'stop'
        self.listener = Listener(on_press=self.on_key_press, on_release=self.on_key_release)
        self.listener.start()
        # Create a timer to publish Twist messages at regular intervals
        self.timer = self.create_timer(0.1, self.timer_callback)

    def on_key_press(self, key):
        if hasattr(key, 'char') and key.char in key_mapping:
            self.last_key_pressed = key.char
            self.get_logger().info(f'Key pressed: {key.char}')
        elif key == Key.esc:
            self.get_logger().info('Stopping...')
            # Stop the listener
            return False

    def on_key_release(self, key):
        if hasattr(key, 'char') and key.char == self.last_key_pressed:
            self.last_key_pressed = 'stop'
            self.get_logger().info(f'Key released: {key.char}')

    def timer_callback(self):
        twist = Twist()
        linear, angular = key_mapping[self.last_key_pressed]
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher_.publish(twist)
        self.get_logger().info(f'Publishing Twist: linear.x={linear}, angular.z={angular}')


def main(args=None):
    try:
        rclpy.init(args=args)
        keyboard_teleop = KeyboardTeleop()
        rclpy.spin(keyboard_teleop)
    except KeyboardInterrupt:
        print("Terminating node")
        keyboard_teleop.destroy_node()

if __name__ == '__main__':
    main()
