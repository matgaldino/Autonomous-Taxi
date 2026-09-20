import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class GamepadTeleop(Node):
    def __init__(self):
        super().__init__('gamepad_teleop')

        self.declare_parameter('steering_axis', 0)
        self.declare_parameter('lt_axis', 2)
        self.declare_parameter('rt_axis', 5)
        self.declare_parameter('deadman_button', 4)

        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('throttle_deadzone', 0.05)
        self.declare_parameter('joy_timeout', 0.5)

        self.steering_axis = self.get_parameter('steering_axis').value
        self.lt_axis = self.get_parameter('lt_axis').value
        self.rt_axis = self.get_parameter('rt_axis').value
        self.deadman_button = self.get_parameter('deadman_button').value

        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.throttle_deadzone = self.get_parameter('throttle_deadzone').value
        self.joy_timeout = self.get_parameter('joy_timeout').value

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.last_joy_time = None
        self.joy_timed_out = False
        self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)

        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.get_logger().info('Gamepad teleop node started')

    def joy_callback(self, msg):
        self.last_joy_time = self.get_clock().now()

        if self.joy_timed_out:
            self.get_logger().info('Joystick connection restored')
            self.joy_timed_out = False

        cmd = Twist()

        # 8BitDo Ultimate 2C mapping
        steering = msg.axes[self.steering_axis] # Left stick horizontal
        lt = msg.axes[self.lt_axis] # Left trigger
        rt = msg.axes[self.rt_axis] # Right trigger
        deadman = msg.buttons[self.deadman_button]  # LB

        # ROS joy convention observed:
        # trigger released = +1
        # trigger pressed  = -1
        throttle = (lt - rt) / 2.0

        # Small dead zones
        # Already in joy_node
        # if abs(steering) < 0.05:
        #     steering = 0.0

        if abs(throttle) < self.throttle_deadzone:
            throttle = 0.0

        if deadman:
            cmd.linear.x = throttle * self.max_linear_speed
            cmd.angular.z = steering * self.max_angular_speed

        self.publisher.publish(cmd)

    def watchdog_callback(self):
        if self.last_joy_time is None:
            return

        elapsed = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9

        if elapsed > self.joy_timeout:
            self.publisher.publish(Twist())  # Publish zero velocity
            if not self.joy_timed_out:
                self.get_logger().warning('Joystick connection lost, stopping the vehicle')
                self.joy_timed_out = True

def main(args=None):
    rclpy.init(args=args)

    node = GamepadTeleop()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()