import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial


class SerialBridge(Node):

    def __init__(self):
        super().__init__('serial_bridge')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('command_timeout', 0.3)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        self.ser = serial.Serial(port, baudrate, timeout=0.1)

        self.steering = 0
        self.throttle = 0

        self.last_cmd_time = self.get_clock().now()

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # 20 Hz
        self.timer = self.create_timer(0.05, self.send_command)

        self.get_logger().info(
            f'Serial bridge connected to {port} at {baudrate} baud'
        )

    def cmd_vel_callback(self, msg):
        # temporary normalized mapping:
        # angular.z [-1, 1] -> steering [-1000, 1000]
        # linear.x  [-1, 1] -> throttle [-1000, 1000]

        angular = max(-1.0, min(1.0, msg.angular.z))
        linear = max(-1.0, min(1.0, msg.linear.x))

        self.steering = int(angular * 1000)
        self.throttle = int(linear * 1000)

        self.last_cmd_time = self.get_clock().now()

    def send_command(self):
        timeout = self.get_parameter('command_timeout').value

        elapsed = (
            self.get_clock().now() - self.last_cmd_time
        ).nanoseconds / 1e9

        if elapsed > timeout:
            steering = 0
            throttle = 0
        else:
            steering = self.steering
            throttle = self.throttle

        command = f'C,{steering},{throttle}\n'

        self.ser.write(command.encode('ascii'))

    def destroy_node(self):
        if self.ser.is_open:
            self.ser.write(b'C,0,0\n')
            self.ser.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = SerialBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()