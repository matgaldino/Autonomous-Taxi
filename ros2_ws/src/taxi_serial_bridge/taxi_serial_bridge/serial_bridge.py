import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
from copy import deepcopy

from sensor_msgs.msg import Imu, MagneticField
from taxi_interfaces.msg import WheelTicks

import serial


class SerialBridge(Node):

    def __init__(self):
        super().__init__('serial_bridge')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('command_timeout', 0.3)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        self.ser = serial.Serial(port, baudrate, timeout=0, write_timeout=0.02)
        self.rx_buffer = bytearray()

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

        self.wheel_pub = self.create_publisher(
            WheelTicks, 'wheel_ticks', 10
        )

        self.imu_pub = self.create_publisher(
            Imu, 'imu/data_raw', 10
        )

        self.mag_pub = self.create_publisher(
            MagneticField, 'imu/mag', 10
        )

        self.rx_timer = self.create_timer(
            0.01, self.read_telemetry
        )

        self.previous_ticks = None
        self.stationary_count = 0

        self.imu_ekf_pub = self.create_publisher(
            Imu, 'imu/data_ekf', 10
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

    def read_telemetry(self):
        available = self.ser.in_waiting

        if available == 0:
            return

        self.rx_buffer.extend(
            self.ser.read(min(available, 4096))
        )

        while b'\n' in self.rx_buffer:
            line, _, remaining = self.rx_buffer.partition(b'\n')
            self.rx_buffer = bytearray(remaining)

            if line.startswith(b'T,'):
                self.process_telemetry(
                    line.decode('ascii', errors='replace').strip()
                )

        if len(self.rx_buffer) > 256:
            self.rx_buffer.clear()

    def process_telemetry(self, line):
        fields = line.split(',')

        if len(fields) != 15:
            return

        try:
            (
                timestamp_ms,
                left_ticks, right_ticks,
                imu_valid,
                ax, ay, az,
                gx, gy, gz,
                mag_valid,
                mx, my, mz
            ) = map(int, fields[1:])

        except ValueError:
            return

        if not 0 <= timestamp_ms <= 0xFFFFFFFF:
            return

        ticks = (left_ticks, right_ticks)

        if ticks == self.previous_ticks:
            self.stationary_count += 1
        else:
            self.stationary_count = 0

        self.previous_ticks = ticks

        if imu_valid not in (0, 1) or mag_valid not in (0, 1):
            return

        if not all(-2**63 <= v < 2**63
                for v in (left_ticks, right_ticks)):
            return

        stamp = self.get_clock().now().to_msg()

        # Wheel encoders
        wheel = WheelTicks()
        wheel.header.stamp = stamp
        wheel.header.frame_id = 'base_link'

        wheel.timestamp_ms = timestamp_ms
        wheel.left_ticks = left_ticks
        wheel.right_ticks = right_ticks

        self.wheel_pub.publish(wheel)

        # Accelerometer and gyroscope
        if imu_valid:
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = 'imu_link'

            accel_scale = 9.80665 / 16384.0
            gyro_scale = math.pi / (180.0 * 131.0)

            imu.linear_acceleration.x = ax * accel_scale
            imu.linear_acceleration.y = ay * accel_scale
            imu.linear_acceleration.z = az * accel_scale

            imu.angular_velocity.x = gx * gyro_scale
            imu.angular_velocity.y = gy * gyro_scale
            imu.angular_velocity.z = gz * gyro_scale

            # Orientation is not estimated yet
            imu.orientation_covariance[0] = -1.0

            imu.angular_velocity_covariance = [
                0.0004, 0.0,    0.0,
                0.0,    0.0004, 0.0,
                0.0,    0.0,    0.0004
            ]

            self.imu_pub.publish(imu)
            imu_ekf = deepcopy(imu)

            if (
                self.stationary_count >= 5
                and abs(imu.angular_velocity.z) < 0.02
            ):
                imu_ekf.angular_velocity.z = 0.0

            self.imu_ekf_pub.publish(imu_ekf)

        # Magnetometer
        if mag_valid:
            mag = MagneticField()
            mag.header.stamp = stamp
            mag.header.frame_id = 'imu_link'

            mag_scale = 0.15e-6

            # Align AK8963 axes with MPU9250 axes
            mag.magnetic_field.x = my * mag_scale
            mag.magnetic_field.y = mx * mag_scale
            mag.magnetic_field.z = -mz * mag_scale

            self.mag_pub.publish(mag)


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