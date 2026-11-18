import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from taxi_interfaces.msg import WheelTicks
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class WheelOdometry(Node):

    def __init__(self):
        super().__init__('wheel_odometry')

        self.declare_parameter('wheel_radius', 0.0305)
        self.declare_parameter('counts_per_rev', 2400)
        self.declare_parameter('left_offset', 0.0970)
        self.declare_parameter('right_offset', 0.0934)

        self.tf_broadcaster = TransformBroadcaster(self)

        radius = self.get_parameter('wheel_radius').value
        counts = self.get_parameter('counts_per_rev').value

        self.left_offset = self.get_parameter('left_offset').value
        self.right_offset = self.get_parameter('right_offset').value

        self.path_pub = self.create_publisher(
            Path, 'odom/path', 10
        )

        self.path = Path()
        self.path.header.frame_id = 'odom'

        self.track_width = self.left_offset + self.right_offset
        self.meters_per_tick = 2.0 * math.pi * radius / counts

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.previous = None

        self.publisher = self.create_publisher(
            Odometry, 'odom/wheel', 10
        )

        self.subscription = self.create_subscription(
            WheelTicks,
            'wheel_ticks',
            self.encoder_callback,
            10
        )

        # Trajectory publisher
        self.path_pub = self.create_publisher(Path, 'odom/path', 10)

        self.path = Path()
        self.path.header.frame_id = 'odom'

        self.path_timer = self.create_timer(0.5, self.publish_path)

    def encoder_callback(self, msg):

        current = (
            msg.timestamp_ms,
            msg.left_ticks,
            msg.right_ticks
        )

        if self.previous is None:
            self.previous = current
            return

        prev_time, prev_left, prev_right = self.previous

        self.previous = current

        # Handle uint32 timestamp wraparound
        dt_ms = (msg.timestamp_ms - prev_time) & 0xFFFFFFFF

        if dt_ms == 0 or dt_ms > 1000:
            return

        dt = dt_ms / 1000.0

        # Wheel displacements
        dl = (msg.left_ticks - prev_left) * self.meters_per_tick
        dr = (msg.right_ticks - prev_right) * self.meters_per_tick

        # Vehicle displacement
        ds = (
            self.right_offset * dl +
            self.left_offset * dr
        ) / self.track_width

        dtheta = (dr - dl) / self.track_width

        # Midpoint integration
        self.x += ds * math.cos(self.theta + dtheta / 2.0)
        self.y += ds * math.sin(self.theta + dtheta / 2.0)

        self.theta += dtheta

        self.theta = math.atan2(
            math.sin(self.theta),
            math.cos(self.theta)
        )

        # Odometry message
        odom = Odometry()

        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom.twist.twist.linear.x = ds / dt
        odom.twist.twist.angular.z = dtheta / dt

        # Temporary conservative covariances
        for i in range(6):
            odom.pose.covariance[i * 7] = 1e6
            odom.twist.covariance[i * 7] = 1e6

        self.publisher.publish(odom)

        transform = TransformStamped()

        transform.header = odom.header
        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(transform)



        # Update trajectory
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose

        self.path.header.stamp = odom.header.stamp

        if not self.path.poses or math.hypot(
            self.x - self.path.poses[-1].pose.position.x,
            self.y - self.path.poses[-1].pose.position.y
        ) >= 0.01:

            self.path.poses.append(pose)

            if len(self.path.poses) > 2000:
                self.path.poses.pop(0)

        
    def publish_path(self):
        if self.path.poses:
            self.path_pub.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometry()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()